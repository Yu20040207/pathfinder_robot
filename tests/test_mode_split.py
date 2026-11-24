"""开机=手动模式；模式栈按需启动，切模式先清理干净（2026-11-24 用户要求）。

结构：
  fwr-base.service  常驻硬件层（底盘/雷达×2/IMU/EKF/合并/车尾守卫）
  fwr-nav / fwr-map / fwr-follow  按需启动，起的时候 start_base:=false
  mqtt_bridge 的 'mode' 动作：确认硬件层在跑 → 停掉所有非目标模式的栈
                                （等 systemd 真报 inactive）→ 再启动目标栈
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'ros2_ws' / 'src' / 'fwr_bringup' / 'launch'
MAPPING = (LAUNCH / 'real_mapping.launch.py').read_text(encoding='utf-8')
NAV = (LAUNCH / 'real_navigation.launch.py').read_text(encoding='utf-8')
LOC = (LAUNCH / 'real_localization.launch.py').read_text(encoding='utf-8')
RUNNAV = (LAUNCH / 'run_navigation.launch.py').read_text(encoding='utf-8')
BASE = (LAUNCH / 'real_base.launch.py').read_text(encoding='utf-8')
SCRIPT = (ROOT / 'scripts' / 'run_navigation.sh').read_text(encoding='utf-8')
BRIDGE = (ROOT / 'ros2_ws' / 'src' / 'fwr_bringup' / 'fwr_bringup'
          / 'mqtt_bridge.py').read_text(encoding='utf-8')
PAGE = (ROOT / 'cloud' / 'web' / 'car.html').read_text(encoding='utf-8')


def test_base_launch_is_hardware_only():
    assert 'real_mapping.launch.py' in BASE
    assert "'start_slam': 'false'" in BASE
    assert "'start_base': 'true'" in BASE
    # 薄包装：节点定义只允许在 real_mapping 里有一处
    assert "executable='base_bridge'" not in BASE


def test_hardware_nodes_gated_by_start_base():
    assert "DeclareLaunchArgument('start_base'" in MAPPING
    assert 'def _both(' in MAPPING
    # 硬件节点必须带 start_base 开关（底盘/雷达/IMU/EKF/合并/守卫/静态 TF）
    assert MAPPING.count("_both('start_base'") >= 5
    assert MAPPING.count("IfCondition(LaunchConfiguration('start_base'))") >= 7
    # slam_map_save 只在建图时起（否则纯硬件层也会跑它）
    assert "executable='slam_map_save',\n             name='slam_map_save', output='screen',\n             condition=IfCondition(LaunchConfiguration('start_slam'))" in MAPPING


def test_start_base_threaded_through_nav_chain():
    for text, name in ((LOC, 'real_localization'), (NAV, 'real_navigation'),
                       (RUNNAV, 'run_navigation')):
        assert "DeclareLaunchArgument('start_base'" in text, name
    assert "'start_base': LaunchConfiguration('start_base')" in NAV
    assert "'start_base': LaunchConfiguration('start_base')" in RUNNAV
    assert "'start_base': LaunchConfiguration('start_base')" in LOC


def test_nav_script_does_not_touch_hardware_when_managed():
    # 硬件由 fwr-base 托管时，导航栈绝不能 pkill base_bridge（会误判旧栈仍在）
    assert '硬件层由 fwr-base 托管，跳过清理' in SCRIPT
    assert '"start_base:=${FWR_START_BASE:-true}"' in SCRIPT


def test_bridge_mode_action_cleans_before_start():
    assert 'MODE_UNITS' in BRIDGE and 'ALL_MODE_UNITS' in BRIDGE
    assert "def _switch_mode(" in BRIDGE
    assert "if action == 'mode':" in BRIDGE
    # 先停非目标栈，再起目标栈
    i_stop = BRIDGE.index("切到 {target}：先停 {unit}")
    i_start = BRIDGE.index("self._systemctl('start', unit, timeout=120)")
    assert i_stop < i_start, '必须先清理再启动'
    # 硬件层没跑要先拉起来（否则手动模式没底盘）
    assert 'fwr-base 未运行，先拉起硬件层' in BRIDGE
    # 状态里要报真实模式，页面才知道当前是 manual/nav/map/follow
    assert 'mode=self._current_mode' in BRIDGE


def test_page_switches_mode_through_single_action():
    assert 'function sendModeSwitch(' in PAGE
    assert "action: 'mode'" in PAGE
    # 旧的 mapping_start/stop 发送点应已改走 mode（bridge 侧仍兼容）
    assert "action: 'mapping_start'" not in PAGE
    assert "action: 'mapping_stop'" not in PAGE
    # 手动/跟随 → 导航：必须显式启导航栈（不再开机自启）
    assert "sendModeSwitch('nav');" in PAGE
    assert "sendModeSwitch('manual');" in PAGE
    assert "sendModeSwitch('follow');" in PAGE


def test_lidar_watchdog_never_spawns_its_own_node():
    """硬件常驻 fwr-base 后，看门狗只能杀不能起（否则抢串口、两个 publisher）。"""
    wd = (ROOT / 'tools' / 'lidar_watchdog.py').read_text(encoding='utf-8')
    assert "'--no-spawn'" in wd
    assert '交给 fwr-base 的 launch respawn' in wd


def test_cloud_relay_forwards_mode_field():
    """云端 /ws/control 转发 nav 时是字段白名单，漏一个字段车端就收不到。

    2026-11-24 现场：手机切「导航模式」→ 云端只转发 {'action':'mode'} →
    车端 `obj.get('mode')` 为空 → 回落成手动 → 页面等 1 min+ 一直
    「等待导航栈就绪（第 N 次检查）」，导航栈根本没起。
    """
    api = (ROOT / 'cloud' / 'api' / 'app.py').read_text(encoding='utf-8')
    i = api.index("if raw.get('type') == 'nav':")
    block = api[i:i + 1200]
    assert "if raw.get('mode'):" in block, '云端转发必须带上 mode'
    assert "body['mode'] = str(raw['mode'])" in block


def test_bridge_ignores_unknown_mode_instead_of_falling_back():
    """未知/缺失模式必须忽略，不能回落成手动（否则一切切换都变成停车）。"""
    assert "忽略未知模式" in BRIDGE
    assert "target = (target or '').strip().lower()" in BRIDGE
    assert "target = 'manual'\n" not in BRIDGE.split('def _switch_mode')[1][:400]
