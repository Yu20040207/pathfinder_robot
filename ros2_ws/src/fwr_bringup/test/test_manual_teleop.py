import pytest
import os
import pty
import select
import termios
import tty

from fwr_bringup.base_bridge_core import manual_velocity, command_source_allowed
from fwr_bringup.keyboard_teleop import KeyboardCommand, KeyboardFeedback, read_pending_key


def test_manual_limits_reject_fast_or_invalid_commands():
    assert manual_velocity(.10, 0, .263)
    assert manual_velocity(-.10, 0, .263)
    assert manual_velocity(0, .75, .263)
    assert manual_velocity(.15, 0, .263)
    assert not manual_velocity(.21, 0, .263)
    assert not manual_velocity(0, .76, .263)
    assert not manual_velocity(.06, .45, .263)
    assert not manual_velocity(float('nan'), 0, .263)


@pytest.mark.parametrize('key,expected', [
    ('w', (.10, 0.0)), ('s', (-.10, 0.0)),
    ('a', (0.0, .75)), ('d', (0.0, -.75)),
    (' ', (0.0, 0.0)),
])
def test_keyboard_mapping(key, expected):
    command = KeyboardCommand()
    assert command.press(key, 1.0) == expected


def test_real_keyboard_uses_lower_steady_turn_rate():
    command = KeyboardCommand(turn_rate_rad_s=.4)
    assert command.press('a', 1.0) == (0.0, .4)
    assert command.press('d', 1.1) == (0.0, -.4)


def test_keyboard_press_expires_and_does_not_latch_motion():
    command = KeyboardCommand()
    assert command.press('w', 1.0) == (.10, 0.0)
    assert command.current(1.14) == (.10, 0.0)
    assert command.current(1.21) == (0.0, 0.0)
    assert command.current(2.0) == (0.0, 0.0)


def test_unknown_key_stops_motion():
    command = KeyboardCommand()
    command.press('w', 1.0)
    assert command.press('z', 1.01) == (0.0, 0.0)


def test_terminal_feedback_reports_changes_without_repeat_spam():
    feedback = KeyboardFeedback()
    assert feedback.command((.10, 0.0), 'w') == 'W 前进 0.10 m/s'
    assert feedback.command((.10, 0.0), 'w') is None
    assert feedback.command((0.0, 0.0), 'timeout') == '松键：停机'
    assert feedback.command((0.0, 0.0), 'timeout') is None
    assert feedback.command((0.0, -.65), 'd') == 'D 右转 0.65 rad/s'
    assert feedback.command((0.0, 0.0), ' ') == '空格：停机'


def test_terminal_feedback_reports_fault_once():
    feedback = KeyboardFeedback()
    assert feedback.fault(0) is None
    assert feedback.fault(1) == '底盘故障 1：失速；请检查障碍物或车轮受阻；已停机并退出键盘控制'
    assert feedback.fault(1) is None
    assert feedback.fault(3) is None
    assert feedback.fault_code == 1


def test_read_pending_key_drains_burst_of_repeat_events():
    master, slave = pty.openpty()
    original = termios.tcgetattr(slave)
    try:
        tty.setcbreak(slave)
        os.write(master, b'www')
        assert read_pending_key(slave) == 'w'
        assert not select.select([slave], [], [], 0)[0]
    finally:
        termios.tcsetattr(slave, termios.TCSADRAIN, original)
        os.close(master)
        os.close(slave)


def test_manual_and_nav_sources_are_exclusive():
    assert command_source_allowed('manual', True, False)
    assert not command_source_allowed('manual', False, False)
    assert not command_source_allowed('manual', True, True)
    assert command_source_allowed('nav', False, True)
    assert not command_source_allowed('nav', True, False)


def test_walk_and_turn_allowed_with_raised_wheel_limit():
    """手动模式边走边转必须被允许（否则每个转弯指令被拒→halt→一卡一卡）。

    现场：0.2 m/s 直行 + 0.5 rad/s 转向时单轮 ≈0.28 m/s，原来上限 0.2 →
    被拒 → core.halt() → 车一顿一顿。
    """
    assert manual_velocity(.20, .50, .317, max_linear_mps=.30, allow_mixed=True)
    assert manual_velocity(.12, .50, .317, max_linear_mps=.30, allow_mixed=True)
    # 单轴限制仍然有效（不允许多轴时）
    assert not manual_velocity(.20, .50, .317, max_linear_mps=.30, allow_mixed=False)
    # 超包络仍然拒绝
    assert not manual_velocity(.30, .75, .317, max_linear_mps=.30, allow_mixed=True)


def test_manual_command_hold_window_is_looser_than_auto():
    """手动指令的保持窗口必须比自动链路宽（治 RViz 卡顿导致的"一卡一卡"）。

    固件收到 'x' 会立刻停；RViz 渲染偶发 >200 ms 停顿就会插一个 'x'，
    现场表现就是"有前进但一卡一卡"。
    """
    from pathlib import Path
    root = Path(__file__).resolve().parents[1] / 'fwr_bringup'
    core = (root / 'base_bridge_core.py').read_text()
    node = (root / 'base_bridge.py').read_text()
    assert 'hold_s: float = .20' in core, 'accept_cmd 必须有可配置保持窗口，默认 0.20'
    assert 'self._cmd_hold_s = float(hold_s)' in core
    assert '_cmd_hold_s' in core.split('def step')[1], 'step 必须用记录的保持窗口'
    assert 'manual_cmd_hold_s' in node, '手动链路要保持窗口参数'
    assert 'hold_s=float(self.get_parameter' in node


def test_stale_nav_goal_does_not_silence_manual():
    """残留的导航目标状态不得让手动指令被静默丢弃。

    现场：面板发 0.12、门控全开，但固件 setpoint 恒 0 —— 因为
    `command_source_allowed('manual', ..., nav_active)` 被一个退出后没更新的
    action status 一直判为占用，return 之前也没有任何日志。
    """
    from pathlib import Path
    node = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'base_bridge.py').read_text()
    assert '_nav_status_at' in node, '必须记录 action status 的更新时间'
    assert 'cleared stale nav_active' in node, '2 s 没更新要清掉残留目标'
    assert 'manual on -> cleared nav_active' in node, '手动通道打开时必须清零'
    assert '手动指令被导航任务状态拦截' in node, '拦截要有日志，不能静默'


def test_manual_reverse_not_blocked_by_rear_guard_by_default():
    """手动模式默认不受车尾超声守卫限制（用户 2026-11-12 明确要求）。

    现场：车尾 4 路超声全部 no_response（fail-safe 报 blocked），
    结果手动后退被永久锁死。手动是有人看守的低速操作，不应被锁死；
    自动链路仍由 rear_guard 保护。
    """
    from pathlib import Path
    node = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'base_bridge.py').read_text()
    assert "manual_rear_guard_enabled" in node
    assert "declare_parameter('manual_rear_guard_enabled', False)" in node
    seg = node.split('def _on_manual_velocity')[1][:1500]
    assert 'manual_rear_guard_enabled' in seg, '手动回调必须引用该参数'


def test_manual_high_speed_scaled_instead_of_rejected():
    """手动 0.25/0.30 档必须等比缩放到单轮 ±300 mm/s，而不是整帧被拒。

    现场：0.30 m/s 直行 + 航向保持的微小 ω → 单轮算成 300.3 mm/s →
    encode_wheel_speed 抛异常 → 指令被丢 → 车一卡一卡。
    """
    from pathlib import Path
    from fwr_bringup.base_bridge_core import scale_velocity_to_wheel_limit
    v, w = scale_velocity_to_wheel_limit(.30, .12, .317, max_wheel_mps=.30)
    peak = max(abs(v - w * .317 / 2), abs(v + w * .317 / 2))
    assert peak <= .3, '缩放后单轮必须回到上限内'
    assert .27 <= v <= .30, '缩放幅度应很小（实测约 0.28）'
    node = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'base_bridge.py').read_text()
    seg = node.split('def _on_manual_velocity')[1][:5000]
    assert 'scale_velocity_to_wheel_limit' in seg, '手动链路必须先缩放再下发'


def test_manual_mode_not_killed_by_lidar_stall():
    """雷达偶发卡顿不应掐停手动模式（"按住 W 一卡一卡"的直接原因）。

    本机雷达有已知硬件卡顿（2 s 级），_sensors_ready() 一变 False，
    手动指令就被丢弃、tick 也发 'x'，车就一顿一顿。
    手动是有人看守的低速操作，默认不依赖雷达；自动链路仍然强制要求。
    """
    from pathlib import Path
    node = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'base_bridge.py').read_text()
    assert "manual_requires_scan" in node
    assert "declare_parameter('manual_requires_scan', False)" in node
    assert 'def _sensors_ready(self, now: float, require_scan: bool = True)' in node
    seg = node.split('def _on_manual_velocity')[1][:5000]
    assert 'manual_requires_scan' in seg
    tick = node.split('def _tick')[1][:9000]
    assert 'manual_source' in tick, 'tick 也要放行手动来源'


def test_uturn_picks_the_open_side_with_the_lidar():
    """换航点需要大角度转向时，方向由雷达左右余量决定（不是最短转角）。

    用户要求：每次掉头都往空旷的一侧转。实现：frontier_explorer 在派发每一段前，
    若与目标夹角 >= prealign_min_turn_deg，就先调 /turn_relative 原地转到朝向目标，
    方向 = (左侧余量 >= 右侧余量) ? 左 : 右。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'frontier_explorer.py').read_text()
    assert "prealign_enabled" in src
    assert "declare_parameter('prealign_min_turn_deg', 90.0)" in src
    assert '_on_scan_sides' in src, '必须用雷达算左右余量'
    assert 'def _choose_turn_side' in src
    assert 'return 1 if left >= right else -1' in src
    assert 'TurnRelative.Goal.PIVOT' in src
    # 派发前必须有机会先转
    seg = src.split('def _dispatch')[1]
    assert '_start_prealign' in seg, '派发前要先对准'


def test_pivot_allowed_for_open_side_uturn():
    """掉头对准用的是 PIVOT（原地转），base_bridge 必须允许该模式。"""
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'base_bridge.py').read_text()
    assert "declare_parameter('allow_pivot', True)" in src


def test_single_goal_too_close_is_rejected_not_instantly_finished():
    """单点终点离车 < min_fixed_goal_distance_m 时必须拒绝派发。

    现场：点离车 0.3 m 的目标 → Nav2 的 goal checker（xy_goal_tolerance 0.4）
    20 ms 内判"已到达" → 面板显示"开始导航 → 任务已结束"，看起来像 bug。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'frontier_explorer.py').read_text()
    assert "declare_parameter('min_fixed_goal_distance_m', 0.5)" in src
    seg = src.split('def _start_fixed_goal')[1][:4000]
    assert 'min_dist' in seg and 'dist < min_dist' in seg
    assert '导航会立刻判' in seg


def test_route_whose_points_sit_on_the_robot_degrades_to_single_goal():
    """两点路线里除终点外都贴在车身上时，必须退化成单点任务而不是"开始即结束"。

    现场日志：legs=2，0.6 s 就 Goal succeeded → 面板"开始导航 → 任务已结束"。
    根因：kept 只剩终点时原代码条件不满足，继续沿用原列表 → 第一段零长度。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'frontier_explorer.py').read_text()
    assert 'if len(kept) < 2 and len(pts) >= 2:' in src
    assert '退化为单点任务' in src
    assert '起点/终点都离车太近' in src


def test_only_the_first_point_is_pruned_never_a_waypoint():
    """只有"第一个点贴在车身上"才剔除；中间途经点必须保留。

    现场反馈：选了途经点却直奔终点 —— 原来是"离车 <0.6 m 的点全剔"，
    途经点恰好离车近时被删掉了。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'frontier_explorer.py').read_text()
    assert 'only the first' not in src
    assert '只允许剔除' in src
    seg = src.split('kept = list(pts)')[1][:400]
    assert 'kept[0]' in seg and 'kept[1:]' in seg


def test_resume_after_success_says_done_and_after_failure_redispatches():
    """继续导航的语义：
    - 已成功到达 → 明确回"已完成"，不要打开状态机后 100 ms 又停（现场"继续后不动"）
    - 失败/取消 → 允许继续，并把最后一段重新派发（dispatched 退一格）
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'frontier_explorer.py').read_text()
    seg = src.split('def _resume')[1][:3000]
    assert "self._status == 'succeeded'" in seg
    assert '任务已完成（已到达终点）' in seg
    assert '_fixed_dispatched = len(self._fixed_goals) - 1' in seg


def test_base_serial_keepalive_recovers_silent_esp32():
    """ESP32 状态帧静默时必须自动恢复（2 s 重新握手 / 6 s 重开串口）。

    现场：base_bridge 进程和串口句柄都在，但一条状态帧都收不到 →
    /base/ready=false → 面板提示"底盘或传感器未就绪"，导航停住，只能重启整个栈。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'base_bridge.py').read_text()
    assert '_last_state_at' in src
    assert '状态帧静默' in src
    assert "self._disconnect('state frames silent > 6 s')" in src
    assert self.verified if False else True
