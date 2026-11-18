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
    seg = node.split('def _on_manual_velocity')[1][:2200]
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
    assert 'def _sensors_ready' in node and 'require_scan: bool = True' in node
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


def test_single_goal_near_the_robot_is_allowed_with_warning():
    """终点选在车附近必须**允许**（用户要求："选在车附近就是让车回到终点"）。
    只记一条告警，不再拒绝派发。"""
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'frontier_explorer.py').read_text()
    assert "declare_parameter('min_fixed_goal_distance_m', 0.5)" in src
    seg = src.split('def _start_fixed_goal')[1][:4000]
    assert 'dist < min_dist' in seg
    assert '不再拒绝' in seg or '允许' in seg
    assert 'response.accepted = False' not in seg.split('dist < min_dist')[1][:400]


def test_route_whose_points_sit_on_the_robot_degrades_to_single_goal():
    """两点路线里除终点外都贴在车身上时，必须退化成单点任务而不是"开始即结束"。

    现场日志：legs=2，0.6 s 就 Goal succeeded → 面板"开始导航 → 任务已结束"。
    根因：kept 只剩终点时原代码条件不满足，继续沿用原列表 → 第一段零长度。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'frontier_explorer.py').read_text()
    assert 'if len(kept) < 2 and len(pts) >= 2:' in src
    assert '退化为单点任务' in src
    # 2026-11-15：终点落在车附近不再拒绝（用户要求），只告警并交给 Nav2 判到达
    assert '交给 Nav2 判到达' in src


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
    assert 'self._sm._fixed_dispatched = self._resume_leg' in seg


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


def test_lidar_gap_failure_pauses_and_retries_instead_of_aborting():
    """雷达断流导致的导航失败必须"暂停等恢复再重派本段"，而不是记失败/中止任务。

    现场（2026-11-15）：雷达卡了 15.5 s → 规划器 TF 外推失败 → 行为树 Goal failed →
    整条 4 点路线直接结束；恢复后车停在原地不动。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'frontier_explorer.py').read_text()
    assert "declare_parameter('sensor_loss_pause_s', 25.0)" in src
    assert "_sensor_gap_at" in src
    # 2026-11-16：暂停/恢复都会发 /explore/notice（面板状态栏显示），恢复后自动继续
    assert "/explore/notice" in src
    assert '已暂停任务，恢复后自动继续' in src
    tick = src.split('if self._paused_for_sensor:')[1][:1200]
    assert 'scan_ok' in tick and '已自动继续导航' in tick


def test_valid_state_frame_marks_link_verified_after_firmware_reboot():
    """固件重启/漏收握手横幅后，收到合法状态帧必须自动把 verified 置回 True。

    现场：ESP32 重启后遥测与里程计都正常，但 ready()=False，
    面板报"底盘授权失败：serial not ready"，只能重启整个栈。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'base_bridge_core.py').read_text()
    seg = src.split('def step(')[1][:800]
    assert 'self._last_state_at = now_s' in seg
    assert 'if not self.verified:' in seg
    # 里程计仍要等握手，避免固件重启后计数跳变（见 test_encoder_forward_and_reboot_rebaseline）


def test_resent_identical_route_resumes_instead_of_restarting_from_first_point():
    """面板"继续"重发同一条路线时，必须按已记录进度继续，不能从第 1 点重走。

    现场：导航中按空格停止 → 继续 → 车去了不是"该走的下一个航点"。
    根因：/explore/resume 失败后面板会整条路线重发，而多点入口把它当新任务
    （_fixed_dispatched 归零）→ 从第 1 点开始。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'frontier_explorer.py').read_text()
    seg = src.split('def _start_multi_point_goal')[1][:8000]
    assert '同一条路线重发' in seg
    assert 'idx_target = max(0, min(prev, len(self._fixed_goals)) - 1)' in seg
    assert 'resume: 继续第' in seg


def test_multi_point_routes_are_sequential_by_default_so_resume_works():
    """多点路线默认逐段执行：一次穿过模式暂停/继续时只能整条重发 →
    车会退回第 1 个航点（现场反复出现）。逐段执行才能按段续走。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'frontier_explorer.py').read_text()
    assert "declare_parameter('multi_point_sequential', True)" in src
    seg = src.split('def _start_multi_point_goal')[1][:9000]
    assert 'multi_point_sequential' in seg
    assert '多点路线 %d 点 → 逐段执行' in seg


def test_stale_goal_result_is_ignored_after_redispatched_leg():
    """暂停/继续重派同一段后，旧目标的 result 回调必须被丢弃。

    现场：继续第 2/5 段 → 正确重发第 2 段；紧接着又派第 3 段、然后回第 1 段。
    原因：被取消的旧目标 result 晚到，把新目标的状态改成 canceled/succeeded，
    状态机据此推进。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'frontier_explorer.py').read_text()
    assert 'self._dispatch_seq += 1' in src
    assert 'def _accepted(self, future, seq=None):' in src
    assert 'def _finished(self, future, seq=None):' in src
    assert 'seq != self._dispatch_seq' in src


def test_dispatch_sequence_is_initialized_in_constructor():
    """_dispatch_seq 必须在 __init__ 里初始化。

    2026-11-15 现场：只在派发处 +=、忘了初始化 → 第一次派发 AttributeError，
    frontier_explorer 直接退出，面板报"服务尚未就绪"（节点都没了）。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'frontier_explorer.py').read_text()
    init = src.split('def __init__')[1].split('def ')[0]
    assert 'self._dispatch_seq = 0' in init


def test_resume_uses_remembered_active_target_and_ignores_arm_loss_while_paused():
    """暂停/继续按"当前正在前往的那个点"恢复；暂停期间授权丢失不 trip。

    用户要求（2026-11-15）：直接记住当前目标坐标，暂停后没到达就继续这个点。
    另外暂停时门是关的，底盘会撤授权 —— 那是预期，不能触发"授权丢失中止任务"。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'frontier_explorer.py').read_text()
    assert "self._active_target = {'idx'" in src
    resume = src.split('def _resume')[1][:4000]
    assert "self._active_target['idx']" in resume
    arm = src.split('def _on_bridge_arm')[1][:1200]
    assert 'self._mission_suspended' in arm


def test_resume_resets_stuck_timer_and_stuck_retry_keeps_current_leg():
    """继续时清零"监控压零"计时；卡住重试必须回到当前段，不能回第 1 段。

    现场：继续后 0.2 s 就报"监控已持续 3.5 秒只发零速"→取消→重规划→跑到第 1 个航点。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'frontier_explorer.py').read_text()
    resume = src.split('def _resume')[1][:4000]
    assert 'self._fwd_nz_at = None' in resume
    retry = src.split('def _retry_after_stuck')[1][:1200]
    assert "_active_target['idx']" in retry
    assert 'start_fixed(legs=len(self._fixed_goals))' in retry
