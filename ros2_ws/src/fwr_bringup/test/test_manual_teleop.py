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
