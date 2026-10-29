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
