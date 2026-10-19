import pytest

from fwr_bringup.base_bridge_core import manual_velocity, command_source_allowed
from fwr_bringup.keyboard_teleop import KeyboardCommand


def test_manual_limits_reject_fast_or_invalid_commands():
    assert manual_velocity(.10, 0, .263)
    assert manual_velocity(-.10, 0, .263)
    assert manual_velocity(0, .75, .263)
    assert not manual_velocity(.11, 0, .263)
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


def test_manual_and_nav_sources_are_exclusive():
    assert command_source_allowed('manual', True, False)
    assert not command_source_allowed('manual', False, False)
    assert not command_source_allowed('manual', True, True)
    assert command_source_allowed('nav', False, True)
    assert not command_source_allowed('nav', True, False)
