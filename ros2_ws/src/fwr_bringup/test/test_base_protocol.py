import math

import pytest

from fwr_bringup.base_protocol import encode_wheel_speed, parse_handshake, parse_state_line


def test_complete_state_line_preserves_counts_and_fault():
    state = parse_state_line('S2 123456 -1468 1468 2 3 -90 88 -100 100 -0.60 0.65 1 6')
    assert state.esp_us == 123456
    assert (state.left_count, state.right_count) == (-1468, 1468)
    assert (state.left_skipped, state.right_skipped) == (2, 3)
    assert (state.left_measured, state.right_measured) == (-90.0, 88.0)
    assert (state.left_setpoint, state.right_setpoint) == (-100.0, 100.0)
    assert (state.left_pwm, state.right_pwm, state.active, state.fault) == (-0.60, 0.65, True, 6)


@pytest.mark.parametrize('line', [
    'S2 1 0', 'S2 1 0 0 0 0 0 0 0 0 0 0 0',
    'S2 1 0 0 0 0 nan 0 0 0 0 0 0 0',
    'S2 1 0 0 0 0 0 0 0 0 0 0 0 99',
    'S2 1 0 0 0 0 0 0 0 0 0 0 0 0 extra',
    'S2 ' + '0 ' * 300,
])
def test_invalid_state_is_rejected(line):
    assert parse_state_line(line) is None


def test_version_handshake_accepts_only_v2():
    assert parse_handshake('BASE_CONTROL_V2') is True
    assert parse_handshake('BASE_CONTROL_V1') is False


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf, 300.01, -300.01])
def test_wheel_speed_rejects_nonfinite_and_out_of_range(value):
    with pytest.raises(ValueError):
        encode_wheel_speed(value, 0)


def test_wheel_speed_has_single_newline_terminated_record():
    assert encode_wheel_speed(-100, 100) == b'v -100.000 100.000\n'
