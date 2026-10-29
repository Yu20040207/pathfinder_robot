import pytest

from fwr_bringup.base_bridge_core import BaseBridgeCore
from fwr_bringup.base_protocol import parse_state_line


def state(us=100000, left=0, right=0, fault=0):
    return parse_state_line(f'S2 {us} {left} {right} 0 0 0 0 0 0 0 0 0 {fault}')


def ready_core():
    core = BaseBridgeCore()
    core.connect(0.0)
    core.handshake('BASE_CONTROL_V2')
    core.step(state(), 0.0)
    return core


def test_forward_reverse_and_positive_ccw_mapping():
    core = ready_core()
    assert core.accept_cmd(.08, 0, .01) == (80, 80)
    assert core.step(None, .02).command == b'v 80.000 80.000\n'
    assert core.accept_cmd(-.08, 0, .03) == (-80, -80)
    left, right = core.accept_cmd(0, .4, .04)
    assert left == pytest.approx(-53.6)
    assert right == pytest.approx(53.6)


def test_invalid_or_excessive_command_is_rejected():
    core = ready_core()
    with pytest.raises(ValueError):
        core.accept_cmd(.31, 0, .01)
    with pytest.raises(ValueError):
        core.accept_cmd(float('nan'), 0, .01)
    assert core.step(None, .02).command == b'x'


def test_invalid_command_cancels_previous_motion():
    core = ready_core()
    core.accept_cmd(.08, 0, .01)
    with pytest.raises(ValueError):
        core.accept_cmd(float('nan'), 0, .02)
    assert core.step(None, .03).command == b'x'


def test_startup_and_stale_state_cannot_move():
    core = BaseBridgeCore()
    assert core.step(None, 0).command == b''
    core.connect(0)
    assert core.step(None, .01).command == b'x'
    assert not core.handshake('BASE_CONTROL_V1')
    assert core.step(state(), .02).command == b'x'
    assert core.handshake('BASE_CONTROL_V2')
    core.step(state(), .03)
    core.accept_cmd(.08, 0, .04)
    assert core.step(None, .30).command == b'x'


def test_stale_command_and_fault_zero_output_without_auto_clear():
    core = ready_core()
    core.accept_cmd(.08, 0, .01)
    assert core.step(None, .25).command == b'x'
    core.accept_cmd(.08, 0, .26)
    assert core.step(state(us=200000, fault=1), .27).command == b'x'
    assert core.fault == 1
    assert core.step(state(us=300000, fault=0), .28).command == b'x'


def test_fault_clear_requires_explicit_request_and_fresh_zero_state():
    core = ready_core()
    core.step(state(us=200000, fault=1), .1)
    assert core.fault == 1
    assert core.request_fault_clear(.11)
    assert core.step(state(us=300000, fault=1), .12).command == b'x'
    assert core.fault == 1
    assert core.step(state(us=400000, fault=0), .13).command == b'x'
    assert core.fault == 0
    assert core.ready(.13)


def test_fault_clear_rejects_motion_and_reconnect_cancels_request():
    core = ready_core()
    assert not core.request_fault_clear(.01)
    core.accept_cmd(.08, 0, .02)
    core.step(state(us=200000, fault=1), .03)
    assert not core.request_fault_clear(.04)
    core.halt()
    assert core.request_fault_clear(.04)
    core.disconnect()
    core.connect(.05)
    core.handshake('BASE_CONTROL_V2')
    core.step(state(us=300000, fault=0), .06)
    assert core.fault == 1


def test_fault_clear_gate_requires_all_motion_paths_off():
    from fwr_bringup.base_bridge_core import fault_clear_allowed
    assert fault_clear_allowed(False, False, False, False, 'stopped', True)
    assert not fault_clear_allowed(True, False, False, False, 'stopped', True)
    assert not fault_clear_allowed(False, True, False, False, 'stopped', True)
    assert not fault_clear_allowed(False, False, True, False, 'stopped', True)
    assert not fault_clear_allowed(False, False, False, True, 'stopped', True)
    assert not fault_clear_allowed(False, False, False, False, 'navigating', True)
    assert not fault_clear_allowed(False, False, False, False, 'stopped', False)


def test_disconnect_reconnect_requires_fresh_command():
    core = ready_core()
    core.accept_cmd(.08, 0, .01)
    core.disconnect()
    assert core.step(None, .02).command == b''
    core.connect(.03)
    core.handshake('BASE_CONTROL_V2')
    assert core.step(state(us=10000), .04).command == b'x'
    core.accept_cmd(.08, 0, .05)
    assert core.step(None, .06).command == b'v 80.000 80.000\n'


def test_invalid_record_does_not_update_odom():
    core = ready_core()
    assert core.step(parse_state_line('S2 malformed'), .1).odom is None


def test_encoder_forward_and_reboot_rebaseline():
    core = ready_core()
    out = core.step(state(us=1100000, left=-1468, right=1468), 1.0)
    # 一圈计数 = 一个有效轮周长（2026-10-28 实测标定为 0.2012 m）
    assert out.odom.x == pytest.approx(.2012)
    assert out.wheel_positions_rad == pytest.approx((2 * 3.141592653589793,) * 2)
    assert core.step(state(us=1000), 1.1).odom is None
    assert core.step(state(us=11000, left=-10, right=10), 1.11).odom is None
    core.handshake('BASE_CONTROL_V2')
    assert core.step(state(us=11000, left=-10, right=10), 1.12).odom is None
    assert core.step(state(us=21000, left=-20, right=20), 1.13).odom.x == pytest.approx(.2012 + .2012 * 10 / 1468)


def test_implausible_encoder_jump_does_not_move_joint_positions():
    core = ready_core()
    out = core.step(state(us=200000, left=-100000, right=100000), .1)
    assert out.odom is None
    assert out.wheel_positions_rad == (0.0, 0.0)


def test_reboot_while_command_active_requires_new_handshake_and_command():
    core = ready_core()
    core.accept_cmd(.08, 0, .01)
    assert core.step(state(us=110000), .02).command.startswith(b'v ')
    assert core.step(state(us=1000), .03).command == b'x'
    assert not core.verified
    assert core.step(state(us=2000), .04).command == b'x'
    core.handshake('BASE_CONTROL_V2')
    core.step(state(us=3000), .05)
    assert core.step(None, .06).command == b'x'
    core.accept_cmd(.08, 0, .07)
    assert core.step(None, .08).command.startswith(b'v ')


def test_forward_only_envelope_rejects_reverse_and_admits_bounded_pivot():
    from fwr_bringup.base_bridge_core import forward_only_velocity
    assert forward_only_velocity(.06, .2, .263)
    # 原地旋转：两轮反向，单轮 0.3*0.268/2 ≈ 0.040 m/s，仍在固件上限内
    assert forward_only_velocity(0, .3, .263)
    # 倒车仍然禁止
    assert not forward_only_velocity(-.02, 0, .263)
    # 超过单轮 0.3 m/s 的急转仍被拒绝
    assert not forward_only_velocity(0, 2.4, .263)
    assert forward_only_velocity(0, 0, .263)


def test_smoother_residual_rotation_is_admitted_as_bounded_pivot():
    from fwr_bringup.base_bridge_core import forward_only_velocity
    track = .263
    # Nav2 smooths linear and angular components independently; linear can
    # reach zero first, leaving a residual angular command. That residual is a
    # bounded pivot now, not a reason to stop the base mid-corner.
    trajectory = [(.06, .25), (.04, .20), (.02, .20), (0.0, .10)]
    assert all(forward_only_velocity(v, w, track) for v, w in trajectory)


def test_halt_invalidates_command_before_reenable():
    core = ready_core()
    core.accept_cmd(.06, 0, .01)
    assert core.step(None, .02).command.startswith(b'v ')
    core.halt()
    assert core.step(None, .03).command == b'x'
    assert core.step(None, .04).command == b'x'


def test_safe_command_gap_halts_before_disarming_idle_navigation():
    from fwr_bringup.base_bridge_core import safe_command_gap_action
    assert safe_command_gap_action(.10, was_zero=False) == 'ok'
    assert safe_command_gap_action(.18, was_zero=False) == 'disarm'
    assert safe_command_gap_action(.18, was_zero=True) == 'halt'
    assert safe_command_gap_action(.90, was_zero=True) == 'halt'
    assert safe_command_gap_action(1.01, was_zero=True) == 'disarm'


def test_wheel_limit_scaling_keeps_the_command_moving():
    from fwr_bringup.base_bridge_core import scale_velocity_to_wheel_limit
    track = .263
    # 原地旋转：两轮 92 mm/s，未超限。
    assert scale_velocity_to_wheel_limit(0.0, .7, track) == (0.0, .7)
    # 超速直行仍可运行，但单轮不超过固件上限。
    v, w = scale_velocity_to_wheel_limit(.2, 0, track)
    assert .199 < v <= .2 and w == 0
    # 组合超限时保留差速比例，避免整帧停机。
    v, w = scale_velocity_to_wheel_limit(.3, .3, track)
    assert 0 < v < .3 and 0 < w < .3
    assert abs(v / w - 1.0) < 1e-9
    assert max(abs(v - w * track / 2), abs(v + w * track / 2)) <= .300000001


def test_wheel_limit_scaling_preserves_tight_turn_curvature():
    from fwr_bringup.base_bridge_core import scale_velocity_to_wheel_limit

    track = .263
    v, w = scale_velocity_to_wheel_limit(.15, 1.2, track)
    # The firmware can drive at most 0.3 m/s per wheel. A sharp arc must
    # slow both components together; keeping the linear speed and clipping
    # yaw widens the requested turn until it no longer fits the path.
    assert 0 < v < .15
    assert 0 < w < 1.2
    assert abs(v / w - .15 / 1.2) < 1e-9
    assert max(abs(v - w * track / 2), abs(v + w * track / 2)) <= .300000001
