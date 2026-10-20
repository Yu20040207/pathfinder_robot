import math

import pytest

from fwr_bringup.turn_controller import TurnController, TurnFinished
from fwr_bringup.base_bridge_core import BaseBridgeCore
from fwr_bringup.base_protocol import parse_state_line


def state():
    return parse_state_line('S2 100000 0 0 0 0 0 0 0 0 0 0 0 0')


def test_pivot_uses_opposite_wheels_and_arc_uses_forward_wheels():
    pivot = TurnController(5, 1, 0)
    assert pivot.update(0, 0, .01, 0, 0) == (-80, 80)
    right = TurnController(-5, 1, 0)
    assert right.update(0, 0, .01, 0, 0) == (80, -80)
    arc = TurnController(5, 2, 0)
    assert arc.update(0, 0, .01, 0, 0) == (50, 80)
    assert TurnController(-5, 2, 0).update(0, 0, .01, 0, 0) == (80, 50)


def test_five_degree_completion_and_slowdown():
    turn = TurnController(5, 1, 0)
    assert turn.update(math.radians(2.5), 0, .1, -.001, .001) == (-50, 50)
    done = turn.update(math.radians(4), 0, .2, -.002, .002)
    assert isinstance(done, TurnFinished) and done.success
    assert done.command == (0, 0)


def test_ten_degree_turn_uses_imu_progress_until_target():
    turn = TurnController(10, TurnController.PIVOT, 0)
    assert turn.update(math.radians(3), 0, .1, -.003, .003) == (-80, 80)
    assert turn.update(math.radians(7), 0, .2, -.006, .006) == (-50, 50)
    assert turn.update(math.radians(7.5), 0, .3, -.009, .009) == (-50, 50)
    done = turn.update(math.radians(9), 0, .35, -.011, .011)
    assert isinstance(done, TurnFinished) and done.success
    assert done.actual_angle_deg == pytest.approx(9)
    assert done.command == (0, 0)


def test_negative_angle_wrap_and_overshoot_stop():
    turn = TurnController(-5, 1, 0)
    done = turn.update(math.radians(355), 0, .1, .001, -.001)
    assert isinstance(done, TurnFinished) and done.success
    overshoot = TurnController(5, 1, 0).update(math.radians(11), 0, .1, 0, 0)
    assert isinstance(overshoot, TurnFinished) and not overshoot.success
    assert overshoot.command == (0, 0)


def test_stale_imu_encoder_fault_cancel_and_timeout_stop():
    turn = TurnController(5, 1, 0)
    for reason, output in [
        ('imu_stale', turn.update(0, 0, .2, 0, 0, imu_age_s=.3)),
        ('cancelled', TurnController(5, 1, 0).cancel()),
        ('fault', TurnController(5, 1, 0).update(0, 0, .1, 0, 0, fault=1)),
        ('timeout', TurnController(5, 1, 0).update(0, 0, 3.1, 0, 0)),
        ('encoder_stall', TurnController(5, 1, 0).update(0, 0, .5, 0, 0)),
    ]:
        assert isinstance(output, TurnFinished)
        assert output.reason == reason and output.command == (0, 0)


def test_initial_twitch_then_one_wheel_stops_is_stall():
    turn = TurnController(5, 1, 0)
    turn.update(0, 0, .1, -.003, .003)
    turn.update(0, 0, .2, -.003, .006)
    output = turn.update(0, 0, .5, -.003, .012)
    assert isinstance(output, TurnFinished)
    assert output.reason == 'encoder_stall'


def test_bridge_rejects_nav_velocity_while_turn_owns_it():
    core = BaseBridgeCore()
    core.connect(0)
    core.handshake('BASE_CONTROL_V2')
    core.step(state(), 0)
    assert core.begin_turn(.01)
    with pytest.raises(RuntimeError):
        core.accept_cmd(.08, 0, .01)
    core.accept_turn_cmd(-80, 80, .02)
    assert core.step(None, .03).command == b'v -80.000 80.000\n'
    core.end_turn()
    assert core.step(None, .04).command == b'x'


def test_turn_cannot_take_over_fresh_navigation_velocity():
    core = BaseBridgeCore()
    core.connect(0)
    core.handshake('BASE_CONTROL_V2')
    core.step(state(), 0)
    core.accept_cmd(.08, 0, .01)
    assert not core.begin_turn(.02)
    assert core.step(None, .03).command == b'v 80.000 80.000\n'
    assert core.begin_turn(.30) is False  # serial state is stale by then
