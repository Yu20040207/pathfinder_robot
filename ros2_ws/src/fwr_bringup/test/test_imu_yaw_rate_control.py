import pytest

from fwr_bringup.imu_yaw_rate_control import ImuYawRateController


def test_hold_left_uses_imu_error_and_stays_bounded():
    control = ImuYawRateController()
    initial = control.update(.4, 0.0, 1.0)
    stalled = control.update(.4, 0.0, 1.1)
    boosted = control.update(.4, 0.0, 1.2)
    moving = control.update(.4, .4, 1.3)
    assert .4 < initial <= stalled <= .75
    assert boosted == .75
    assert .4 <= moving < stalled


def test_hold_right_is_symmetric_and_never_reverses_output():
    control = ImuYawRateController()
    assert control.update(-.4, 0.0, 1.0) < -.4
    for n in range(1, 30):
        output = control.update(-.4, 0.0, 1.0 + n * .05)
        assert -.75 <= output < 0


def test_release_and_direction_flip_clear_integral():
    control = ImuYawRateController()
    for n in range(20):
        control.update(.4, 0.0, n * .05)
    assert control.update(0.0, .1, 1.1) == 0.0
    fresh = control.update(-.4, 0.0, 1.2)
    assert fresh == pytest.approx(-ImuYawRateController().update(.4, 0.0, 1.2))


def test_saturated_turn_does_not_keep_accelerating_after_caster_breaks_free():
    control = ImuYawRateController()
    for n in range(40):
        output = control.update(.65, 0.0, n * .05)
        assert output <= .75
    recovered = control.update(.65, .65, 2.05)
    assert recovered <= .68
