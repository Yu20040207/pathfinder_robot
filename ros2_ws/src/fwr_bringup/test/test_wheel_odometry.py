"""Wheel encoder parsing and differential drive calibration checks."""

import math
import pytest

from fwr_bringup.wheel_odometry import WheelOdometry, parse_encoder_line


def test_parse_real_firmware_report():
    sample = parse_encoder_line(
        'I (873462) fwr_motor_pulse: encoder A=1451 skipped=0 '
        'B=-1464 skipped=0 (4x edges, uncalibrated)'
    )
    assert sample == (873462, 1451, 0, -1464, 0)
    assert parse_encoder_line('I (873462) fwr_motor_pulse: STOP') is None


def test_forward_and_reverse_one_revolution():
    odom = WheelOdometry(circumference_m=0.210, track_m=0.263, counts_per_rev=1468)
    assert odom.update(0, 0, 0.0, boot_ms=100) is None
    forward = odom.update(-1468, 1468, 1.0, boot_ms=1100)
    assert math.isclose(forward.x, 0.210, abs_tol=1e-9)
    assert math.isclose(forward.y, 0.0, abs_tol=1e-9)
    assert math.isclose(forward.yaw, 0.0, abs_tol=1e-9)
    assert math.isclose(forward.linear_mps, 0.210, abs_tol=1e-9)
    reverse = odom.update(0, 0, 2.0, boot_ms=2100)
    assert math.isclose(reverse.x, 0.0, abs_tol=1e-9)
    assert math.isclose(reverse.linear_mps, -0.210, abs_tol=1e-9)


def test_right_wheel_only_turns_left():
    odom = WheelOdometry(circumference_m=0.210, track_m=0.263, counts_per_rev=1468)
    odom.update(0, 0, 0.0, boot_ms=100)
    sample = odom.update(0, 1468, 1.0, boot_ms=1100)
    assert math.isclose(sample.yaw, 0.210 / 0.263, abs_tol=1e-9)
    assert sample.x > 0
    assert sample.y > 0
    assert sample.angular_rps > 0


def test_device_reboot_rebaselines_without_pose_jump():
    odom = WheelOdometry(circumference_m=0.210, track_m=0.263, counts_per_rev=1468)
    odom.update(0, 0, 0.0, boot_ms=100)
    odom.update(-1468, 1468, 1.0, boot_ms=1100)
    assert odom.update(0, 0, 2.0, boot_ms=50) is None
    sample = odom.update(-1468, 1468, 3.0, boot_ms=1050)
    assert math.isclose(sample.x, 0.420, abs_tol=1e-9)


def test_implausible_count_jump_is_rejected_and_rebaselined():
    odom = WheelOdometry(circumference_m=0.210, track_m=0.263, counts_per_rev=1468)
    odom.update(0, 0, 0.0, boot_ms=100)
    assert odom.update(-100000, 100000, 0.1, boot_ms=200) is None
    assert math.isclose(odom.x, 0.0, abs_tol=1e-9)


def test_connection_loss_preserves_pose_but_rebaselines_counts():
    odom = WheelOdometry(circumference_m=0.210, track_m=0.263, counts_per_rev=1468)
    odom.update(0, 0, 0.0, boot_ms=100)
    odom.update(-1468, 1468, 1.0, boot_ms=1100)
    odom.reset_reference()
    assert odom.update(0, 0, 10.0, boot_ms=9000) is None
    sample = odom.update(-1468, 1468, 11.0, boot_ms=10000)
    assert math.isclose(sample.x, 0.420, abs_tol=1e-9)


def test_encoder_signed_wrap_does_not_jump_pose():
    odom = WheelOdometry(circumference_m=.210, track_m=.263, counts_per_rev=1468)
    assert odom.update(-2147483640, 2147483640, 0.0, boot_ms=100) is None
    sample = odom.update(2147483640, -2147483640, 1.0, boot_ms=1100)
    assert sample.x == pytest.approx(.210 * 16 / 1468)
