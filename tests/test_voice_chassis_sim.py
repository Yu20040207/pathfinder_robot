import math

from fwr_navigation.simulation import ChassisState, DemoVelocityLatch


def test_voice_forward_and_turn_change_rviz_pose_and_wheels():
    chassis = ChassisState()
    chassis.step(0.12, 0.0, 1.0)
    assert math.isclose(chassis.pose.x, 0.12, abs_tol=1e-6)
    assert math.isclose(chassis.left_wheel_rad, 2.4, abs_tol=1e-6)
    assert math.isclose(chassis.right_wheel_rad, 2.4, abs_tol=1e-6)
    chassis.step(0.0, 0.35, 1.0)
    assert math.isclose(chassis.pose.yaw, 0.35, abs_tol=1e-6)
    assert chassis.left_wheel_rad < 2.4 < chassis.right_wheel_rad


def test_demo_velocity_expires_when_commands_stop():
    latch = DemoVelocityLatch(timeout_s=0.3)
    latch.accept(0.12, 0.35, now=10)
    assert latch.current(now=10.2) == (0.12, 0.35)
    assert latch.current(now=10.3) == (0.0, 0.0)
