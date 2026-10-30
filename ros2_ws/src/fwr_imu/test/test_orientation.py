import math
import unittest

from fwr_imu.orientation import quaternion_xyzw, relative_quaternion_xyzw


class OrientationTest(unittest.TestCase):
    def test_device_wxyz_becomes_normalized_ros_xyzw(self):
        yaw = math.radians(30)
        q = quaternion_xyzw((1.2 * math.cos(yaw / 2), 0.0, 0.0,
                             1.2 * math.sin(yaw / 2)))
        self.assertAlmostEqual(q[0], 0.0)
        self.assertAlmostEqual(q[1], 0.0)
        self.assertAlmostEqual(q[2], math.sin(yaw / 2))
        self.assertAlmostEqual(q[3], math.cos(yaw / 2))

    def test_rejects_invalid_quaternion(self):
        self.assertIsNone(quaternion_xyzw((0.0, 0.0, 0.0, 0.0)))
        self.assertIsNone(quaternion_xyzw((math.nan, 0.0, 0.0, 1.0)))

    def test_relative_heading_starts_at_zero_and_tracks_left_turn(self):
        def yaw(degrees):
            angle = math.radians(degrees) / 2
            return (0.0, 0.0, math.sin(angle), math.cos(angle))

        initial = yaw(30)
        for actual, expected in zip(relative_quaternion_xyzw(initial, initial),
                                    (0.0, 0.0, 0.0, 1.0)):
            self.assertAlmostEqual(actual, expected)
        changed = relative_quaternion_xyzw(initial, yaw(45))
        self.assertAlmostEqual(changed[2], math.sin(math.radians(15) / 2))
        self.assertAlmostEqual(changed[3], math.cos(math.radians(15) / 2))


if __name__ == '__main__':
    unittest.main()


def test_scale_yaw_xyzw_is_identity_at_unit_scale():
    import math as m
    from fwr_imu.orientation import scale_yaw_xyzw, _xyzw_from_euler
    q = _xyzw_from_euler(0.1, -0.05, m.radians(40))
    out = scale_yaw_xyzw(q, 1.0, m.radians(10))
    for a, b in zip(q, out):
        assert abs(a - b) < 1e-9


def test_scale_yaw_xyzw_corrects_under_reported_rotation():
    import math as m
    from fwr_imu.orientation import scale_yaw_xyzw, _xyzw_from_euler, _euler_from_xyzw
    ref = 0.0
    # IMU 认为转了 120°，真实是 120 * 1.0127
    q = _xyzw_from_euler(0.0, 0.0, m.radians(120))
    out = scale_yaw_xyzw(q, 1.0127, ref)
    _, _, yaw = _euler_from_xyzw(out)
    assert abs(m.degrees(yaw) - 120 * 1.0127) < 1e-6


def test_scale_yaw_xyzw_wraps_across_pi():
    import math as m
    from fwr_imu.orientation import scale_yaw_xyzw, _xyzw_from_euler, _euler_from_xyzw
    ref = m.radians(170)
    q = _xyzw_from_euler(0.0, 0.0, m.radians(-175))
    out = scale_yaw_xyzw(q, 2.0, ref)
    _, _, yaw = _euler_from_xyzw(out)
    # 相对参考的最短角差 = +15°（从 170° 到 185°），乘 2 → +30° → 绝对 200°（等价 -160°）
    assert abs(((m.degrees(yaw) - 200 + 180) % 360) - 180) < 1e-6
