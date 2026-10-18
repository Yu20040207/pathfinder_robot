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
