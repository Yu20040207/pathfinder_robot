import math
import unittest

from fwr_bringup.pose_alignment import align_map_to_odom


class PoseAlignmentTest(unittest.TestCase):
    def test_manual_model_move_changes_map_to_odom(self):
        x, y, yaw = align_map_to_odom(
            map_base=(-1.86, -0.13, 0.0),
            odom_base=(1.85, -0.19, 0.0),
        )
        self.assertAlmostEqual(x, -3.71)
        self.assertAlmostEqual(y, 0.06)
        self.assertAlmostEqual(yaw, 0.0)

    def test_relative_rotation_is_applied_to_odom_translation(self):
        x, y, yaw = align_map_to_odom(
            map_base=(2.0, 3.0, math.pi / 2),
            odom_base=(1.0, 0.0, 0.0),
        )
        self.assertAlmostEqual(x, 2.0)
        self.assertAlmostEqual(y, 2.0)
        self.assertAlmostEqual(yaw, math.pi / 2)


if __name__ == '__main__':
    unittest.main()
