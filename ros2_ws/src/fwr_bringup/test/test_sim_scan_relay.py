from sensor_msgs.msg import LaserScan
import math
import pytest
from fwr_bringup.sim_scan_relay import inject_front_obstacle, inject_rear_obstacle


def test_injected_obstacle_changes_front_arc_only():
    scan = LaserScan()
    scan.angle_min = -1.
    scan.angle_increment = .25
    scan.range_min = .05
    scan.range_max = 8.
    scan.ranges = [2.] * 9
    changed = inject_front_obstacle(scan)
    assert list(scan.ranges) == [2.] * 9
    assert changed.ranges[4] == pytest.approx(.40)
    assert changed.ranges[0] == 2.


def test_injected_rear_obstacle_changes_rear_arc_only():
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_increment = math.pi / 4
    scan.range_min = .05
    scan.range_max = 8.
    scan.ranges = [2.] * 9
    changed = inject_rear_obstacle(scan)
    assert list(scan.ranges) == [2.] * 9
    assert changed.ranges[0] == pytest.approx(.55)
    assert changed.ranges[8] == pytest.approx(.55)
    assert changed.ranges[4] == 2.
