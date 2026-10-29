import math
from pathlib import Path
import sys

import pytest
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fwr_bringup.scan_mapping_filter import filter_scan_message


def scan(ranges):
    msg = LaserScan()
    msg.header.frame_id = 'laser_front'
    msg.header.stamp.sec = 42
    msg.angle_min = -math.pi
    msg.angle_increment = math.pi / 4
    msg.angle_max = math.pi
    msg.range_min = 0.12
    msg.range_max = 12.0
    msg.ranges = ranges
    msg.intensities = [7.0]
    return msg


def test_filter_keeps_metadata_and_does_not_mutate_original():
    original = scan([1.0] * 9)
    filtered = filter_scan_message(original, math.pi / 2)
    assert filtered is not original
    assert filtered.header == original.header
    assert filtered.angle_min == original.angle_min
    assert filtered.angle_max == original.angle_max
    assert filtered.angle_increment == original.angle_increment
    assert list(filtered.intensities) == [7.0]
    assert list(original.ranges) == [1.0] * 9
    assert [math.isnan(v) for v in filtered.ranges] == [False, False, True, True, True, True, True, False, False]


def test_inf_forward_is_valid_no_return():
    original = scan([math.inf] * 9)
    assert filter_scan_message(original, math.pi / 2) is not None


@pytest.mark.parametrize('value', [math.nan, 0.0])
def test_no_usable_front_samples_rejected(value):
    original = scan([value] * 9)
    with pytest.raises(ValueError):
        filter_scan_message(original, math.pi / 2)


def test_malformed_geometry_rejected():
    original = scan([1.0] * 9)
    original.angle_max = 0.0
    with pytest.raises(ValueError):
        filter_scan_message(original, math.pi / 2)


def test_single_remaining_front_return_is_not_healthy():
    original = scan([math.nan] * 9)
    original.ranges[0] = 1.0
    with pytest.raises(ValueError):
        filter_scan_message(original, math.pi / 2)
