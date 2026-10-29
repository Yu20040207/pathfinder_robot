"""Keep permanent rear self-occlusion out of real mapping scans."""

import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from fwr_bringup.scan_mask_core import mask_rear_ranges


def filter_scan_message(message: LaserScan, rear_half_angle: float) -> LaserScan:
    """Return a masked scan or raise ValueError for unusable scan geometry/data."""
    count = len(message.ranges)
    if (count < 2 or not math.isfinite(message.angle_max) or
            not math.isfinite(message.range_min) or
            not math.isfinite(message.range_max) or
            message.range_min < 0 or message.range_max <= message.range_min):
        raise ValueError('invalid LaserScan metadata')
    expected_max = message.angle_min + (count - 1) * message.angle_increment
    if (not math.isfinite(expected_max) or
            abs(expected_max - message.angle_max) > max(abs(message.angle_increment) * 2, 1e-3)):
        raise ValueError('LaserScan angle_max does not match ranges')
    masked = mask_rear_ranges(message.ranges, message.angle_min,
                              message.angle_increment, rear_half_angle)
    totals = [0, 0, 0]
    usable = [0, 0, 0]
    for index, value in enumerate(masked):
        raw_angle = message.angle_min + index * message.angle_increment
        rear_angle = math.atan2(math.sin(raw_angle), math.cos(raw_angle))
        if abs(rear_angle) <= rear_half_angle + 1e-6:
            continue
        front_angle = math.atan2(math.sin(raw_angle + math.pi),
                                 math.cos(raw_angle + math.pi))
        sector = min(2, max(0, int((front_angle + math.pi / 2) / (math.pi / 3))))
        totals[sector] += 1
        if value == math.inf or (math.isfinite(value) and
                                 message.range_min <= value <= message.range_max):
            usable[sector] += 1
    if any(total == 0 or good / total < 0.5 for total, good in zip(totals, usable)):
        raise ValueError('front or side scan coverage below 50%')
    result = copy.deepcopy(message)
    result.ranges = masked
    return result


class ScanMappingFilter(Node):
    def __init__(self):
        super().__init__('scan_mapping_filter')
        self.declare_parameter('rear_half_angle_rad', math.pi / 2)
        self._half_angle = float(self.get_parameter('rear_half_angle_rad').value)
        if not 0 < self._half_angle < math.pi:
            raise ValueError('rear_half_angle_rad must be between 0 and pi')
        self._publisher = self.create_publisher(LaserScan, '/scan_mapping', qos_profile_sensor_data)
        self._health = self.create_publisher(Bool, '/scan_mapping/healthy', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, message: LaserScan):
        try:
            filtered = filter_scan_message(message, self._half_angle)
        except ValueError as error:
            self.get_logger().warn(f'Mapping scan rejected: {error}', throttle_duration_sec=2.0)
            self._health.publish(Bool(data=False))
            return
        self._publisher.publish(filtered)
        self._health.publish(Bool(data=True))


def main():
    rclpy.init()
    node = ScanMappingFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
