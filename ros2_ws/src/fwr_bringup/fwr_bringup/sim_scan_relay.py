"""Simulation-only scan relay for repeatable sensor-loss injection."""

from copy import deepcopy
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


class SimScanRelay(Node):
    def __init__(self):
        super().__init__('sim_scan_relay')
        self.enabled = True
        self.front_blocked = False
        self.rear_blocked = False
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.safety_pub = self.create_publisher(LaserScan, '/scan_safety', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_raw', self._scan, qos_profile_sensor_data)
        self.create_service(SetBool, '/sim/scan_enabled', self._set_enabled)
        self.create_service(SetBool, '/sim/front_blocked', self._set_front_blocked)
        self.create_service(SetBool, '/sim/rear_blocked', self._set_rear_blocked)

    def _scan(self, msg):
        if self.enabled:
            self.pub.publish(msg)
            safe_scan = inject_front_obstacle(msg) if self.front_blocked else msg
            if self.rear_blocked:
                safe_scan = inject_rear_obstacle(safe_scan)
            self.safety_pub.publish(safe_scan)

    def _set_enabled(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = 'scan enabled' if self.enabled else 'scan disabled'
        return response

    def _set_front_blocked(self, request, response):
        self.front_blocked = request.data
        response.success = True
        response.message = 'front blocked' if self.front_blocked else 'front clear'
        return response

    def _set_rear_blocked(self, request, response):
        self.rear_blocked = request.data
        response.success = True
        response.message = 'rear blocked' if self.rear_blocked else 'rear clear'
        return response


def inject_front_obstacle(scan):
    result = deepcopy(scan)
    # C1 origin is 0.19 m ahead of base_link: 0.40 m here appears at x=0.59 m,
    # inside front_stop (x<=0.62) but outside the 0.40 m swept chassis disk.
    distance = .40
    if not (scan.range_min <= distance <= scan.range_max):
        return result
    for i, value in enumerate(result.ranges):
        angle = result.angle_min + i * result.angle_increment
        if math.isfinite(angle) and abs(angle) <= .35:
            result.ranges[i] = min(value, distance) if math.isfinite(value) else distance
    return result


def inject_rear_obstacle(scan):
    result = deepcopy(scan)
    # Lidar is 0.19 m ahead of base_link; a 0.55 m rear ray lands at x=-0.36 m.
    distance = .55
    if not (scan.range_min <= distance <= scan.range_max):
        return result
    for i, value in enumerate(result.ranges):
        angle = result.angle_min + i * result.angle_increment
        if math.isfinite(angle) and abs(abs(angle) - math.pi) <= .35:
            result.ranges[i] = min(value, distance) if math.isfinite(value) else distance
    return result


def main():
    rclpy.init()
    node = SimScanRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
