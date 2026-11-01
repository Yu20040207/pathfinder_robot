"""Select the direction-appropriate Nav2 Collision Monitor output."""

import math
import os
import time

from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .directional_safety_core import DirectionalSafetyCore


class DirectionalSafetySelector(Node):
    def __init__(self):
        super().__init__('directional_safety_selector')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('require_manual_status', True)
        # 纯旋转放行：车头（扫描角 ±180，±30° 扇区）最近障碍超过这个值才放行旋转。
        # 0.38 m ≈ 车体半对角线 0.37 m，旋转扫不到。
        # 0.34：车体前缘 0.30 + 4 cm 余量。0.38（半对角线）偏保守，
        # 现场"明明右转有空间"却被拒；旋转是原地转，前缘不会比 0.30 更近。
        self.declare_parameter('rotate_clear_m', 0.34)
        self._last_scan = None
        self._front_min_m = float('inf')
        # 2026-11-01：默认关闭 —— reverse 监控贴墙必然触发，会否决所有旋转
        self.declare_parameter('require_reverse_monitor', False)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.core = DirectionalSafetyCore(
            require_manual_status=bool(self.get_parameter('require_manual_status').value))
        self.core.require_reverse_monitor = bool(
            self.get_parameter('require_reverse_monitor').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.monitor_clients = {
            direction: self.create_client(GetState, f'/collision_monitor_{direction}/get_state')
            for direction in ('forward', 'reverse')
        }
        self.monitor_queries = {direction: None for direction in self.monitor_clients}
        for direction in self.monitor_clients:
            self.create_subscription(
                TransitionEvent, f'/collision_monitor_{direction}/transition_event',
                lambda _, direction=direction: self.core.observe_monitor(
                    direction, False, time.monotonic()), 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_auto_safe', 10)
        self.create_subscription(Twist, '/cmd_vel_auto_unchecked', self._raw, 10)
        self.create_subscription(Twist, '/cmd_vel_auto_forward_filtered',
                                 lambda msg: self._filtered('forward', msg), 10)
        self.create_subscription(Twist, '/cmd_vel_auto_reverse_filtered',
                                 lambda msg: self._filtered('reverse', msg), 10)
        def _on_scan(msg):
            self._last_scan = msg
            self._front_min_m = self._compute_front_min(msg)
            self.core.observe_scan(time.monotonic())

        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 _on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/explore/state',
                                 lambda msg: self.core.observe_explore_state(msg.data, time.monotonic()), 10)
        self.create_subscription(Bool, '/base/manual_enabled',
                                 lambda msg: self.core.observe_manual(msg.data, time.monotonic()), 10)
        self.create_timer(.05, self._tick)
        self.create_timer(.1, self._poll_monitors)
        self.publisher.publish(Twist())

    def _poll_monitors(self):
        now = time.monotonic()
        for direction, client in self.monitor_clients.items():
            if not client.service_is_ready():
                self.core.observe_monitor(direction, False, now)
                self.monitor_queries[direction] = None
                continue
            query = self.monitor_queries[direction]
            if query is not None and not query.done():
                continue
            if query is not None:
                try:
                    active = query.result().current_state.label == 'active'
                except (AttributeError, RuntimeError):
                    active = False
                self.core.observe_monitor(direction, active, now)
            self.monitor_queries[direction] = client.call_async(GetState.Request())

    def _raw(self, msg):
        self.core.observe_raw(msg.linear.x, msg.angular.z, time.monotonic())

    def _filtered(self, direction, msg):
        self.core.observe_filtered(direction, msg.linear.x, msg.angular.z,
                                   time.monotonic())

    @staticmethod
    def _compute_front_min(scan):
        """车头方向（扫描角 ±180，±30°）最近障碍；无回波返回 inf。"""
        n = len(scan.ranges)
        if n == 0:
            return float('inf')
        vals = []
        for i, r in enumerate(scan.ranges):
            a = math.degrees(scan.angle_min + i * scan.angle_increment)
            if abs(abs(a) - 180.0) <= 30.0 and math.isfinite(r) and r > 0.05:
                vals.append(r)
        return min(vals) if vals else float('inf')

    def _tick(self):
        now = time.monotonic()
        self.core.observe_tf(self._tf_ready(), now)
        self.core.rotate_clear_ok = (
            self._front_min_m >= float(self.get_parameter('rotate_clear_m').value))
        linear_x, angular_z = self.core.select(now)
        if os.environ.get('FWR_SELECTOR_DEBUG'):
            reason = self.core.last_reason
            if reason != getattr(self, '_last_reason', None):
                self._last_reason = reason
                print(f'[selector] {reason}', flush=True)
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.publisher.publish(command)

    def _tf_ready(self) -> bool:
        try:
            odom_tf = self.tf_buffer.lookup_transform('odom', self.base_frame, Time())
            self.tf_buffer.lookup_transform(self.base_frame, 'laser_front', Time())
        except TransformException:
            return False
        stamp = odom_tf.header.stamp.sec + odom_tf.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        return stamp > 0 and -.1 <= age <= .25


def main():
    rclpy.init()
    node = DirectionalSafetySelector()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
