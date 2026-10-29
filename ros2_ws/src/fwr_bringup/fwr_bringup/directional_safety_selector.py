"""Select the direction-appropriate Nav2 Collision Monitor output."""

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
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.core = DirectionalSafetyCore(
            require_manual_status=bool(self.get_parameter('require_manual_status').value))
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
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 lambda _: self.core.observe_scan(time.monotonic()),
                                 qos_profile_sensor_data)
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

    def _tick(self):
        now = time.monotonic()
        self.core.observe_tf(self._tf_ready(), now)
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
