"""Activate the Nav2 lifecycle group once map, scan and map->base TF are ready."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav2_msgs.srv import ManageLifecycleNodes
import tf2_ros

from .auto_activate_core import ready_to_activate


class AutoActivateNav(Node):
    def __init__(self):
        super().__init__('auto_activate_nav')
        self.declare_parameter('service', '/lifecycle_manager_navigation/manage_nodes')
        self.declare_parameter('max_attempts', 6)
        self.declare_parameter('retry_delay_s', 8.0)
        self._map_at = None
        self._scan_at = None
        self._done = False
        self._attempts = 0
        self._last_try = 0.0
        self._future = None
        self._pending = None
        self._success = False
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, self)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(LaserScan, '/scan', self._on_scan, scan_qos)
        self._client = self.create_client(
            ManageLifecycleNodes, str(self.get_parameter('service').value))
        self.create_timer(1.0, self._tick)

    def _on_map(self, _msg):
        self._map_at = time.monotonic()

    def _on_scan(self, _msg):
        self._scan_at = time.monotonic()

    def _tick(self):
        if self._done:
            return
        now = time.monotonic()
        if self._future is not None:
            if not self._future.done():
                if now - self._last_try > 120.0:
                    self.get_logger().warn('lifecycle call timed out; retrying')
                    self._future = None
                    self._pending = None
                return
            self._future = None
            done_command = self._pending
            self._pending = None
            if self._success:
                if done_command == ManageLifecycleNodes.Request.RESET:
                    # RESET 完成，立刻重新 STARTUP（不计入失败次数）。
                    self.get_logger().info('lifecycle reset done; retrying startup')
                    self._send(ManageLifecycleNodes.Request.STARTUP, now)
                    return
                self.get_logger().info('navigation lifecycle activated automatically')
                self._done = True
                return
            if done_command == ManageLifecycleNodes.Request.STARTUP:
                self.get_logger().warn(
                    f'startup attempt {self._attempts} failed; resetting before retry')
                if self._attempts >= int(self.get_parameter('max_attempts').value):
                    self.get_logger().error('auto activation gave up; activate manually')
                    self._done = True
                    return
                # bt_navigator 偶发卡住时，RESET 能把整组拉回 unconfigured 再启动。
                self._send(ManageLifecycleNodes.Request.RESET, now)
            return
        if now - self._last_try < float(self.get_parameter('retry_delay_s').value):
            return
        tf_ok = self._buffer.can_transform('map', 'base_footprint', rclpy.time.Time())
        scan_age = None if self._scan_at is None else now - self._scan_at
        if not ready_to_activate(self._map_at is not None, scan_age, tf_ok):
            return
        if not self._client.service_is_ready():
            return
        self._last_try = now
        self._send(ManageLifecycleNodes.Request.STARTUP, now)

    def _send(self, command, now):
        request = ManageLifecycleNodes.Request()
        request.command = command
        if command == ManageLifecycleNodes.Request.STARTUP:
            self._attempts += 1
            self.get_logger().info(
                f'starting the navigation lifecycle (attempt {self._attempts})')
        self._success = False
        self._pending = command
        self._future = self._client.call_async(request)
        self._future.add_done_callback(self._on_response)

    def _on_response(self, future):
        try:
            self._success = bool(future.result().success)
        except Exception as exc:  # noqa: BLE001 - log and let the retry loop handle it
            self.get_logger().warn(f'activation call failed: {exc}')
            self._success = False


def main():
    rclpy.init()
    node = AutoActivateNav()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
