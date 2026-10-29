"""Seed AMCL with a global scan match right after the navigation stack starts."""
import math
import os
import time

import numpy as np
import yaml
import tf2_ros
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav2_msgs.srv import SetInitialPose

from .auto_seed_core import base_pose_from_laser, find_best_pose, refine_pose


class AutoInitialPose(Node):
    def __init__(self):
        super().__init__('auto_initial_pose')
        self.declare_parameter('scan_topic', '/scan_mapping')
        self.declare_parameter('max_range_m', 8.0)
        # 本图有重复走廊：错误盆地也能到 0.06–0.08。真实位姿通常 0.015–0.035，
        # 阈值收紧到 0.05，宁可拒绝让操作者点 2D Pose Estimate，也不播错。
        self.declare_parameter('max_score_m', 0.05)
        self.declare_parameter('prior_score_m', 0.06)
        self.declare_parameter('checkpoint_file',
                               os.path.expanduser('~/.config/fwr/last_pose.yaml'))
        self.declare_parameter('laser_offset_m', 0.21)
        self.declare_parameter('coarse_step_m', 0.2)
        self.declare_parameter('coarse_yaw_deg', 5.0)
        self.declare_parameter('fine_step_m', 0.04)
        self.declare_parameter('fine_yaw_deg', 1.0)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._map = None
        self._scan = None
        self._client = self.create_client(SetInitialPose, '/set_initial_pose')
        self._future = None
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(LaserScan, str(self.get_parameter('scan_topic').value),
                                 self._on_scan, scan_qos)
        self.create_timer(0.5, self._tick)
        self._started = time.monotonic()
        self._seeded = False
        self._gave_up = False
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_ok_at = time.monotonic()
        self.get_logger().info('auto initial pose: waiting for map and scan')

    def _on_map(self, msg):
        self._map = msg

    def _on_scan(self, msg):
        self._scan = msg

    def _tick(self):
        if self._seeded:
            # 底盘桥/里程计重启后 AMCL 的 map→odom 会失效，位姿就没了。
            # 这里做看门狗：连续 10 s 拿不到 map→base 就重新播种。
            if self._tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
                self._tf_ok_at = time.monotonic()
            elif time.monotonic() - self._tf_ok_at > 10.0:
                self.get_logger().warn('localization lost (no map->base); re-seeding')
                self._seeded = False
                self._tf_ok_at = time.monotonic()
            return
        if self._gave_up:
            return
        if self._future is not None:
            if not self._future.done():
                # 服务端没激活时 call_async 的 future 可能永远不完成，超时后重试。
                if time.monotonic() - self._future_at > 5.0:
                    self.get_logger().warn(
                        'auto initial pose: seed call timed out; retrying')
                    self._future = None
                return
            if self._future.done():
                try:
                    self._future.result()
                except Exception as exc:  # noqa: BLE001 - service error is reported
                    self.get_logger().error(f'auto initial pose service failed: {exc}')
                else:
                    self.get_logger().info('auto initial pose: AMCL seeded')
                    self._seeded = True
                self._future = None
            return
        if self._map is None or self._scan is None:
            if time.monotonic() - self._started > 30.0:
                self.get_logger().error(
                    'auto initial pose: no map/scan after 30 s; '
                    'slam localization mode does not need this node')
                self._gave_up = True
            return
        score, x, y, yaw = self._match()
        if score > 9.0:
            self.get_logger().error('auto initial pose: global match failed')
            return
        base_x, base_y, base_yaw = base_pose_from_laser(
            x, y, yaw, float(self.get_parameter('laser_offset_m').value))
        limit = float(self.get_parameter('max_score_m').value)
        if score > limit:
            # 匹配不可信时不要覆盖 AMCL 位姿（覆盖反而制造 kidnap），
            # 交给操作者用 RViz 的 2D Pose Estimate 人工校准。
            self.get_logger().error(
                f'auto initial pose: weak match ({score:.3f} m > {limit:.2f} m); '
                f'NOT seeding, please verify localization manually '
                f'(best guess {base_x:.2f},{base_y:.2f},{math.degrees(base_yaw):.0f} deg)')
            self._seeded = True
            return
        self.get_logger().info(
            f'auto initial pose: match {score:.3f} m -> base '
            f'({base_x:.2f},{base_y:.2f},{math.degrees(base_yaw):.0f} deg)')
        self._send(base_x, base_y, base_yaw)

    def _checkpoint_laser_pose(self):
        """上次保存的 base 位姿换算成激光位姿，作为局部细化的先验。"""
        path = str(self.get_parameter('checkpoint_file').value)
        try:
            with open(path, 'r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
            base_x = float(data['x'])
            base_y = float(data['y'])
            base_yaw = float(data['yaw'])
        except (OSError, KeyError, TypeError, ValueError):
            return None
        offset = float(self.get_parameter('laser_offset_m').value)
        return (base_x + offset * math.cos(base_yaw),
                base_y + offset * math.sin(base_yaw),
                base_yaw + math.pi)

    def _points(self):
        scan = self._scan
        limit = float(self.get_parameter('max_range_m').value)
        return np.array([
            (r * math.cos(scan.angle_min + i * scan.angle_increment),
             r * math.sin(scan.angle_min + i * scan.angle_increment))
            for i, r in enumerate(scan.ranges)
            if math.isfinite(r) and 0.05 < r < limit])

    def _match(self):
        info = self._map.info
        width, height, resolution = info.width, info.height, info.resolution
        origin_x, origin_y = info.origin.position.x, info.origin.position.y
        occupied = (np.array(self._map.data, dtype=np.int16).reshape(height, width) >= 65)
        points = self._points()
        if len(points) < 50:
            self.get_logger().error(f'auto initial pose: only {len(points)} scan points')
            return 9.9, 0.0, 0.0, 0.0
        # 第一级：用上次位姿做局部细化（车没被搬动时最可靠）。
        prior = self._checkpoint_laser_pose()
        if prior is not None:
            score, x, y, yaw = refine_pose(
                occupied, resolution, origin_x, origin_y, points, *prior)
            if score <= float(self.get_parameter('prior_score_m').value):
                self.get_logger().info(
                    f'auto initial pose: checkpoint refined (score {score:.3f} m)')
                return score, x, y, yaw
            self.get_logger().warn(
                f'auto initial pose: checkpoint does not fit (score {score:.3f} m); '
                'falling back to a global search')
        # 第二级：全图搜索 + 局部细化。
        bounds = (origin_x, origin_x + width * resolution,
                  origin_y, origin_y + height * resolution)
        return find_best_pose(
            occupied, resolution, origin_x, origin_y, bounds, points,
            coarse_step=float(self.get_parameter('coarse_step_m').value),
            coarse_yaw_deg=float(self.get_parameter('coarse_yaw_deg').value),
            fine_step=float(self.get_parameter('fine_step_m').value),
            fine_yaw_deg=float(self.get_parameter('fine_yaw_deg').value))

    def _send(self, x, y, yaw):
        request = SetInitialPose.Request()
        request.pose.header.frame_id = 'map'
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.pose.position.x = float(x)
        request.pose.pose.pose.position.y = float(y)
        request.pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        request.pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.02
        cov[35] = 0.004
        request.pose.pose.covariance = cov
        if not self._client.service_is_ready():
            if not self._client.wait_for_service(timeout_sec=1.0):
                return
        self._future = self._client.call_async(request)
        self._future_at = time.monotonic()


def main():
    rclpy.init()
    node = AutoInitialPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
