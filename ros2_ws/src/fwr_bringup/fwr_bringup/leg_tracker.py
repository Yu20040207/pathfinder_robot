"""雷达腿部跟踪节点（P1：**只发布，绝不发速度**）。

2026-11-24 新建。用户需求：人绕到车侧/车后仍要跟。相机视野 60~70°，人一转过去就丢；
360° 雷达是唯一能补这个洞的传感器。本节点先只把「雷达看到的人」发布出来，
用来现场采集数据 + 调参，**不参与控制**（P2 才接进 person_follower）。

安全
----
* 不发布任何 `cmd_vel`，不碰 `motion_enabled` / `manual_enabled`；
* 只订阅 + 只发布只读话题；
* 车可以完全静止，一个人在旁边走就能验证。

话题
----
* 订阅 `/scan_merged`（LaserScan，360° 合并扫描）
* 订阅 `/odom`（只用它的位姿把点变换到 odom 系）
* 订阅 `/robot/vision/persons`（相机身份，用于「相机背书」门控；没有也能跑）
* 发布 `/robot/lidar/persons`（JSON，给页面/日志用）
* 发布 `/robot/lidar/persons_markers`（MarkerArray，给 RViz 看）

自检输出（每秒一行，现场看得懂）::

    簇 3 个 | 最佳腿对 间距 0.31 m 宽 0.12/0.14 m 距离 1.82 m | 轨迹 1：#1(1.85,0.12) v=0.42
"""

from __future__ import annotations

import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from fwr_bringup.leg_track import (DEFAULTS, LegTracker, ang_norm,
                                   cluster_points, describe, detect_pairs,
                                   scan_to_points, to_world)


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LegTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__('leg_tracker')
        self.p = {k: self.declare_parameter(k, v).value
                  for k, v in DEFAULTS.items()}
        self.scan_topic = self.declare_parameter(
            'scan_topic', '/scan_merged').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.out_topic = self.declare_parameter(
            'out_topic', '/robot/lidar/persons').value
        self.markers_topic = self.declare_parameter(
            'markers_topic', '/robot/lidar/persons_markers').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.verbose = bool(self.declare_parameter('verbose', True).value)
        self.verbose_period_s = float(
            self.declare_parameter('verbose_period_s', 1.0).value)
        # 相机水平视场：只用来把框中心近似成方位角（±35° 门限下这个精度够用）
        self.cam_hfov_deg = float(
            self.declare_parameter('cam_hfov_deg', 70.0).value)
        self.cam_stale_s = float(
            self.declare_parameter('cam_stale_s', 1.5).value)

        self.tracker = LegTracker(self.p)
        self.rx = self.ry = self.ryaw = 0.0
        self._odom_at = 0.0
        self._warned_no_odom = 0.0
        self.cam_bearing: float | None = None
        self._cam_at = 0.0
        self._last_clusters = 0
        self._last_pairs = 0
        self._last_print = 0.0

        self.out = self.create_publisher(String, self.out_topic, 10)
        self.markers = self.create_publisher(MarkerArray, self.markers_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        self.create_subscription(String, '/robot/vision/persons',
                                 self.on_persons, 10)
        self.get_logger().info(
            '雷达腿跟踪就绪（只发布、不发速度）：扫描 %s，腿宽 %.2f~%.2f m，'
            '腿距 %.2f~%.2f m，确认 %d 帧（无相机 %d 帧），丢 %.1f s 删除'
            % (self.scan_topic, self.p['leg_w_min'], self.p['leg_w_max'],
               self.p['pair_gap_min'], self.p['pair_gap_max'],
               self.p['confirm_frames'], self.p['confirm_frames_camless'],
               self.p['drop_after_s']))

    # ------------------------------------------------------------ 回调
    def on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.rx = float(pose.position.x)
        self.ry = float(pose.position.y)
        self.ryaw = _yaw_from_quat(pose.orientation)
        self._odom_at = time.monotonic()

    def on_persons(self, msg: String) -> None:
        """相机只提供身份：选中目标在画面里的方位角（近似）。"""
        try:
            data = json.loads(msg.data or '{}')
        except Exception:                        # noqa: BLE001
            return
        sel = int(data.get('sel', -1))
        bearing = None
        for person in data.get('persons', []):
            try:
                if int(person.get('id', -2)) != sel:
                    continue
                box = person.get('box') or {}
                cx = float(box.get('x', 0.5)) + float(box.get('w', 0.0)) / 2.0
                bearing = -(cx - 0.5) * math.radians(self.cam_hfov_deg)
            except (TypeError, ValueError):
                continue
            break
        # 只存**车体系**方位角；换算到 odom 系要等用的时候（车可能在转）
        self.cam_bearing = bearing
        self._cam_at = time.monotonic()

    def on_scan(self, msg: LaserScan) -> None:
        now = time.monotonic()
        # 没 odom 就按「车在原点」算（P1 现场车是静止的，这样也准）
        if (now - self._odom_at) > 2.0 and (now - self._warned_no_odom) > 10.0:
            self.get_logger().warn('还没收到 /odom（>2 s）：先按车在原点计算')
            self._warned_no_odom = now

        pts_local = scan_to_points(msg.ranges, msg.angle_min,
                                   msg.angle_increment, self.p)
        pts = to_world(pts_local, self.rx, self.ry, self.ryaw)
        clusters = cluster_points(pts, self.p)
        pairs, legs = detect_pairs(pts, self.p,
                                  origin=(self.rx, self.ry))
        cam = None
        if (self.cam_bearing is not None
                and (now - self._cam_at) <= self.cam_stale_s):
            # 相机方位是车体系 → 加航向角变成 odom 系，才能和腿对方的位角比
            cam = {'bearing': ang_norm(self.ryaw + self.cam_bearing),
                   'range': None}
        tracks = self.tracker.step(pairs, now, cam)

        self._last_clusters = len(clusters)
        self._last_pairs = len(pairs)
        self._publish(now, tracks, len(legs), len(clusters))
        if self.verbose and (now - self._last_print) >= self.verbose_period_s:
            self._last_print = now
            self.get_logger().info(
                describe(len(clusters), pairs, tracks)
                + ' | 腿簇 %d 待确认 %d'
                % (len(legs), self.tracker.pending_count))

    # ------------------------------------------------------------ 发布
    def _publish(self, now: float, tracks, n_legs: int, n_clusters: int) -> None:
        objs = []
        for tr in tracks:
            dx = tr['x'] - self.rx
            dy = tr['y'] - self.ry
            rng = math.hypot(dx, dy)
            bearing = math.atan2(dy, dx) - self.ryaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            obj = dict(tr)
            obj.update({'range': round(rng, 3),
                        'bearing': round(bearing, 3),
                        'cam': bool(tr.get('cam'))})
            objs.append(obj)
        payload = {'stamp': round(now, 3), 'frame': self.odom_frame,
                   'clusters': n_clusters, 'legs': n_legs,
                   'pairs': self._last_pairs,
                   'pending': self.tracker.pending_count,
                   'tracks': objs}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.out.publish(msg)
        self._publish_markers(objs, now)

    def _publish_markers(self, objs, now: float) -> None:
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for obj in objs:
            m = Marker()
            m.header.frame_id = self.odom_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'lidar_person'
            m.id = int(obj['id'])
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(obj['x'])
            m.pose.position.y = float(obj['y'])
            m.pose.position.z = 0.35
            m.scale.x = m.scale.y = m.scale.z = 0.25
            if obj.get('cam'):
                m.color.r, m.color.g, m.color.b = 0.25, 0.85, 0.35
            elif obj.get('static'):
                m.color.r, m.color.g, m.color.b = 0.55, 0.55, 0.55
            else:
                m.color.r, m.color.g, m.color.b = 0.85, 0.60, 0.20
            m.color.a = 0.85
            m.lifetime.sec = 0
            m.lifetime.nanosec = 300_000_000
            arr.markers.append(m)

            t = Marker()
            t.header = m.header
            t.ns = 'lidar_person_id'
            t.id = 100 + int(obj['id'])
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(obj['x'])
            t.pose.position.y = float(obj['y'])
            t.pose.position.z = 0.75
            t.scale.z = 0.22
            t.color.r = t.color.g = t.color.b = 1.0
            t.color.a = 1.0
            t.text = '%s%d%s' % ('~' if obj.get('static') else '#' if obj.get('cam')
                                 else '#', int(obj['id']),
                                 '' if obj.get('cam') else ' ?')
            arr.markers.append(t)
        self.markers.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LegTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:                        # noqa: BLE001
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
