"""Seed a running slam_toolbox (map_and_localization) via the standard /initialpose topic.

slam_toolbox's LocalizationSlamToolbox subscribes to `initialpose`, so the same
two-stage matching used for AMCL can seed SLAM *after* start-up, then the pose is
verified against the published map and corrected if needed.
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from PIL import Image
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from scipy.ndimage import distance_transform_edt
import tf2_ros


class Seeder(Node):
    def __init__(self, args):
        super().__init__('seed_slam_runtime')
        self.args = args
        self.map_msg = None
        self.scan = None
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        from nav_msgs.msg import OccupancyGrid
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(LaserScan, '/scan_mapping', self._on_scan, scan_qos)
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)

    def _on_map(self, msg):
        self.map_msg = msg

    def _on_scan(self, msg):
        self.scan = msg

    def wait_inputs(self, timeout=30.0):
        deadline = time.time() + timeout
        while time.time() < deadline and (self.map_msg is None or self.scan is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.map_msg is not None and self.scan is not None

    def _grid(self):
        info = self.map_msg.info
        occ = (np.array(self.map_msg.data, dtype=np.int16).reshape(info.height, info.width) >= 65)
        return occ, info.resolution, info.origin.position.x, info.origin.position.y

    def _points(self):
        s = self.scan
        return np.array([(r * math.cos(s.angle_min + i * s.angle_increment),
                          r * math.sin(s.angle_min + i * s.angle_increment))
                         for i, r in enumerate(s.ranges)
                         if math.isfinite(r) and 0.1 < r < self.args.max_range])

    def score(self, occ, res, ox, oy, pts, dist, tx, ty, th):
        c, s = math.cos(th), math.sin(th)
        height, width = occ.shape
        x = c * pts[:, 0] - s * pts[:, 1] + tx
        y = s * pts[:, 0] + c * pts[:, 1] + ty
        ix = ((x - ox) / res).astype(int)
        iy = (height - 1 - (y - oy) / res).astype(int)
        ok = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
        if ok.sum() < len(pts) * 0.5:
            return 9.9
        return float(dist[iy[ok], ix[ok]].mean())

    def current_laser_pose(self):
        try:
            tr = self.tf.lookup_transform('map', 'laser_front', rclpy.time.Time(),
                                          timeout=Duration(seconds=1.0))
        except Exception:
            return None
        t = tr.transform.translation
        q = tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)

    def publish_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.02
        cov[35] = 0.004
        msg.pose.covariance = cov
        self.pub.publish(msg)

    def refine_around(self, occ, res, ox, oy, pts, dist, base, span, yaw_span_deg, step=0.03):
        best = (self.score(occ, res, ox, oy, pts, dist, *base),) + tuple(base)
        for dx in np.arange(-span, span + 1e-9, step):
            for dy in np.arange(-span, span + 1e-9, step):
                for dth in np.radians(np.arange(-yaw_span_deg, yaw_span_deg + 1e-9, 1.0)):
                    v = self.score(occ, res, ox, oy, pts, dist,
                                   base[0] + dx, base[1] + dy, base[2] + dth)
                    if v < best[0]:
                        best = (v, base[0] + dx, base[1] + dy, base[2] + dth)
        return best


def global_search(node, occ, res, ox, oy, pts, dist):
    height, width = occ.shape
    best = (9.9, 0.0, 0.0, 0.0)
    for tx in np.arange(ox, ox + width * res, 0.25):
        for ty in np.arange(oy, oy + height * res, 0.25):
            for th in np.radians(np.arange(-180, 180, 8)):
                v = node.score(occ, res, ox, oy, pts, dist, tx, ty, th)
                if v < best[0]:
                    best = (v, tx, ty, th)
    c0 = best
    for tx in np.arange(c0[1] - 0.25, c0[1] + 0.26, 0.04):
        for ty in np.arange(c0[2] - 0.25, c0[2] + 0.26, 0.04):
            for th in c0[3] + np.radians(np.arange(-8, 8.1, 1.0)):
                v = node.score(occ, res, ox, oy, pts, dist, tx, ty, th)
                if v < best[0]:
                    best = (v, tx, ty, th)
    return best


def laser_to_base(x, y, yaw, offset=0.21):
    byaw = yaw - math.pi
    return (x - offset * math.cos(byaw), y - offset * math.sin(byaw), byaw)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--reference', default='',
                        help='reference map yaml on disk (preferred over /map topic)')
    parser.add_argument('--iterations', type=int, default=4)
    parser.add_argument('--max-score', type=float, default=0.06)
    parser.add_argument('--max-range', type=float, default=8.0)
    args = parser.parse_args()

    rclpy.init()
    node = Seeder(args)
    if not node.wait_inputs():
        print('SEED_FAIL no map/scan', file=sys.stderr)
        return 2
    if args.reference:
        meta = yaml.safe_load(open(args.reference, encoding='utf-8'))
        img = os.path.join(os.path.dirname(args.reference), meta['image'])
        grid = np.array(Image.open(img))
        occ = grid < 65
        res = float(meta['resolution'])
        ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
        print(f'SEED_MAP file {args.reference}')
    else:
        occ, res, ox, oy = node._grid()
        print('SEED_MAP topic /map')
    dist = distance_transform_edt(~occ) * res
    pts = node._points()

    pose = node.current_laser_pose()
    if pose is None:
        # localization 模式下 slam 尚未定位时不会发 map→laser，先用地图+扫描全图搜索。
        print('SEED_SOURCE global (no tf yet)')
        best = global_search(node, occ, res, ox, oy, pts, dist)
        print(f'global residual {best[0]:.3f} at ({best[1]:.2f},{best[2]:.2f})')
        if best[0] > 0.12:
            print('SEED_FAIL global search poor', file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            return 2
        bx, by, byaw = laser_to_base(best[1], best[2], best[3])
        node.publish_pose(bx, by, byaw)
        deadline = time.time() + 4.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    for attempt in range(args.iterations):
        pose = node.current_laser_pose()
        if pose is None:
            break
        best = node.refine_around(occ, res, ox, oy, pts, dist, pose, 0.6, 15.0)
        print(f'iter {attempt}: residual {best[0]:.3f} at ({best[1]:.2f},{best[2]:.2f})')
        if best[0] <= args.max_score:
            print(f'SEEDED score={best[0]:.3f}')
            node.destroy_node()
            rclpy.shutdown()
            return 0
        bx, by, byaw = laser_to_base(best[1], best[2], best[3])
        node.publish_pose(bx, by, byaw)
        deadline = time.time() + 2.5
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    print('SEED_FAIL residual still high', file=sys.stderr)
    return 5


if __name__ == '__main__':
    sys.exit(main())
