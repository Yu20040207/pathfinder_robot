#!/usr/bin/env python3
"""用当前雷达扫描在已保存地图里做全局匹配，算出机器人位姿并发布 /initialpose 给 AMCL。

用法： .venv/bin/python tools/seed_amcl.py <map.yaml> [--publish]
不带 --publish 只打印结果。
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from PIL import Image
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

sys.path.insert(0, 'ros2_ws/src/fwr_bringup')
from fwr_bringup.auto_seed_core import (base_pose_from_laser, find_best_pose,
                                        occupancy_from_grid)


def main():
    yaml_path = sys.argv[1]
    publish = '--publish' in sys.argv
    meta = yaml.safe_load(open(yaml_path, encoding='utf-8'))
    img = np.array(Image.open(yaml_path.rsplit('/', 1)[0] + '/' + meta['image']))
    # pgm: 0=占用, 255=空闲
    grid = np.where(img < 80, 100, np.where(img > 200, 0, -1)).astype(np.int8)
    height, width = grid.shape
    occupied = occupancy_from_grid(grid.ravel(), width, height)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])

    rclpy.init()
    node = Node('seed_amcl')
    scans = []
    q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(LaserScan, '/scan_mapping', lambda m: scans.append(m), q)
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    t = time.time()
    while time.time() - t < 6 and not scans:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not scans:
        print('没收到 /scan_mapping')
        return
    scan = scans[-1]
    pts = []
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < 0.2 or r > 10.0:
            continue
        a = scan.angle_min + i * scan.angle_increment
        pts.append((r * math.cos(a), r * math.sin(a)))
    pts = np.array(pts[::3])
    print(f'扫描点 {len(pts)} 个，地图 {width}x{height} res={res}')

    coarse = .3 if width * height < 400000 else .45
    score, lx, ly, lyaw = find_best_pose(
        occupied, res, ox, oy,
        (ox, ox + width * res, oy, oy + height * res), pts,
        coarse_step=coarse, coarse_yaw_deg=15.)
    bx, by, byaw = base_pose_from_laser(lx, ly, lyaw)
    print(f'匹配结果: score={score:.3f} m  laser=({lx:.2f},{ly:.2f},{math.degrees(lyaw):.1f}°)')
    print(f'          base =({bx:.2f},{by:.2f},{math.degrees(byaw):.1f}°)')
    if not publish:
        print('（未发布；加 --publish 才会设置 AMCL 初始位姿）')
        return
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = float(bx)
    msg.pose.pose.position.y = float(by)
    msg.pose.pose.orientation.z = math.sin(byaw / 2)
    msg.pose.pose.orientation.w = math.cos(byaw / 2)
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.25
    cov[35] = 0.07
    msg.pose.covariance = cov
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    print('已发布 /initialpose')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
