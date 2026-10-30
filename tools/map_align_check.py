#!/usr/bin/env python3
"""检查「实时雷达点」与「地图墙线」是否重合 —— 建图/定位的健康度指标。

用法：
  .venv/bin/python tools/map_align_check.py            # 单次检查并给结论
  .venv/bin/python tools/map_align_check.py --watch    # 每 5 秒打印一次

判据（到最近地图障碍格的距离）：
  中位数 < 10 cm  → 对齐良好，可继续
  10~20 cm        → 开始漂移，注意
  > 20 cm         → 已错位，地图/定位不可信，建议回退或重扫

为什么需要它：RViz 里肉眼很难判断「差一点」还是「没对上」，
而 map→odom 的修正量正常时也可能很大（IMU 航向尺度误差累积），
所以看修正量会误判；直接量扫描与墙的距离才是准的。
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import LaserScan


def measure(node, buf, maps, scans):
    if not maps or not scans:
        return None
    m = maps[-1]
    s = scans[-1]
    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    w, h = m.info.width, m.info.height
    grid = np.array(m.data).reshape(h, w)
    dmap = distance_transform_edt(grid <= 50) * res
    if not buf.can_transform('map', 'laser_front', rclpy.time.Time()):
        return None
    tr = buf.lookup_transform('map', 'laser_front', rclpy.time.Time(),
                              timeout=Duration(seconds=2))
    tx, ty = tr.transform.translation.x, tr.transform.translation.y
    qw, qx, qy, qz = (tr.transform.rotation.w, tr.transform.rotation.x,
                      tr.transform.rotation.y, tr.transform.rotation.z)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    dists = []
    for i, r in enumerate(s.ranges):
        a = s.angle_min + i * s.angle_increment
        if not math.isfinite(r) or r < 0.1 or r > 6.0:
            continue
        px = tx + r * math.cos(a + yaw)
        py = ty + r * math.sin(a + yaw)
        cx, cy = int((px - ox) / res), int((py - oy) / res)
        if 0 <= cy < h and 0 <= cx < w:
            dists.append(dmap[cy, cx])
    if not dists:
        return None
    a = np.array(dists)
    return {
        'n': int(a.size),
        'median': float(np.median(a)),
        'mean': float(a.mean()),
        'within10': float((a < 0.10).mean()),
        'within20': float((a < 0.20).mean()),
    }


def verdict(r):
    if r is None:
        return 'no_data', '没有数据（地图或扫描未就绪，或 TF 缺失）'
    if r['median'] < 0.10:
        return 'ok', '对齐良好'
    if r['median'] < 0.20:
        return 'warn', '开始漂移'
    return 'bad', '已错位，地图/定位不可信'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--watch', action='store_true', help='每 5 秒重复检查')
    args = ap.parse_args()

    rclpy.init()
    node = Node('map_align_check')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    maps, scans = [], []
    mq = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE)
    sq = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(OccupancyGrid, '/map', lambda m: maps.append(m), mq)
    node.create_subscription(LaserScan, '/scan_mapping', lambda m: scans.append(m), sq)

    t0 = time.time()
    while time.time() - t0 < 8 and (not maps or not scans):
        rclpy.spin_once(node, timeout_sec=0.1)
    if not maps or not scans:
        print('错误：8 秒内没拿到 /map 或 /scan_mapping')
        rclpy.shutdown()
        return 2

    code = 0
    while True:
        for _ in range(15):
            rclpy.spin_once(node, timeout_sec=0.05)
        r = measure(node, buf, maps, scans)
        state, text = verdict(r)
        if r is None:
            print(f'[{state}] {text}')
        else:
            print(f'[{state}] {text}  中位 {r["median"]*100:.1f} cm  平均 {r["mean"]*100:.1f} cm  '
                  f'<10cm {r["within10"]*100:.0f}%  <20cm {r["within20"]*100:.0f}%  ({r["n"]} 点)')
        code = {'ok': 0, 'warn': 1, 'bad': 2, 'no_data': 3}[state]
        if not args.watch:
            break
        time.sleep(5.0)
    rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
