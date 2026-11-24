#!/usr/bin/env python3
"""离线分析「雷达腿跟踪」的 bag（2026-11-24 P1 标定）。

用法::

    source /opt/ros/humble/setup.bash
    python3 tools/analyze_leg_bag.py ~/bags_local/leg_demo_20261124/01_static

只读 bag，不连 ROS 网络（用 rosbag2_py 直接读库），输出：

* 每 0.5 s 一行：簇数 / 半径 3 m 内的腿对（到车距离, 腿间距）/ 已确认轨迹
* 汇总：有腿对的帧占比、轨迹出生次数、轨迹丢掉次数、
  **同一个人被拆成两条轨迹的次数**（两对中心相距 < 0.45 m）
"""

from __future__ import annotations

import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rosbag2_py import (ConverterOptions, SequentialReader, StorageOptions,
                        StorageFilter)
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

sys.path.insert(0, 'ros2_ws/src/fwr_bringup')
from fwr_bringup.leg_track import (LegTracker, ang_norm,  # noqa: E402
                                   cluster_points, detect_pairs,
                                   scan_to_points, to_world)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(bag: str, overrides: dict) -> None:
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='sqlite3'),
                ConverterOptions(input_serialization_format='cdr',
                                 output_serialization_format='cdr'))
    reader.set_filter(StorageFilter(topics=['/scan_merged', '/odom']))

    pose = (0.0, 0.0, 0.0)
    tracker = LegTracker(overrides)
    t0 = None
    frames = 0
    frames_with_pair = 0
    frames_with_front = 0
    frames_front_tracked = 0
    births = 0
    drops = 0
    dup_events = 0
    prev_ids: set = set()
    prev_t = 0.0
    next_print = 0.0
    born_at: dict = {}

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t = stamp / 1e9
        if t0 is None:
            t0 = t
        if topic == '/odom':
            msg = deserialize_message(data, Odometry)
            pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                    yaw_of(msg.pose.pose.orientation))
            continue

        scan = deserialize_message(data, LaserScan)
        pts_local = scan_to_points(scan.ranges, scan.angle_min,
                                   scan.angle_increment)
        pts = to_world(pts_local, *pose)
        origin = (pose[0], pose[1])
        clusters = cluster_points(pts)
        pairs, legs = detect_pairs(pts, origin=origin)
        tracks = tracker.step(pairs, t)

        frames += 1
        near = [p for p in pairs if p.r <= 3.0]
        front = [p for p in pairs if p.r <= 3.0
                 and abs(math.degrees(ang_norm(p.bearing - pose[2]))) < 25.0]
        if front:
            frames_with_front += 1
            best = min(front, key=lambda p: p.r)
            if any(math.hypot(best.x - tr['x'], best.y - tr['y']) <= 0.60
                   for tr in tracks):
                frames_front_tracked += 1
        if near:
            frames_with_pair += 1

        # 同一人被拆成两条轨迹：任意两对中心距离 < 0.45 m
        for i in range(len(pairs)):
            for j in range(i + 1, len(pairs)):
                d = math.hypot(pairs[i].x - pairs[j].x,
                               pairs[i].y - pairs[j].y)
                if d < 0.45:
                    dup_events += 1
                    break
            else:
                continue
            break

        ids = {tr['id'] for tr in tracks}
        births += len(ids - prev_ids)
        drops += len(prev_ids - ids)
        prev_ids = ids

        if t - t0 >= next_print:
            next_print = t - t0 + 0.5
            pair_txt = ' '.join('(%.2fm,间距%.2f)' % (p.r, p.gap)
                                for p in sorted(near, key=lambda p: p.r)[:3])
            parts = []
            for tr in tracks:
                dx = tr['x'] - pose[0]
                dy = tr['y'] - pose[1]
                parts.append('#%d@%.2fm/%+.0f°' % (
                    tr['id'], math.hypot(dx, dy),
                    math.degrees(ang_norm(math.atan2(dy, dx) - pose[2]))))
            trk_txt = ' '.join(parts)
            print('%6.1fs 簇%2d 腿%2d 对%2d | %-46s | %s'
                  % (t - t0, len(clusters), len(legs), len(near),
                     pair_txt or '—', trk_txt or '轨迹0'))

    print('\n=== 汇总 ===')
    print('帧数 %d（%.1f s）' % (frames, (t - t0) if frames else 0.0))
    print('有 3 m 内腿对的帧占比：%.0f%%'
          % (100.0 * frames_with_pair / max(1, frames)))
    print('正前方有候选的帧：%d，其中轨迹压在候选上：%.0f%%'
          % (frames_with_front,
             100.0 * frames_front_tracked / max(1, frames_with_front)))
    print('轨迹出生 %d 次 / 消失 %d 次' % (births, drops))
    print('同一个人被拆成两条轨迹的帧数：%d' % dup_events)
    frames_with_pair
    _ = (prev_t, born_at)


if __name__ == '__main__':
    bag = sys.argv[1] if len(sys.argv) > 1 else '.'
    ov = {}
    for arg in sys.argv[2:]:
        k, _, v = arg.partition('=')
        if not _:
            continue
        try:
            ov[k] = int(v)
        except ValueError:
            try:
                ov[k] = float(v)
            except ValueError:
                ov[k] = v in ('1', 'true', 'True')
    main(bag, ov)
