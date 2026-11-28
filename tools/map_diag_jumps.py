#!/usr/bin/env python3
"""建图诊断：从 bag 里找 `map→odom` 位姿跳变（2026-11-28「整张图突然转 ~30°」）。

用法（在车上跑，需要 bag 目录）：

    python3 tools/map_diag_jumps.py map_diag/bag_20261128_181211

输出的三块：
1. 每次跳变的时间、角度、当时的位姿 —— 用来区分「单帧扫描匹配跳错峰」和「回环修正」；
2. 跳变前后 ±1 s 的细节（含 `/scan_merged` 的实际到达间隔、odom/IMU 角速度）；
3. 全局统计：最大单步跳变、角速度超过 0.6 rad/s 的时段（建图摇杆上限）。

判据：
  slam 以 50 Hz 发 `map→odom`，两次之间车最多转 0.6 rad/s × 0.02 s = 0.69°，
  所以「相邻两帧差 > 2°」就是一次真实跳变（不是正常运动）。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

JUMP_DEG = 2.0          # 相邻 map→odom 样本的角差阈值
TURN_CAP = 0.60         # 建图摇杆转向上限 rad/s


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def read(bag: Path):
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    msg_cls = {name: get_message(t) for name, t in types.items() if t.startswith('tf2_msgs')}

    tf = []        # (t, parent, child, x, y, yaw)
    scans = []     # (t)
    odom_rates = []  # (t, yaw_rate)  来自 /odom twist
    imu_rates = []   # (t, yaw_rate)  来自 /imu/data_raw
    while rd.has_next():
        topic, data, stamp = rd.read_next()
        t = stamp / 1e9
        if topic in ('/tf', '/tf_static'):
            cls = msg_cls.get(topic)
            if cls is None:
                continue
            for tr in deserialize_message(data, cls).transforms:
                trs = tr.transform.translation
                tf.append((t, tr.header.frame_id, tr.child_frame_id,
                           trs.x, trs.y, yaw_of(tr.transform.rotation)))
        elif topic == '/scan_merged':
            scans.append(t)
        elif topic == '/odom':
            cls = msg_cls.get(topic)
            if cls is None:
                continue
            m = deserialize_message(data, cls)
            odom_rates.append((t, m.twist.twist.angular.z))
        elif topic == '/imu/data_raw':
            cls = msg_cls.get(topic)
            if cls is None:
                continue
            m = deserialize_message(data, cls)
            imu_rates.append((t, m.angular_velocity.z))
    return tf, scans, odom_rates, imu_rates


def series(tf, parent, child):
    return [(t, x, y, yaw) for t, p, c, x, y, yaw in tf if p == parent and c == child]


def near(rows, t0, span):
    return [(t, v) for t, v in rows if abs(t - t0) <= span]


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    bag = Path(sys.argv[1])
    tf, scans, odom_rates, imu_rates = read(bag)
    mo = series(tf, 'map', 'odom')
    ob = series(tf, 'odom', 'base_footprint')
    if not mo:
        print('bag 里没有 map→odom：建图模式是不是没真正跑起来？')
        return 1
    t0 = mo[0][0]
    print(f'bag: {bag}')
    print(f'map→odom 样本 {len(mo)} 个，时长 {mo[-1][0] - t0:.1f} s；'
          f'/scan_merged {len(scans)} 帧；odom {len(odom_rates)} 帧；imu {len(imu_rates)} 帧')

    jumps = []
    for (ta, xa, ya, ya_), (tb, xb, yb, yb_) in zip(mo, mo[1:]):
        d = norm(yb_ - ya_)
        if abs(math.degrees(d)) >= JUMP_DEG:
            jumps.append((tb - t0, math.degrees(d), xb, yb, yb_, tb))
    print(f'\n单步角差 ≥ {JUMP_DEG}° 的跳变：{len(jumps)} 次')
    for rel, d, x, y, yaw, t_abs in jumps[:12]:
        print(f'  t={rel:7.2f}s  Δ={d:+7.2f}°  当时位姿 x={x:+.2f} y={y:+.2f} yaw={math.degrees(yaw):+7.1f}°')

    total = sum(abs(d) for _, d, *_ in jumps)
    print(f'\n跳变角度绝对值合计 {total:.1f}°')
    if jumps:
        worst = max(jumps, key=lambda r: abs(r[1]))
        print(f'最大一次 {worst[1]:+.1f}° @ t={worst[0]:.2f}s')

    # 是否存在「大跳变」——一次 >=10° 通常就是这次事故的形状
    big = [r for r in jumps if abs(r[1]) >= 10.0]
    print(f'≥10° 的大跳变：{len(big)} 次'
          + ('（这就是"整张图突然转一下"）' if big else ''))

    if scans:
        gaps = [(b - a) for a, b in zip(scans, scans[1:])]
        worst_gap = max(gaps) if gaps else 0.0
        print(f'/scan_merged 最大到达间隔 {worst_gap * 1000:.0f} ms（正常 100 ms）')

    fast = [t for t, w in odom_rates if abs(w) > TURN_CAP * 1.05]
    print(f'odom 角速度超过 {TURN_CAP} rad/s 的帧：{len(fast)}'
          + (f'（最大 {max(abs(w) for _, w in odom_rates):.2f} rad/s）'
             if odom_rates else ''))

    if imu_rates and odom_rates:
        # 用 0.2 s 分箱比较 IMU 与编码器航向角速度：差异大 = 打滑或轮子方向不对
        bins: dict[int, list[float]] = {}
        for t, w in odom_rates:
            bins.setdefault(int((t - t0) * 5), []).append(w)
        imu_bins: dict[int, list[float]] = {}
        for t, w in imu_rates:
            imu_bins.setdefault(int((t - t0) * 5), []).append(w)
        diffs = []
        for k, ws in bins.items():
            if k in imu_bins and ws and imu_bins[k]:
                diffs.append(abs(sum(ws) / len(ws) - sum(imu_bins[k]) / len(imu_bins[k])))
        if diffs:
            diffs.sort()
            print(f'编码器 ω 与 IMU ω 的分箱差：中位 {diffs[len(diffs) // 2]:.3f} rad/s，'
                  f'P95 {diffs[int(len(diffs) * 0.95) - 1]:.3f} rad/s'
                  '（差得大 = 打滑或右轮编码器方向不对）')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
