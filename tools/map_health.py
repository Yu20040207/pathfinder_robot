#!/usr/bin/env python3
"""建图健康监视：盯着 map→odom 的修正量。

map→odom 是 slam 对里程计的修正：
  - 平移 <0.15 m、旋转 <3°  → 正常
  - 旋转 >5°                → 扫描匹配开始漂，实时雷达会和地图错开
  - 旋转 >10°               → 地图已不可信，建议回退/重开
"""
import math
import time

import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node

WARN_DEG = 5.0
BAD_DEG = 10.0


def yaw_of(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


def main():
    rclpy.init()
    node = Node('map_health')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    last_report = 0.0
    state = 'ok'
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        now = time.monotonic()
        if now - last_report < 2.0:
            continue
        last_report = now
        try:
            tr = buf.lookup_transform('map', 'odom', rclpy.time.Time(),
                                      timeout=Duration(seconds=0.5))
        except Exception:  # noqa: BLE001
            continue
        t = tr.transform.translation
        yaw = yaw_of(tr.transform.rotation)
        drift = math.hypot(t.x, t.y)
        new_state = 'ok'
        if abs(yaw) > BAD_DEG:
            new_state = 'bad'
        elif abs(yaw) > WARN_DEG:
            new_state = 'warn'
        line = f'[health] map→odom 修正 平移 {drift:.2f} m  旋转 {yaw:+.1f}°'
        if new_state != state:
            if new_state == 'warn':
                line += '  ⚠️ 开始漂移，实时雷达可能已和地图错开'
            elif new_state == 'bad':
                line += '  ❌ 地图已不可信：建议 bash tools/map_rollback.sh 回退'
            else:
                line += '  ✅ 恢复正常'
            state = new_state
        print(line, flush=True)


if __name__ == '__main__':
    main()
