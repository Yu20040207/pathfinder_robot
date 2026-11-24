#!/usr/bin/env python3
"""等某个话题**真的在按频率出数据**再放行（替代写死的 sleep / TimerAction）。

用法（launch 里和 OnProcessExit 串起来，就能做到"上一步真就绪了再起下一步"）：

    gate = ExecuteProcess(cmd=[py, 'tools/wait_ready.py',
                              '--topic', '/scan_merged',
                              '--msg-type', 'sensor_msgs/msg/LaserScan',
                              '--min-hz', '8', '--window', '3.0'])
    RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=[slam_node]))

为什么要它：`real_mapping.launch.py` 里 slam_toolbox 被 `TimerAction(period=35.0)`
硬延后 35 秒 —— 因为栈刚起时后雷达/scan_merger 还没出数据，SLAM 用"半圈"首帧
就把地图窗口定死，车不动时地图永远缺一半。实测双雷达通常 5~8 秒就稳了，
白等 27 秒。改成"话题达到频率就放行"，快时快、慢时也不误事。

退出码：0 = 就绪；1 = 超时（调用方仍可继续，或按失败处理）。
"""

import argparse
import importlib
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def _msg_class(type_name: str):
    """'sensor_msgs/msg/LaserScan' → LaserScan 类。"""
    parts = type_name.replace('/', '.').split('.')
    if len(parts) != 3:
        raise SystemExit(f'消息类型要写成 pkg/msg/Name，收到: {type_name}')
    pkg, _mid, name = parts
    return getattr(importlib.import_module(f'{pkg}.msg'), name)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', required=True)
    ap.add_argument('--msg-type', default='sensor_msgs/msg/LaserScan')
    ap.add_argument('--min-hz', type=float, default=5.0)
    ap.add_argument('--window', type=float, default=3.0,
                    help='统计窗口（秒），窗口内平均频率达标才算就绪')
    ap.add_argument('--timeout-s', type=float, default=60.0)
    ap.add_argument('--label', default='', help='日志里显示的名字（可选）')
    args, _ = ap.parse_known_args(argv)

    rclpy.init()
    node = Node('wait_ready')
    log = node.get_logger()
    stamps = deque()
    got = []

    def _cb(msg):                                       # noqa: ANN001
        stamps.append(time.monotonic())
        got.append(1)

    cls = _msg_class(args.msg_type)
    node.create_subscription(cls, args.topic, _cb, qos_profile_sensor_data)

    t0 = time.monotonic()
    ok = False
    while time.monotonic() - t0 < args.timeout_s:
        rclpy.spin_once(node, timeout_sec=0.2)
        now = time.monotonic()
        while stamps and now - stamps[0] > args.window:
            stamps.popleft()
        # 窗口要填满才有意义（刚启动那 1~2 帧不能算频率）
        filled = now - t0 >= args.window
        if filled and len(stamps) / args.window >= args.min_hz:
            ok = True
            break

    waited = time.monotonic() - t0
    name = args.label or args.topic
    if ok:
        log.info('%s 已就绪：%.0f Hz（等了 %.1f s，窗口 %.1f s）'
                 % (name, len(stamps) / max(args.window, 1e-6), waited, args.window))
    else:
        log.error('%s 在 %.0f s 内没达到 %.0f Hz（收到 %d 帧），继续启动但可能缺数据'
                  % (name, args.timeout_s, args.min_hz, sum(got)))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
