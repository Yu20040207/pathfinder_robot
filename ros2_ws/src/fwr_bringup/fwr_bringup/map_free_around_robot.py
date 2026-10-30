#!/usr/bin/env python3
"""把地图上「机器人脚下」那一小片强制标成空闲，供前沿探索使用。

问题（2026-10-30 实测）：Cartographer 只画雷达扫到的格子，车身正下方和正后方
没有回波，因此机器人所在格子是**未知**。explore_lite 的前沿搜索必须从
「机器人附近的空闲格」出发，一旦该格不是空闲就报
`Could not find nearby clear cell to start search` 并放弃探索。

本节点订阅 /map（transient_local），把机器人位姿周围 radius 米内的格子改成
空闲(0)，发布到 /map_explore 给 explore_lite 用。原始 /map 不受影响。
"""
import math

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MapFreeAroundRobot(Node):
    def __init__(self) -> None:
        super().__init__('map_free_around_robot')
        self.declare_parameter('in_topic', '/map')
        self.declare_parameter('out_topic', '/map_explore')
        self.declare_parameter('radius_m', 0.45)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self._radius = float(self.get_parameter('radius_m').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)

        in_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('out_topic').value), in_qos)
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('in_topic').value),
            self._on_map, in_qos)
        self._buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._buf, self)

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            tr = self._buf.lookup_transform(self._map_frame, self._base_frame,
                                            rclpy.time.Time(),
                                            timeout=Duration(seconds=0.2))
        except Exception:                                   # noqa: BLE001
            return                                          # 还没 TF，先不发布
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        rx, ry = tr.transform.translation.x, tr.transform.translation.y
        cx, cy = int((rx - ox) / res), int((ry - oy) / res)
        r = max(1, int(round(self._radius / res)))
        y0, y1 = max(0, cy - r), min(h, cy + r + 1)
        x0, x1 = max(0, cx - r), min(w, cx + r + 1)
        if y1 <= y0 or x1 <= x0:
            return
        yy, xx = np.ogrid[y0 - cy:y1 - cy, x0 - cx:x1 - cx]
        disc = (xx * xx + yy * yy) <= r * r
        sub = data[y0:y1, x0:x1]
        sub[disc & (sub != 0)] = 0                          # 未知/障碍 → 空闲

        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        out.data = data.reshape(-1).astype(np.int8).tolist()
        self._pub.publish(out)


def main():
    rclpy.init()
    node = MapFreeAroundRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
