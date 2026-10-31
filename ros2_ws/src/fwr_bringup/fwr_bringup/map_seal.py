#!/usr/bin/env python3
"""把地图上的细小缺口封住，给 Nav2 的静态层用。

问题（2026-10-31 实测）：
  Cartographer 的地图上墙经常缺 1~3 个格子（5~15 cm 的缝）。
  · 全局代价地图 track_unknown_space: false 时这些缝=可通行 → 规划路径直接穿墙
  · 改成 true 后，前沿目标点本身在未知边界上 → 目标被 Nav2 拒绝

解法：
  对障碍格做形态学膨胀（默认 1 格 = 5 cm），把细缝封死，再发布到 /map_sealed。
  门洞（≥0.86 m = 17 格）完全不受影响。
  未知格保持未知，不改成可通行。

参数：
  in_topic / out_topic / dilate_cells / occupied_threshold / period_s
"""
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MapSeal(Node):
    def __init__(self) -> None:
        super().__init__('map_seal')
        self.declare_parameter('in_topic', '/map')
        self.declare_parameter('out_topic', '/map_sealed')
        self.declare_parameter('dilate_cells', 1)
        self.declare_parameter('occupied_threshold', 60)
        self.declare_parameter('period_s', 0.5)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('out_topic').value), qos)
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('in_topic').value), self._on_map, qos)
        self.create_timer(float(self.get_parameter('period_s').value), self._tick)
        self._msg = None
        self.get_logger().info(
            f'地图缝合节点启动：{self.get_parameter("in_topic").value} → '
            f'{self.get_parameter("out_topic").value}，'
            f'膨胀 {self.get_parameter("dilate_cells").value} 格')

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._msg = msg

    def _tick(self) -> None:
        msg = self._msg
        if msg is None:
            return
        # 不做去重：地图一直在变，去重反而可能卡住只发一次（实测）
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)

        thr = int(self.get_parameter('occupied_threshold').value)
        r = max(0, int(self.get_parameter('dilate_cells').value))
        if r > 0:
            occ = data >= thr
            out = data.copy()
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dx) + abs(dy) > r:      # 菱形/圆形近似，避免太方
                        continue
                    ys = slice(max(0, dy), h + min(0, dy))
                    xs = slice(max(0, dx), w + min(0, dx))
                    src = occ[max(0, -dy):h - max(0, dy), max(0, -dx):w - max(0, dx)]
                    out[ys, xs] = np.where(src, 100, out[ys, xs])
            data = out
        sealed = OccupancyGrid()
        sealed.header = msg.header
        sealed.info = msg.info
        sealed.data = data.reshape(-1).astype(np.int8).tolist()
        self._pub.publish(sealed)


def main():
    rclpy.init()
    node = MapSeal()
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
