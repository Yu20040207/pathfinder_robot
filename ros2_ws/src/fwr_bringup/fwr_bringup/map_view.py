"""把 slam 的 /map 降采样后以 Best Effort 低频发布，专供 RViz 显示。

见 map_view_core 的说明：目的是避免 RViz 做可靠消费者时在 WiFi 上触发重传风暴。
代价地图与 Cartographer/slam_toolbox 仍使用原始 /map，不受影响。
"""

from __future__ import annotations

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .map_view_core import crop_around, downsample


class MapView(Node):
    def __init__(self) -> None:
        super().__init__('map_view')
        self.declare_parameter('in_topic', '/map')
        self.declare_parameter('out_topic', '/map_view')
        self.declare_parameter('downsample_factor', 2)
        self.declare_parameter('period_s', 2.0)
        # 2026-11-18：改用"保持 50mm 全分辨率 + 只发机器人周围 crop_window_m 米"。
        # 降采样会把墙显示粗一倍；而整张 149 KB 的地图会触发 DDS 分片重传风暴。
        # 裁一小块（约 14 KB）既能保持真实墙厚，又不会撑爆网络。
        self.declare_parameter('crop_window_m', 6.0)

        self._factor = max(1, int(self.get_parameter('downsample_factor').value))
        self._map: OccupancyGrid | None = None
        self._robot = None

        src = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, str(self.get_parameter('in_topic').value),
                                 self._on_map, src)
        dst = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('out_topic').value), dst)
        # 机器人位姿（用于决定裁剪窗口中心）
        from nav_msgs.msg import Odometry
        self.create_subscription(Odometry, '/odom',
                                 lambda m: setattr(self, '_robot',
                                                   (m.pose.pose.position.x,
                                                    m.pose.pose.position.y)), 20)
        period = max(0.2, float(self.get_parameter('period_s').value))
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'map_view: {self.get_parameter("in_topic").value} → '
            f'{self.get_parameter("out_topic").value}'
            f'（1/{self._factor} 降采样, {1/period:.1f} Hz, Best Effort）')

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _tick(self) -> None:
        m = self._map
        if m is None:
            return
        try:
            data, nw, nh = downsample(m.data, m.info.width, m.info.height, self._factor)
        except ValueError as exc:
            self.get_logger().warn(f'downsample 失败: {exc}', throttle_duration_sec=10.0)
            return
        res = m.info.resolution * self._factor
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        crop = float(self.get_parameter('crop_window_m').value)
        if crop > 0 and self._robot is not None and self._factor == 1:
            half = int(crop / 2.0 / res)
            cx = int((self._robot[0] - ox) / res)
            cy = int((self._robot[1] - oy) / res)
            try:
                data, nw, nh, dx, dy = crop_around(
                    m.data, m.info.width, m.info.height, cx, cy, half)
            except ValueError as exc:
                self.get_logger().warn(f'crop 失败: {exc}', throttle_duration_sec=10.0)
                return
            ox += dx * res
            oy += dy * res
        out = OccupancyGrid()
        out.header = m.header
        out.info = m.info
        out.info.width = nw
        out.info.height = nh
        out.info.resolution = res
        out.info.origin.position.x = ox
        out.info.origin.position.y = oy
        out.data = data
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = MapView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
