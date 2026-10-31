"""仿真里的后向超声：把 4 路 3 束窄 lidar（LaserScan）转成 sensor_msgs/Range。

实车上这 4 路来自 DYP-E08（RS485/Modbus）→ `/ultrasonic_rear/<name>`。
仿真里用 gpu_lidar 模拟同样位置和朝向，桥接成 LaserScan 后由本节点转换，
这样 `rear_guard` / `RangeSensorLayer` 等上层代码**完全不用改**。

取 3 束中的最小值（相当于超声锥体内最近的回波），无有效回波时按"无目标"发送
`range = max_range`，与实车 0xFFFD 的语义一致（不是 0）。
"""
from __future__ import annotations

import math

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Range

CHANNELS = ('left_out', 'left_in', 'right_in', 'right_out')


class UltrasonicSim(Node):
    def __init__(self) -> None:
        super().__init__('ultrasonic_sim')
        self.declare_parameter('min_range_m', 0.05)
        self.declare_parameter('max_range_m', 3.50)
        self.declare_parameter('fov_rad', math.radians(60.0))

        self._min_r = float(self.get_parameter('min_range_m').value)
        self._max_r = float(self.get_parameter('max_range_m').value)
        self._fov = float(self.get_parameter('fov_rad').value)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pubs = {}
        for name in CHANNELS:
            self._pubs[name] = self.create_publisher(
                Range, f'/ultrasonic_rear/{name}', 10)
            self.create_subscription(
                LaserScan, f'/sim_us/{name}',
                lambda msg, n=name: self._on_scan(n, msg), qos)

        self.get_logger().info(
            f'仿真超声已启动：4 路 → /ultrasonic_rear/*，'
            f'量程 {self._min_r:.2f}~{self._max_r:.2f} m，FOV {math.degrees(self._fov):.0f}°')

    def _on_scan(self, name: str, msg: LaserScan) -> None:
        ranges = [r for r in msg.ranges
                  if math.isfinite(r) and r >= msg.range_min]
        if ranges:
            value = min(ranges)
        else:
            value = self._max_r          # 无回波 = 无目标（绝不用 0 表示"空"）

        out = Range()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = f'ultrasonic_rear_{name}'
        out.radiation_type = Range.ULTRASOUND
        out.field_of_view = self._fov
        out.min_range = self._min_r
        out.max_range = self._max_r
        out.range = float(max(self._min_r, min(self._max_r, value)))
        self._pubs[name].publish(out)


def main() -> None:
    import rclpy
    rclpy.init()
    node = UltrasonicSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
