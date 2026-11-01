"""仿真底盘入口：把自动链（/cmd_vel_auto_safe）和手动链（/cmd_vel_manual）合成
一路 `/cmd_vel_sim` 送给 Gazebo，并模拟 base_bridge 的**车尾联锁**。

实机上 `/cmd_vel_auto_safe` 进 `base_bridge`，由它做：
  · 手动/自动通道择一
  · 后退时检查 `/base/rear_blocked`（车尾超声守卫）
  · 串口下发
仿真里没有 ESP32，本节点只保留前两条（可验证的部分），第三条换成发到 Gazebo。

这样仿真里跑的**门控 / 碰撞监控 / 方向选择器 / 车尾联锁**都是实机同一套逻辑。
"""
from __future__ import annotations

import time

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class SimCmdRouter(Node):
    def __init__(self) -> None:
        super().__init__('sim_cmd_router')
        self.declare_parameter('manual_window_s', 0.3)

        self._manual_at = 0.0
        self._manual = Twist()
        self._auto = Twist()
        self._rear_blocked = False

        self._pub = self.create_publisher(Twist, '/cmd_vel_sim', 20)
        self.create_subscription(Twist, '/cmd_vel_manual', self._on_manual, 20)
        self.create_subscription(Twist, '/cmd_vel_auto_safe', self._on_auto, 20)
        self.create_subscription(Bool, '/base/rear_blocked', self._on_rear, 10)
        self.create_timer(0.05, self._tick)          # 20 Hz

    # ---------- 回调 ----------
    def _on_manual(self, msg: Twist) -> None:
        self._manual = msg
        self._manual_at = time.monotonic()

    def _on_auto(self, msg: Twist) -> None:
        self._auto = msg

    def _on_rear(self, msg: Bool) -> None:
        self._rear_blocked = bool(msg.data)

    # ---------- 合成 ----------
    def _tick(self) -> None:
        now = time.monotonic()
        window = float(self.get_parameter('manual_window_s').value)
        cmd = Twist()

        if now - self._manual_at <= window:
            cmd.linear.x = self._manual.linear.x
            cmd.angular.z = self._manual.angular.z
        else:
            cmd.linear.x = self._auto.linear.x
            cmd.angular.z = self._auto.angular.z

        # 车尾联锁：任何后退只要车尾近障就清零（与 base_bridge 一致）
        if cmd.linear.x < 0.0 and self._rear_blocked:
            self.get_logger().warn('车尾近障：禁止后退（仿真联锁）',
                                   throttle_duration_sec=5.0)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self._pub.publish(cmd)


def main() -> None:
    import rclpy
    rclpy.init()
    node = SimCmdRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
