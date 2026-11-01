"""仿真引导驱动：启动后先让车走一小段，把地图"点着"。

实机由 auto_mapping_supervisor 完成（它还会操作授权门、踢腿、返回）。
仿真里没有 base_bridge，所以用一个更简单的版本：
  等待 delay_s → 沿当前朝向前进 kick_m 米（速度 speed）→ 停住交给探索器。

没有这一步，地图一开始只有出生点附近几平方米空闲，room_explorer 不达
最低空闲面积阈值就不会发目标，形成"要动才能建图、要建图才肯动"的死锁。
"""
from __future__ import annotations

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class BootstrapDrive(Node):
    def __init__(self) -> None:
        super().__init__('bootstrap_drive')
        self.declare_parameter('delay_s', 25.0)
        self.declare_parameter('kick_m', 1.2)
        self.declare_parameter('speed_mps', 0.12)
        self.declare_parameter('timeout_s', 25.0)

        self._start = time.monotonic()
        self._origin = None
        self._state = 'waiting'
        # 必须走手动通道：/cmd_vel_sim 由 sim_cmd_router 以 20 Hz 覆盖，直接发会被清零
        self._pub = self.create_publisher(Twist, '/cmd_vel_manual', 20)
        self.create_subscription(Odometry, '/wheel/odom', self._on_odom, 20)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'引导驱动：{self.get_parameter("delay_s").value:.0f}s 后前进 '
            f'{self.get_parameter("kick_m").value:.1f} m')

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pos = (p.x, p.y)

    def _tick(self) -> None:
        now = time.monotonic() - self._start
        if self._state == 'done':
            return
        if self._state == 'waiting':
            if now >= float(self.get_parameter('delay_s').value):
                self._state = 'driving'
                self._start = time.monotonic()
                self._origin = getattr(self, '_pos', None)
                self.get_logger().info('引导驱动：开始前进')
            return

        moved = 0.0
        if self._origin and getattr(self, '_pos', None):
            moved = math.hypot(self._pos[0] - self._origin[0],
                               self._pos[1] - self._origin[1])
        reached = moved >= float(self.get_parameter('kick_m').value)
        timeout = (time.monotonic() - self._start) >= float(
            self.get_parameter('timeout_s').value)
        if reached or timeout:
            for _ in range(8):
                self._pub.publish(Twist())
            self._state = 'done'
            self.get_logger().info(f'引导驱动：结束（位移 {moved:.2f} m）')
            return

        cmd = Twist()
        cmd.linear.x = float(self.get_parameter('speed_mps').value)
        self._pub.publish(cmd)


def main() -> None:
    import rclpy
    rclpy.init()
    node = BootstrapDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
