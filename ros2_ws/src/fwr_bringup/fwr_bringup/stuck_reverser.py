"""卡死自救：前停区连续拦住指令、且车尾空旷时，主动倒车一小段再交还导航。

现场问题（2026-11-01）：Nav2 持续发"前进+转向"，但前停区（0.32~0.45 m）把整帧
清零 → 车原地不动；而 Nav2 不会自己想到"倒一点再重规划"，脱困逻辑又要等两次
目标失败才触发，于是长时间卡死。

本节点只做一件事：
  · `/cmd_vel_auto_unchecked` 有非零指令（导航在要动作）
  · 但 `/cmd_vel_auto_forward_filtered` 持续为 0（被前停区拦）
  · 且 `/base/rear_blocked` 为 False（后面安全）
  → 在 `/cmd_vel_manual` 上发 v=-0.06 倒车，最多 back_m 米或 back_s 秒
  → 结束后静默 release_s 秒，让 Nav2 重新规划

倒车走手动通道，因此 base_bridge 的车尾联锁仍然生效（rear_blocked 会立刻中断）。
"""
from __future__ import annotations

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


class StuckReverser(Node):
    def __init__(self) -> None:
        super().__init__('stuck_reverser')
        self.declare_parameter('blocked_s', 2.0)     # 被拦多久才动手
        self.declare_parameter('back_m', 0.40)       # 倒多远
        self.declare_parameter('back_speed_mps', 0.06)
        self.declare_parameter('max_back_s', 8.0)
        self.declare_parameter('release_s', 1.5)     # 交还后静默多久
        self.declare_parameter('cooldown_s', 6.0)    # 两次自救的最小间隔

        self._raw_at = 0.0
        self._filt_at = 0.0
        self._rear_blocked = False
        self._pos = None
        self._state = 'idle'
        self._state_at = time.monotonic()
        self._start = None
        self._origin = None
        self._last_action = 0.0

        self._pub = self.create_publisher(Twist, '/cmd_vel_manual', 20)
        self.create_subscription(Twist, '/cmd_vel_auto_unchecked', self._on_raw, 20)
        self.create_subscription(Twist, '/cmd_vel_auto_forward_filtered',
                                 self._on_filtered, 20)
        self.create_subscription(Bool, '/base/rear_blocked', self._on_rear, 10)
        self.create_subscription(Odometry, '/wheel/odom', self._on_odom, 20)
        self.create_timer(0.05, self._tick)
        self.get_logger().info('卡死自救已启动')

    # ---------- 输入 ----------
    def _on_raw(self, msg: Twist) -> None:
        if abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3:
            self._raw_at = time.monotonic()

    def _on_filtered(self, msg: Twist) -> None:
        if abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3:
            self._filt_at = time.monotonic()

    def _on_rear(self, msg: Bool) -> None:
        self._rear_blocked = bool(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pos = (p.x, p.y)

    # ---------- 状态机 ----------
    def _tick(self) -> None:
        now = time.monotonic()
        if self._state == 'idle':
            want = now - self._raw_at < 0.3          # 导航在要动作
            blocked = now - self._filt_at > float(
                self.get_parameter('blocked_s').value)
            cool = now - self._last_action >= float(
                self.get_parameter('cooldown_s').value)
            if want and blocked and not self._rear_blocked and cool:
                self._state = 'backing'
                self._state_at = now
                self._last_action = now
                self._origin = self._pos
                self.get_logger().warn('前停区持续拦截且车尾空旷 → 倒车自救')
            return

        if self._state == 'backing':
            moved = 0.0
            if self._origin and self._pos:
                moved = math.hypot(self._pos[0] - self._origin[0],
                                   self._pos[1] - self._origin[1])
            done = (moved >= float(self.get_parameter('back_m').value) or
                    now - self._state_at >= float(self.get_parameter('max_back_s').value) or
                    self._rear_blocked)
            if done:
                self._pub.publish(Twist())
                self._state = 'releasing'
                self._state_at = now
                self.get_logger().info(f'倒车结束（{moved:.2f} m），交还导航')
                return
            cmd = Twist()
            cmd.linear.x = -abs(float(self.get_parameter('back_speed_mps').value))
            self._pub.publish(cmd)
            return

        if self._state == 'releasing':
            if now - self._state_at >= float(self.get_parameter('release_s').value):
                self._state = 'idle'


def main() -> None:
    import rclpy
    rclpy.init()
    node = StuckReverser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
