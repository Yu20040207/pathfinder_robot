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
from rcl_interfaces.msg import Parameter as RclParameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
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
        # 2026-11-03（用户要求）：倒多远不看固定距离，看"左右转得开没有"。
        # 雷达判到前方左右两侧都 >= turn_clear_m 就认为能转，立刻结束倒车。
        self.declare_parameter('turn_clear_m', 0.90)  # 左右都要达到的净空
        self.declare_parameter('max_back_m', 1.20)    # 硬上限，防止一直倒
        self.declare_parameter('scan_topic', '/scan')

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
        # 自动模式下 manual_enabled=false，手动词道会被 base_bridge 拒绝 →
        # 倒车前临时打开，结束后关回去（与 room_explorer 的脱困做法一致）
        self._param_cli = self.create_client(SetParameters,
                                             '/base_bridge/set_parameters')
        self.create_subscription(Twist, '/cmd_vel_auto_unchecked', self._on_raw, 20)
        self.create_subscription(Twist, '/cmd_vel_auto_forward_filtered',
                                 self._on_filtered, 20)
        self.create_subscription(Bool, '/base/rear_blocked', self._on_rear, 10)
        self.create_subscription(Odometry, '/wheel/odom', self._on_odom, 20)
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import qos_profile_sensor_data
        self._scan = None
        self.create_subscription(LaserScan,
                                 str(self.get_parameter('scan_topic').value),
                                 self._on_scan, qos_profile_sensor_data)
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

    def _on_scan(self, msg) -> None:
        self._scan = msg

    def _turn_clearance(self):
        """返回 (左, 右) 前方净空，单位 m；没数据返回 None。

        雷达倒装：扫描角 0° = 车尾，|angle| > 155° = 车头方向。
        取车头前 120° 弧（150°~210°），左半 150~180、右半 180~210。
        """
        m = self._scan
        if m is None:
            return None
        import math as _math
        left = right = None
        for i, r in enumerate(m.ranges):
            if not (0.05 < r < 3.5):
                continue
            a = _math.degrees(m.angle_min + i * m.angle_increment) % 360.0
            if 150.0 <= a < 180.0:
                left = r if left is None else min(left, r)
            elif 180.0 <= a <= 210.0:
                right = r if right is None else min(right, r)
        if left is None or right is None:
            return None
        return (left, right)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pos = (p.x, p.y)

    def _set_manual(self, enabled: bool) -> None:
        if not self._param_cli.service_is_ready():
            return
        req = SetParameters.Request()
        for name, val in (('motion_enabled', True), ('manual_enabled', enabled)):
            prm = RclParameter()
            prm.name = name
            prm.value.type = ParameterType.PARAMETER_BOOL
            prm.value.bool_value = val
            req.parameters.append(prm)
        self._param_cli.call_async(req)

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
                self._set_manual(True)
                self.get_logger().warn('前停区持续拦截且车尾空旷 → 倒车自救')
            return

        if self._state == 'backing':
            moved = 0.0
            if self._origin and self._pos:
                moved = math.hypot(self._pos[0] - self._origin[0],
                                   self._pos[1] - self._origin[1])
            clear = self._turn_clearance()
            room = (clear is not None and
                    min(clear) >= float(self.get_parameter('turn_clear_m').value))
            cap = max(float(self.get_parameter('back_m').value),
                      float(self.get_parameter('max_back_m').value))
            done = (room or moved >= cap or
                    now - self._state_at >= float(self.get_parameter('max_back_s').value) or
                    self._rear_blocked)
            if done:
                self._pub.publish(Twist())
                self._set_manual(False)
                self._state = 'releasing'
                self._state_at = now
                why = ('左右已够转' if room else
                       '车尾被挡' if self._rear_blocked else '到上限')
                clr = f' 左右净空 {clear[0]:.2f}/{clear[1]:.2f} m' if clear else ''
                self.get_logger().info(
                    f'倒车结束（{moved:.2f} m，{why}{clr}），交还导航')
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
