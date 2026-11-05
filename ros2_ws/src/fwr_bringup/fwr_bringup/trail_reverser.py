"""后退让位：沿"原路"倒退，退到左右都有位置转弯为止。

用户需求（2026-11-05）：
  · 触发：前方被挡 **且 左右都 < 0.5 m**（原地掉头没位置）
  · 动作：**沿原路后退**（不是盲目直退），速度 0.08 m/s
  · 结束：左右都 >= 0.9 m 够转弯了（**不设固定后退距离**）
  · 超声只用来确认"退的路上有没有东西"：车尾 15 cm 被挡立即停
  · 安全上限（防死循环）：1.2 m / 10 s

实现要点：
  · 轨迹来自 /odom，每 5 cm 记一个点，保留最近 5 m（trail_reverser_core）
  · 倒车指令走 /cmd_vel_manual（这样 base_bridge 的车尾联锁自动生效）
  · 结束后关闭手动门，交还自动链路
"""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from . import trail_reverser_core as tc


class TrailReverser(Node):
    def __init__(self) -> None:
        super().__init__('trail_reverser')
        self.declare_parameter('enabled', True)
        self.declare_parameter('reverse_speed_mps', 0.08)
        self.declare_parameter('blocked_s', 1.0)          # 前方被挡多久才介入
        self.declare_parameter('side_tight_m', 0.5)       # 触发：左右都要小于它
        self.declare_parameter('turn_clear_m', 0.9)       # 结束：左右都要达到它
        self.declare_parameter('max_reverse_m', 1.2)      # 安全上限
        self.declare_parameter('max_s', 10.0)             # 安全上限
        self.declare_parameter('cooldown_s', 5.0)
        self.declare_parameter('target_back_m', 0.40)     # 跟踪"身后 0.4 m"的轨迹点
        self.declare_parameter('trail_step_m', 0.05)
        self.declare_parameter('trail_len_m', 5.0)

        self._trail = tc.TrailBuffer(
            float(self.get_parameter('trail_step_m').value),
            float(self.get_parameter('trail_len_m').value))
        self._pose = None            # (x, y, yaw)
        self._scan = None
        self._rear_blocked = False
        self._raw_cmd = None
        self._filtered_cmd = None
        self._blocked_since = None
        self._reversing = False
        self._started_at = 0.0
        self._start_pose = None
        self._reversed_m = 0.0
        self._last_end = 0.0

        self._cmd = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self._params = self.create_client(SetParameters, '/base_bridge/set_parameters')
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/base/rear_blocked',
                                 lambda m: setattr(self, '_rear_blocked', bool(m.data)), 10)
        self.create_subscription(Twist, '/cmd_vel_auto_unchecked', self._on_raw, 20)
        self.create_subscription(Twist, '/cmd_vel_auto_forward_filtered',
                                 self._on_filtered, 20)
        self.create_timer(0.05, self._tick)
        self.get_logger().info('后退让位已启动（%.2f m/s，退到左右都 >= %.1f m 为止）' % (
            self.get_parameter('reverse_speed_mps').value,
            self.get_parameter('turn_clear_m').value))

    # ---------- 输入 ----------
    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self._pose = (p.x, p.y, yaw)
        self._trail.append(p.x, p.y)

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_raw(self, msg: Twist) -> None:
        self._raw_cmd = msg

    def _on_filtered(self, msg: Twist) -> None:
        self._filtered_cmd = msg

    # ---------- 工具 ----------
    def _side_min(self) -> float:
        """左右前扇区的最小净空（雷达倒装：150~180 左前，-180~-150 右前）。"""
        if self._scan is None:
            return float('inf')
        vals = []
        for i, r in enumerate(self._scan.ranges):
            a = math.degrees(self._scan.angle_min + i * self._scan.angle_increment)
            if (150.0 <= a <= 180.0) or (-180.0 <= a <= -150.0):
                if math.isfinite(r) and r > 0.05:
                    vals.append(r)
        return min(vals) if vals else float('inf')

    def _set_manual(self, enabled: bool) -> None:
        if not self._params.service_is_ready():
            return
        req = SetParameters.Request()
        m = ParamMsg()
        m.name = 'manual_enabled'
        m.value.type = ParameterType.PARAMETER_BOOL
        m.value.bool_value = bool(enabled)
        req.parameters.append(m)
        self._params.call_async(req)

    def _stop_cmd(self) -> None:
        self._cmd.publish(Twist())

    # ---------- 主循环 ----------
    def _tick(self) -> None:
        if not bool(self.get_parameter('enabled').value):
            return
        now = time.monotonic()
        if self._reversing:
            self._step_reverse(now)
            return
        self._maybe_start(now)

    def _maybe_start(self, now: float) -> None:
        if self._pose is None or self._scan is None:
            return
        raw = self._raw_cmd
        filt = self._filtered_cmd
        wants_motion = bool(raw and abs(raw.linear.x) + abs(raw.angular.z) > 0.02)
        stopped = bool(filt and abs(filt.linear.x) + abs(filt.angular.z) < 0.01)
        if wants_motion and stopped:
            self._blocked_since = self._blocked_since or now
        else:
            self._blocked_since = None
            return
        side = self._side_min()
        ok, why = tc.should_start(
            blocked_for_s=now - self._blocked_since,
            side_min_m=side,
            rear_blocked=self._rear_blocked,
            trail_len_m=self._trail.length(),
            since_last_s=now - self._last_end,
            blocked_s=float(self.get_parameter('blocked_s').value),
            side_tight_m=float(self.get_parameter('side_tight_m').value),
            cooldown_s=float(self.get_parameter('cooldown_s').value))
        if not ok:
            return
        self._reversing = True
        self._started_at = now
        self._start_pose = self._pose
        self._reversed_m = 0.0
        self._set_manual(True)
        self.get_logger().warn(
            '前方被挡且左右都不足 %.1f m（实测 %.2f m）→ 开始沿原路后退' % (
                float(self.get_parameter('side_tight_m').value), side))

    def _step_reverse(self, now: float) -> None:
        x, y, yaw = self._pose
        self._reversed_m = math.hypot(x - self._start_pose[0], y - self._start_pose[1])
        side = self._side_min()
        done, why = tc.should_stop(
            side_min_m=side, rear_blocked=self._rear_blocked,
            reversed_m=self._reversed_m, elapsed_s=now - self._started_at,
            turn_clear_m=float(self.get_parameter('turn_clear_m').value),
            max_reverse_m=float(self.get_parameter('max_reverse_m').value),
            max_s=float(self.get_parameter('max_s').value))
        if done:
            self._finish(why, side)
            return
        tgt = self._trail.target_behind(float(self.get_parameter('target_back_m').value))
        if tgt is None:
            self._finish('trail-empty', side)
            return
        cmd = Twist()
        cmd.linear.x = -abs(float(self.get_parameter('reverse_speed_mps').value))
        cmd.angular.z = tc.reverse_steering(x, y, yaw, tgt[0], tgt[1])
        self._cmd.publish(cmd)

    def _finish(self, why: str, side: float) -> None:
        self._stop_cmd()
        self._set_manual(False)
        self._reversing = False
        self._last_end = time.monotonic()
        self._blocked_since = None
        self.get_logger().info(
            '后退结束（%s）：退了 %.2f m，左右净空 %.2f m' % (why, self._reversed_m, side))


def main() -> None:
    rclpy.init()
    node = TrailReverser()
    try:
        rclpy.spin(node)
    finally:
        try:
            node._stop_cmd()
            node._set_manual(False)
        except Exception:                              # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
