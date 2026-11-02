"""Select the direction-appropriate Nav2 Collision Monitor output."""

import math
import os
import time

from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .directional_safety_core import DirectionalSafetyCore


class DirectionalSafetySelector(Node):
    def __init__(self):
        super().__init__('directional_safety_selector')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('require_manual_status', True)
        # 纯旋转放行：车头（扫描角 ±180，±30° 扇区）最近障碍超过这个值才放行旋转。
        # 0.38 m ≈ 车体半对角线 0.37 m，旋转扫不到。
        # 0.34：车体前缘 0.30 + 4 cm 余量。0.38（半对角线）偏保守，
        # 现场"明明右转有空间"却被拒；旋转是原地转，前缘不会比 0.30 更近。
        # 雷达装在被转中心**前方 0.21 m**，旋转时车体扫过半径 0.37 m
        # → 雷达读数只要 > 0.37 - 0.21 = 0.16 m 就是安全的。
        # 之前用 0.34（相当于要求 0.55 m 实际余量），难怪"能转却不转"。
        self.declare_parameter('rotate_clear_m', 0.16)
        # 前进被挡的判定距离（= 碰撞监控 front_stop 的外沿 0.45 m）
        self.declare_parameter('forward_block_m', 0.45)
        self._last_scan = None
        self._front_min_m = float('inf')
        self._left_min_m = float('inf')
        self._right_min_m = float('inf')
        # 2026-11-01：默认关闭 —— reverse 监控贴墙必然触发，会否决所有旋转
        self.declare_parameter('require_reverse_monitor', False)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.core = DirectionalSafetyCore(
            require_manual_status=bool(self.get_parameter('require_manual_status').value))
        self.core.require_reverse_monitor = bool(
            self.get_parameter('require_reverse_monitor').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.monitor_clients = {
            direction: self.create_client(GetState, f'/collision_monitor_{direction}/get_state')
            for direction in ('forward', 'reverse')
        }
        self.monitor_queries = {direction: None for direction in self.monitor_clients}
        for direction in self.monitor_clients:
            self.create_subscription(
                TransitionEvent, f'/collision_monitor_{direction}/transition_event',
                lambda _, direction=direction: self.core.observe_monitor(
                    direction, False, time.monotonic()), 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_auto_safe', 10)
        from nav_msgs.msg import Odometry
        self._last_pos = None
        self._last_move_t = 0.0
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self._on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel_auto_unchecked', self._raw, 10)
        self.create_subscription(Twist, '/cmd_vel_auto_forward_filtered',
                                 lambda msg: self._filtered('forward', msg), 10)
        self.create_subscription(Twist, '/cmd_vel_auto_reverse_filtered',
                                 lambda msg: self._filtered('reverse', msg), 10)
        def _on_scan(msg):
            self._last_scan = msg
            self._front_min_m = self._compute_front_min(msg)
            # 反方向的余量：左前（150~180°）与右前（-180~-150°）
            self._left_min_m = self._sector_min(msg, +1)
            self._right_min_m = self._sector_min(msg, -1)
            self.core.observe_scan(time.monotonic())

        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 _on_scan, qos_profile_sensor_data)
        # 2026-11-01：/explore/pause 以前**没有任何节点订阅**（supervisor 在发，
        # 但没人接收），所以按"暂停"车照走。这里接上：暂停 = 输出 0。
        self._paused = False
        # 2026-11-01（用户要求）卡住脱困：原地转身**转过为止**，角度不写死，
        # 哪怕 10° 够用就 10°。每帧检查前方是否已空旷，一空就交回 Nav2。
        self.declare_parameter('escape_turn_rate', 0.35)
        # 2026-11-01（用户反馈"正常行驶时偶发一卡一卡"）：
        # 光看"前方被挡 + 侧面有空间"会误触发 —— 正常行驶时本来就常满足。
        # 必须**真的没在动**持续 escape_after_s 秒才脱困。
        self.declare_parameter('escape_after_s', 2.5)
        self.declare_parameter('odom_topic', '/odom')
        self._escape_active = False
        self._escape_dir = 1.0
        self.create_subscription(Bool, '/explore/pause', self._on_pause, 10)
        self.create_subscription(String, '/explore/state',
                                 lambda msg: self.core.observe_explore_state(msg.data, time.monotonic()), 10)
        self.create_subscription(Bool, '/base/manual_enabled',
                                 lambda msg: self.core.observe_manual(msg.data, time.monotonic()), 10)
        self.create_timer(.05, self._tick)
        self.create_timer(.1, self._poll_monitors)
        self.publisher.publish(Twist())

    def _poll_monitors(self):
        now = time.monotonic()
        for direction, client in self.monitor_clients.items():
            if not client.service_is_ready():
                self.core.observe_monitor(direction, False, now)
                self.monitor_queries[direction] = None
                continue
            query = self.monitor_queries[direction]
            if query is not None and not query.done():
                continue
            if query is not None:
                try:
                    active = query.result().current_state.label == 'active'
                except (AttributeError, RuntimeError):
                    active = False
                self.core.observe_monitor(direction, active, now)
            self.monitor_queries[direction] = client.call_async(GetState.Request())

    def _raw(self, msg):
        self.core.observe_raw(msg.linear.x, msg.angular.z, time.monotonic())

    def _filtered(self, direction, msg):
        self.core.observe_filtered(direction, msg.linear.x, msg.angular.z,
                                   time.monotonic())

    @staticmethod
    def _sector_min(scan, side):
        """side=+1 左前（扫描角 150~180°），-1 右前（-180~-150°）。"""
        n = len(scan.ranges)
        if n == 0:
            return float('inf')
        vals = []
        for i, r in enumerate(scan.ranges):
            a = math.degrees(scan.angle_min + i * scan.angle_increment)
            if side > 0:
                hit = 150.0 <= a <= 180.0 or -180.0 <= a <= -150.0
            else:
                hit = 150.0 <= a <= 180.0 or -180.0 <= a <= -150.0
            if hit and math.isfinite(r) and r > 0.05:
                vals.append(r)
        return min(vals) if vals else float('inf')

    @staticmethod
    def _compute_front_min(scan):
        """车头正前方（扫描角 ±180，**±20°** 窄扇区）最近障碍。

        2026-11-01：原来用 ±30°，把"侧前方 30° 的墙"也算进来了 →
        明明原地能转，却因为旁边有墙被否决。原地旋转时车头正前方
        不会再靠近，真正会扫到的是正前方 ±20° 内的物体。
        """
        n = len(scan.ranges)
        if n == 0:
            return float('inf')
        vals = []
        for i, r in enumerate(scan.ranges):
            a = math.degrees(scan.angle_min + i * scan.angle_increment)
            if abs(abs(a) - 180.0) <= 20.0 and math.isfinite(r) and r > 0.05:
                vals.append(r)
        return min(vals) if vals else float('inf')

    def _on_pause(self, msg) -> None:
        self._paused = bool(msg.data)

    def _on_odom(self, msg) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        now = time.monotonic()
        if self._last_pos is None:
            self._last_pos = (x, y)
            self._last_move_t = now
            return
        if (x - self._last_pos[0]) ** 2 + (y - self._last_pos[1]) ** 2 > 0.02 ** 2:
            self._last_pos = (x, y)
            self._last_move_t = now

    def _is_really_stuck(self, now: float) -> bool:
        """连续 escape_after_s 秒没位移才算卡住。"""
        if self._last_pos is None:
            return False
        return (now - self._last_move_t) >= float(
            self.get_parameter('escape_after_s').value)

    def _tick(self):
        now = time.monotonic()
        self.core.observe_tf(self._tf_ready(), now)
        clear = float(self.get_parameter('rotate_clear_m').value)
        cmd_w = self.core.raw[1] if self.core.raw else 0.0
        # 左转（w>0）要左前空旷；右转（w<0）要右前空旷
        requested = self._left_min_m if cmd_w > 0 else self._right_min_m
        opposite = self._right_min_m if cmd_w > 0 else self._left_min_m
        # 正前方空旷 → 随便转
        self.core.rotate_clear_ok = self._front_min_m >= clear
        # 请求方向本身就空旷 → 直接转（不翻转）
        self.core.rotate_same_side_ok = requested >= clear
        # 请求方向被挡、反方向空旷 → 翻转
        self.core.rotate_flip_ok = (not self.core.rotate_same_side_ok) and opposite >= clear

        # 前进被挡时改原地转：用**前进停车距离**判断（不是旋转阈值！）
        # 前进停车区 0.45 m；旋转只需 0.16 m（雷达前置偏移已扣除）。
        blocked = self._front_min_m < float(self.get_parameter('forward_block_m').value)
        self.core.blocked_rotate_same = blocked and requested >= clear
        self.core.blocked_rotate_ok = blocked and (not self.core.blocked_rotate_same) and opposite >= clear
        linear_x, angular_z = self.core.select(now)

        # ---- 2026-11-01 卡住脱困 ----
        # 场景：Nav2 给了速度，但碰撞监控因为正前方 < 前进停车距离把它清零，
        # 于是车原地不动（监控在选择器之前，选择器无法翻案）。
        # 处理：只要 Nav2 **想要动**、前方确实被挡、且有一侧转身空间足够，
        # 就由选择器自己原地转过去，转到前方空旷为止（角度自适应，10° 也行）。
        front_block_m = float(self.get_parameter('forward_block_m').value)
        raw_lin = abs(self.core.raw[0]) if self.core.raw else 0.0
        wants_motion = raw_lin > 0.02 or abs(cmd_w) > 0.05
        newer = self._left_min_m if cmd_w >= 0 else self._right_min_m
        other = self._right_min_m if cmd_w >= 0 else self._left_min_m
        best_side = max(self._left_min_m, self._right_min_m)
        if self._escape_active:
            if self._front_min_m >= front_block_m:
                self._escape_active = False          # 前方已空，交回 Nav2
            else:
                linear_x = 0.0
                angular_z = self._escape_dir * float(
                    self.get_parameter('escape_turn_rate').value)
        elif (wants_motion and self._front_min_m < front_block_m
              and best_side >= clear and self._is_really_stuck(now)):
            self._escape_active = True
            self._escape_dir = 1.0 if self._left_min_m >= self._right_min_m else -1.0
            linear_x = 0.0
            angular_z = self._escape_dir * float(
                self.get_parameter('escape_turn_rate').value)
        # ---- 脱困结束 ----

        if os.environ.get('FWR_SELECTOR_DEBUG'):
            reason = self.core.last_reason
            if reason != getattr(self, '_last_reason', None):
                self._last_reason = reason
                print(f'[selector] {reason}', flush=True)
        if self._paused:
            linear_x = 0.0
            angular_z = 0.0
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.publisher.publish(command)

    def _tf_ready(self) -> bool:
        try:
            odom_tf = self.tf_buffer.lookup_transform('odom', self.base_frame, Time())
            self.tf_buffer.lookup_transform(self.base_frame, 'laser_front', Time())
        except TransformException:
            return False
        stamp = odom_tf.header.stamp.sec + odom_tf.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        return stamp > 0 and -.1 <= age <= .25


def main():
    rclpy.init()
    node = DirectionalSafetySelector()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
