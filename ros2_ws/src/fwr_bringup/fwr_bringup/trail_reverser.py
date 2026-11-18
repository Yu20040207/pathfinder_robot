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
from std_srvs.srv import Trigger
from std_msgs.msg import Bool

from . import trail_reverser_core as tc
from . import turn_backup_core as tb
from .rear_guard_core import sector_min_range


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
        # 2026-11-18：转弯让位（要转大角度但车头贴太近 → 直线后退一点腾空间）
        self.declare_parameter('turn_backup_enabled', True)
        self.declare_parameter('turn_blocked_s', 0.5)
        self.declare_parameter('turn_front_close_m', 0.45)
        # 车尾至少要够退「turn_backup_dist_m 的一部分 + 余量」，否则不许起步
        self.declare_parameter('turn_rear_need_m', 0.55)
        # 退到前方净空 >= turn_backup_clear_m 为止；dist/max_s 只是兜底上限
        self.declare_parameter('turn_backup_clear_m', 0.55)
        self.declare_parameter('turn_backup_dist_m', 0.60)
        self.declare_parameter('turn_backup_max_s', 8.0)
        self.declare_parameter('turn_backup_speed_mps', 0.10)
        self.declare_parameter('turn_max_attempts', 3)
        self.declare_parameter('turn_cooldown_s', 3.0)
        self.declare_parameter('rear_scan_topic', '/scan_rear_mapping')
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
        # 2026-11-05：广播"正在让位后退"。frontier_explorer 据此**暂停**任务（而不是中止），
        # 退完再恢复；同一个点让位次数超限则跳过该点（用户要求）。
        self._active_pub = self.create_publisher(Bool, '/reverse/active', 10)
        self._params = self.create_client(SetParameters, '/base_bridge/set_parameters')
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        # 2026-11-18：倒车会把手动通道打开 → base_bridge 会撤销自动授权
        # （_on_velocity 里 manual_enabled=True → _auto_armed=False、_permit.abort()）。
        # 如果不还回去，倒车结束后 Nav2 的指令全被 "not _auto_armed" 拒掉 = 车不动，
        # 表面看是"倒出来就卡住了"。这里记录倒车前的授权状态，结束时按原样恢复。
        self.declare_parameter('restore_auto_arm', True)
        self.declare_parameter('restore_arm_delay_s', 0.6)
        self._auto_armed = False
        self._armed_before = False
        self._mode = 'deadend'          # 'deadend' | 'turn'
        self._turn_attempts = 0
        self._turn_last_end = -1e9
        self._rear_scan = None
        self._rear_min = float('inf')
        self._unblocked_since = None
        # 2026-11-18：'被挡时长'改成累计+衰减，不再要求连续。
        # 现场：原地旋转时碰撞监控不停地在 stop/slowdown 之间抖
        # (停 0.2 s → 放行 0.2 s → 停)，连续计时器每次被清零，永远攒
        # 不到 0.8 s → 车一直转，2.9 s 后车角才蹭上墙。
        self._blocked_acc = 0.0
        self._unblocked_run = 0.0
        self._tick_at = None
        self._rearm_at = None
        self._rearm_tries = 0
        self._rearm_deadline = 0.0
        self._arm_client = self.create_client(Trigger, '/base/arm_auto')
        self.create_subscription(Bool, '/base/auto_armed', self._on_auto_armed, 10)
        self.create_subscription(Bool, '/base/rear_blocked',
                                 lambda m: setattr(self, '_rear_blocked', bool(m.data)), 10)
        # 后雷达是传感器数据：直接用 qos_profile_sensor_data（Best Effort, depth 5）
        self.create_subscription(LaserScan, str(self.get_parameter('rear_scan_topic').value),
                                 self._on_rear_scan, qos_profile_sensor_data)
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

    def _on_rear_scan(self, msg: LaserScan) -> None:
        self._rear_scan = msg
        # 后雷达 yaw=0：扫描角 180° 就是车尾正后方（与 rear_guard 一致）
        self._rear_min = sector_min_range(
            msg.ranges, msg.angle_min, msg.angle_increment, 180.0, 35.0)

    def _on_auto_armed(self, msg: Bool) -> None:
        self._auto_armed = bool(msg.data)

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_raw(self, msg: Twist) -> None:
        self._raw_cmd = msg

    def _on_filtered(self, msg: Twist) -> None:
        self._filtered_cmd = msg

    # ---------- 工具 ----------
    def _side_clearance(self) -> tuple[float, float]:
        """左右前扇区各自的最小净空（雷达倒装：150~180 左前，-180~-150 右前）。

        2026-11-05：原来左右混在一起取 min，导致"右边很空、左边贴墙"也会被判成
        两侧都窄 → 明明能右转却开始后退。现在左右分开返回，判定交给核心用 max。
        """
        if self._scan is None:
            return float('inf'), float('inf')
        left, right = [], []
        for i, r in enumerate(self._scan.ranges):
            if not (math.isfinite(r) and r > 0.05):
                continue
            a = math.degrees(self._scan.angle_min + i * self._scan.angle_increment)
            if 150.0 <= a <= 180.0:
                left.append(r)
            elif -180.0 <= a <= -150.0:
                right.append(r)
        return (min(left) if left else float('inf'),
                min(right) if right else float('inf'))

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
        if self._rearm_at is not None and now >= self._rearm_at:
            self._rearm_at = None
            self._restore_arm()
        self._maybe_start(now)

    def _restore_arm(self) -> None:
        """把倒车时撤销的自动授权还回去，让 Nav2 能接着跑原来的目标。

        2026-11-18 现场：base_bridge 回 ``arm 失败：reason=serial not ready
        core.ready=False``。倒车刚要结束时底盘串口可能还在恢复，原来只调一次
        就放弃 → 导航再也接不上。改成带重试（0.7 s 一次，最长约 8 s）。
        """
        if not self._arm_client.service_is_ready():
            self.get_logger().warn('倒车结束但 /base/arm_auto 不可用 → 需要手动重新开始导航')
            return
        future = self._arm_client.call_async(Trigger.Request())
        future.add_done_callback(self._arm_done)
        self.get_logger().info(
            '倒车结束 → 请求恢复自动授权（第 %d 次）' % (self._rearm_tries + 1))

    def _arm_done(self, future) -> None:
        ok = False
        reason = ''
        try:
            res = future.result()
            ok = bool(res is not None and res.success)
            reason = (res.message if res is not None else '') or ''
        except Exception as exc:                       # noqa: BLE001
            reason = str(exc)
        if ok:
            self._rearm_at = None
            self.get_logger().info('自动授权已恢复（Nav2 可继续原目标）')
            return
        self._rearm_tries += 1
        if (self._rearm_tries >= 8
                or time.monotonic() > self._rearm_deadline):
            self._rearm_at = None
            self.get_logger().error(
                '自动授权恢复失败（%d 次，%s）→ 需要重新点"开始导航"'
                % (self._rearm_tries, reason))
            return
        self._rearm_at = time.monotonic() + 0.7
        self.get_logger().warn(
            '恢复自动授权未成功（%s）→ 0.7 s 后重试（第 %d 次）'
            % (reason, self._rearm_tries))

    def _maybe_start(self, now: float) -> None:
        if self._pose is None or self._scan is None:
            return
        raw = self._raw_cmd
        filt = self._filtered_cmd
        wants_motion = bool(raw and abs(raw.linear.x) + abs(raw.angular.z) > 0.02)
        stopped = bool(filt and abs(filt.linear.x) + abs(filt.angular.z) < 0.01)
        # 累计"被挡"时长：挡住时累加，放行时按 2 倍速衰减（不清零）。
        # 这样"停一下/放行一下"的抖动也能攒够 blocked_s，让位能及时启动。
        dt = 0.0 if self._tick_at is None else max(0.0, now - self._tick_at)
        self._tick_at = now
        if wants_motion and stopped:
            self._unblocked_run = 0.0
        else:
            self._unblocked_run += dt
        self._blocked_acc = tb.accumulate_blocked(
            self._blocked_acc, dt, wants_motion and stopped, self._unblocked_run)
        if wants_motion and stopped:
            self._blocked_since = self._blocked_since or now
            self._unblocked_since = None
        else:
            self._blocked_since = None if self._blocked_acc <= 0.0 else self._blocked_since
            # 车又能正常走了 → 连续 3 s 顺畅后把"转弯让位"次数复位，
            # 否则一次任务里用完 3 次就永久不再让位了。
            moving = bool(filt and abs(filt.linear.x) + abs(filt.angular.z) > 0.02)
            if moving:
                if self._unblocked_since is None:
                    self._unblocked_since = now
                elif now - self._unblocked_since > 3.0:
                    self._turn_attempts = 0
                    self._unblocked_since = None
            else:
                self._unblocked_since = None
            return
        left, right = self._side_clearance()
        front_min = min(left, right)
        # 2026-11-18 现场（第二次）：原来"转弯让位短退"排在前面，只要想转就抢着
        # 只退 0.33 m。但现场是死胡同（两侧窄、车尾只剩 0.40 m），短退既退不够、
        # 又会先贴到后墙 —— 用户原话"只是后退了一下没退到可以够位置转弯"。
        # 现在改成：**能沿轨迹倒车就先沿轨迹倒**（它退到左右都 >=0.9 m 才停，
        # 真正腾出转弯空间），短退只作为"两侧开阔、只差前方一点空间"的兜底。
        ok, why = tc.should_start(
            blocked_for_s=self._blocked_acc,
            side_left_m=left,
            side_right_m=right,
            rear_blocked=self._rear_blocked,
            trail_len_m=self._trail.length(),
            since_last_s=now - self._last_end,
            blocked_s=float(self.get_parameter('blocked_s').value),
            side_tight_m=float(self.get_parameter('side_tight_m').value),
            cooldown_s=float(self.get_parameter('cooldown_s').value))
        if not ok:
            # 沿轨迹倒车不适用 → 再看"短退让位"（两侧开阔时才有意义）
            self._try_turn_backup(now, raw, front_min)
            return
        self._reversing = True
        self._tick_at = None
        self._blocked_acc = 0.0
        self._unblocked_run = 0.0
        self._mode = 'deadend'
        # 记住倒车前的自动授权状态，结束时恢复（否则 Nav2 接不上）
        self._armed_before = self._auto_armed
        self._started_at = now
        self._start_pose = self._pose
        self._reversed_m = 0.0
        self._blocked_acc = 0.0
        self._set_manual(True)
        self._active_pub.publish(Bool(data=True))
        self.get_logger().warn(
            '前方被挡且左右都不足 %.1f m（实测 左 %.2f / 右 %.2f m）→ 开始沿原路后退' % (
                float(self.get_parameter('side_tight_m').value), left, right))

    def _try_turn_backup(self, now: float, raw, front_min: float) -> bool:
        """要转大角度但车头贴太近 → 直线后退一点腾出转弯空间。"""
        if not bool(self.get_parameter('turn_backup_enabled').value):
            return False
        if self._blocked_since is None:
            return False
        # 2026-11-18 现场：车尾只剩 0.40 m 时也照样起步，退 0.33 m 就 rear-close
        # 停住 —— 既没腾出空间还先贴上后墙。要求车尾够退「计划距离 + 余量」。
        if self._rear_min < float(self.get_parameter('turn_rear_need_m').value):
            return False
        ok, why = tb.wants_backup(
            blocked_for_s=now - self._blocked_since,
            raw_linear=raw.linear.x if raw else 0.0,
            raw_angular=raw.angular.z if raw else 0.0,
            front_min_m=front_min,
            rear_min_m=self._rear_min,
            rear_blocked=self._rear_blocked,
            attempts=self._turn_attempts,
            since_last_s=now - self._turn_last_end,
            blocked_s=float(self.get_parameter('turn_blocked_s').value),
            front_close_m=float(self.get_parameter('turn_front_close_m').value),
            rear_need_m=float(self.get_parameter('turn_rear_need_m').value),
            max_attempts=int(self.get_parameter('turn_max_attempts').value),
            cooldown_s=float(self.get_parameter('turn_cooldown_s').value))
        if not ok:
            return False
        self._reversing = True
        self._tick_at = None
        self._blocked_acc = 0.0
        self._unblocked_run = 0.0
        self._mode = 'turn'
        self._armed_before = self._auto_armed
        self._started_at = now
        self._start_pose = self._pose
        self._reversed_m = 0.0
        self._set_manual(True)
        self._active_pub.publish(Bool(data=True))
        self.get_logger().warn(
            '要转向但车头仅 %.2f m（车尾 %.2f m 净空）→ 直线后退让位（第 %d 次）' % (
                front_min, self._rear_min, self._turn_attempts + 1))
        return True

    def _step_turn_backup(self, now: float) -> None:
        x, y, _ = self._pose
        self._reversed_m = math.hypot(x - self._start_pose[0], y - self._start_pose[1])
        left, right = self._side_clearance()
        front_min = min(left, right)
        done, why = tb.backup_done(
            backed_m=self._reversed_m,
            elapsed_s=now - self._started_at,
            rear_min_m=self._rear_min,
            rear_blocked=self._rear_blocked,
            front_min_m=front_min,
            dist_m=float(self.get_parameter('turn_backup_dist_m').value),
            max_s=float(self.get_parameter('turn_backup_max_s').value),
            front_clear_m=float(self.get_parameter('turn_backup_clear_m').value))
        if done:
            self._turn_attempts += 1
            self._turn_last_end = now
            self.get_logger().info(
                '转弯让位结束(%s)：已退 %.2f m，前方净空 %.2f m' % (
                    why, self._reversed_m, front_min))
            self._finish(why, self._reversed_m)
            return
        cmd = Twist()
        cmd.linear.x = -abs(float(self.get_parameter('turn_backup_speed_mps').value))
        cmd.angular.z = 0.0
        self._cmd.publish(cmd)

    def _step_reverse(self, now: float) -> None:
        if self._mode == 'turn':
            self._step_turn_backup(now)
            return
        x, y, yaw = self._pose
        self._reversed_m = math.hypot(x - self._start_pose[0], y - self._start_pose[1])
        left, right = self._side_clearance()
        done, why = tc.should_stop(
            side_left_m=left, side_right_m=right, rear_blocked=self._rear_blocked,
            reversed_m=self._reversed_m, elapsed_s=now - self._started_at,
            turn_clear_m=float(self.get_parameter('turn_clear_m').value),
            max_reverse_m=float(self.get_parameter('max_reverse_m').value),
            max_s=float(self.get_parameter('max_s').value))
        if done:
            self._finish(why, min(left, right))
            return
        tgt = self._trail.target_behind(float(self.get_parameter('target_back_m').value))
        if tgt is None:
            self._finish('trail-empty', min(left, right))
            return
        cmd = Twist()
        cmd.linear.x = -abs(float(self.get_parameter('reverse_speed_mps').value))
        cmd.angular.z = tc.reverse_steering(x, y, yaw, tgt[0], tgt[1])
        self._cmd.publish(cmd)

    def _publish_inactive(self) -> None:
        self._active_pub.publish(Bool(data=False))

    def _finish(self, why: str, side: float) -> None:
        self._stop_cmd()
        self._set_manual(False)
        # manual_enabled 是异步参数服务，等它落地后再恢复自动授权
        if (self._armed_before
                and bool(self.get_parameter('restore_auto_arm').value)):
            self._rearm_tries = 0
            self._rearm_deadline = time.monotonic() + 8.0
            self._rearm_at = time.monotonic() + float(
                self.get_parameter('restore_arm_delay_s').value)
        self._publish_inactive()
        self._reversing = False
        self._last_end = time.monotonic()
        self._blocked_since = None
        self.get_logger().info(
            '后退结束[%s]（%s）：退了 %.2f m' % (
                '转弯让位' if getattr(self, '_mode', '') == 'turn' else '沿轨迹倒车',
                why, self._reversed_m))


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
