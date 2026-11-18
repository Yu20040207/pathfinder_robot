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

from . import maneuver_core as mc
from . import trail_reverser_core as tc
from .rear_guard_core import sector_min_range


class TrailReverser(Node):
    def __init__(self) -> None:
        super().__init__('trail_reverser')
        self.declare_parameter('enabled', True)
        # 倒车时的最大角速度：太大就变成原地打转（v=-0.08/w=-0.5 实测退 0.00 m）
        self.declare_parameter('reverse_max_w', 0.15)
        self.declare_parameter('reverse_speed_mps', 0.08)
        self.declare_parameter('blocked_s', 1.0)          # 前方被挡多久才介入
        self.declare_parameter('side_tight_m', 0.5)       # 触发：左右都要小于它
        self.declare_parameter('turn_clear_m', 0.9)       # 结束：左右都要达到它
        self.declare_parameter('max_reverse_m', 1.2)      # 安全上限
        self.declare_parameter('max_s', 10.0)             # 安全上限
        self.declare_parameter('cooldown_s', 5.0)
        # ---- 2026-11-18 统一判据（用户："执行前就判断好是后退还是直接转弯"）----
        self.declare_parameter('turn_need_m', 0.40)     # 两侧都要 >= 它才能就地转
        # 0.45 → 0.40（2026-11-18 用户批准）：车角扫掠半径实测 0.35 m，
        # 0.45 多留 10 cm 导致「左 1.14 / 右 0.43」也判成转不开 →
        # 白退 3 cm、停 2~3 s。0.40 = 0.35 + 5 cm 余量，符合用户
        # 「任一侧够宽就直接转，只有两侧都不够才后退」的要求。
        self.declare_parameter('front_need_m', 0.45)    # 前方也要 >= 它
        self.declare_parameter('rear_need_m', 0.55)     # 车尾要退得动
        # 前方必须受限（门口前方是开阔的，死胡同前方是墙）才允许"想转但转不开"
        self.declare_parameter('turn_front_need_m', 1.30)
        self.declare_parameter('turn_intent_radps', 0.20)  # 多少算"想转弯"
        self.declare_parameter('backup_max_m', 0.80)    # 单次后退距离上限
        self.declare_parameter('backup_max_s', 12.0)    # 单次后退时长上限
        self.declare_parameter('backup_rear_stop_m', 0.20)  # 车尾近到这里必须停
        # 2026-11-18：转弯让位（要转大角度但车头贴太近 → 直线后退一点腾空间）
        self.declare_parameter('turn_backup_enabled', True)
        self.declare_parameter('turn_blocked_s', 0.5)
        # 2026-11-18（用户批准方案 A）：转弯净空守卫。
        # 现场两次都是"转弯时车角蹭墙"：front_stop 内边缘 0.40 m > 车角半径 0.35 m，
        # 多边形只覆盖正前 ±28.8°，转弯时角部完全没保护；把半宽加到 0.24 又会
        # 过门误停车（42 次实测）。所以改成行为侧拦截：想转但两侧太窄就先退出去。
        self.declare_parameter('turn_guard_enabled', True)
        self.declare_parameter('turn_guard_radps', 0.30)    # 角速度超过它算"在转弯"
        self.declare_parameter('turn_guard_lateral_m', 0.45)  # 侧向净空低于它转不开
        self.declare_parameter('turn_guard_s', 0.6)         # 持续这么久才动作（防空触发）
        self.declare_parameter('turn_front_close_m', 0.45)
        # 直行也顶到这么近（雷达原始距离）就算"走不通"，允许后退让位
        self.declare_parameter('turn_front_panic_m', 0.30)
        # 车尾至少要够退「turn_backup_dist_m 的一部分 + 余量」，否则不许起步
        self.declare_parameter('turn_rear_need_m', 0.55)
        # 退到前方净空 >= turn_backup_clear_m 为止；dist/max_s 只是兜底上限
        self.declare_parameter('turn_backup_clear_m', 0.55)
        self.declare_parameter('turn_backup_dist_m', 0.60)
        self.declare_parameter('turn_backup_max_s', 8.0)
        self.declare_parameter('turn_backup_speed_mps', 0.10)
        self.declare_parameter('turn_backup_rear_stop_m', 0.20)
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
        self._turn_tight_acc = 0.0
        self._turn_wide_run = 0.0
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
    def _lateral_clearance(self) -> float:
        """车身左右两侧（雷达 ±60°~±120°）的最小净空 [m]。

        用来判断"这里转得开吗"：车角扫掠半径 = |(0.30, 0.18)| = 0.35 m，
        所以侧向净空必须 > 0.35 m + 余量才允许大角度转弯。
        """
        if self._scan is None:
            return float('inf')
        lo = float('inf')
        for i, r in enumerate(self._scan.ranges):
            if not (math.isfinite(r) and r > 0.05):
                continue
            a = math.degrees(self._scan.angle_min + i * self._scan.angle_increment)
            if 60.0 <= abs(a) <= 120.0:
                lo = min(lo, r)
        return lo

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

    def _set_manual(self, enabled: bool) -> bool:
        """设置底盘手动通道。返回 False 表示服务不可用（调用方必须放弃让位）。

        2026-11-18 现场（"本该后退却前进了"）：这里原来静默 return，
        手动通道没打开，但让位照常"开始" —— 结果倒车指令被底盘丢弃、
        而**自动通道仍然畅通**，Nav2 继续按原速度往前开，直接顶到障碍上。
        """
        if not self._params.service_is_ready():
            return False
        req = SetParameters.Request()
        m = ParamMsg()
        m.name = 'manual_enabled'
        m.value.type = ParameterType.PARAMETER_BOOL
        m.value.bool_value = bool(enabled)
        req.parameters.append(m)
        self._params.call_async(req)
        return True

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

    # ---------- 统一让位：一条规则 ----------
    def _lateral_pair(self) -> tuple[float, float]:
        """左右侧向净空（雷达 ±60°~±120°），分开返回。

        与 _lateral_clearance() 用的是**同一个扇区**—— 判据必须同源，
        否则又会"一侧说转不开、另一侧说空间够了"。
        """
        if self._scan is None:
            return float('inf'), float('inf')
        left = right = float('inf')
        for i, r in enumerate(self._scan.ranges):
            if not (math.isfinite(r) and r > 0.05):
                continue
            a = math.degrees(self._scan.angle_min + i * self._scan.angle_increment)
            if 60.0 <= a <= 120.0:
                left = min(left, r)
            elif -120.0 <= a <= -60.0:
                right = min(right, r)
        return left, right

    def _front_min(self) -> float:
        """车头 ±30° 的最小净空（雷达原始距离）。"""
        if self._scan is None:
            return float('inf')
        return sector_min_range(self._scan, 30.0, 0.0)

    def _maneuver_params(self) -> dict:
        g = lambda n: float(self.get_parameter(n).value)   # noqa: E731
        return dict(turn_need_m=g('turn_need_m'), front_need_m=g('front_need_m'))

    def _maybe_start(self, now: float) -> None:
        """**执行前**判断：能转就不动，转不开/快撞就后退。"""
        if self._pose is None or self._scan is None:
            return
        dt = 0.0 if self._tick_at is None else max(0.0, now - self._tick_at)
        self._tick_at = now
        raw = self._raw_cmd
        filt = self._filtered_cmd
        wants_motion = bool(raw and abs(raw.linear.x) + abs(raw.angular.z) > 0.02)
        stopped = bool(filt and abs(filt.linear.x) + abs(filt.angular.z) < 0.01)
        if wants_motion and stopped:
            self._unblocked_run = 0.0
        else:
            self._unblocked_run += dt
        self._blocked_acc = mc.accumulate_blocked(
            self._blocked_acc, dt, wants_motion and stopped, self._unblocked_run)

        lat_l, lat_r = self._lateral_pair()
        front = self._front_min()
        turn_need = float(self.get_parameter('turn_need_m').value)
        # "想转但转不开"要持续一小会儿才动作，防单帧噪声
        # 2026-11-18 回归（"过门口就后退"）：原来只看"|ω|>=0.2 且两侧窄"，
        # 而门口天然两侧都窄、过门又有曲率 → 每过一个门就后退。
        # 现在要求"大角度转向 + 前方受限或确实卡住"（门口前方是开阔的）。
        tight = bool(raw) and mc.tight_turn_needed(
            raw_w_radps=raw.angular.z,
            lat_left_m=lat_l, lat_right_m=lat_r,
            front_m=front,
            blocked_for_s=self._blocked_acc,
            turn_need_m=turn_need,
            w_min_radps=float(self.get_parameter('turn_intent_radps').value),
            front_need_m=float(self.get_parameter('turn_front_need_m').value),
            blocked_s=float(self.get_parameter('blocked_s').value))
        self._turn_tight_acc = self._turn_tight_acc + dt if tight else 0.0

        ok, why = mc.wants_backup(
            want_turn=self._turn_tight_acc >= float(
                self.get_parameter('turn_guard_s').value),
            blocked_for_s=self._blocked_acc,
            front_m=front,
            lat_left_m=lat_l,
            lat_right_m=lat_r,
            rear_min_m=self._rear_min,
            rear_blocked=self._rear_blocked,
            attempts=self._turn_attempts,
            since_last_s=now - self._last_end,
            turn_need_m=turn_need,
            front_need_m=float(self.get_parameter('front_need_m').value),
            rear_need_m=float(self.get_parameter('rear_need_m').value),
            blocked_s=float(self.get_parameter('blocked_s').value),
            max_attempts=int(self.get_parameter('turn_max_attempts').value),
            cooldown_s=float(self.get_parameter('turn_cooldown_s').value))
        if not ok:
            # 连续顺畅行驶 3 s 就把让位次数复位，否则一次任务用完就不再让位
            if wants_motion and not stopped:
                self._unblocked_since = self._unblocked_since or now
                if now - self._unblocked_since > 3.0:
                    self._turn_attempts = 0
                    self._unblocked_since = None
            else:
                self._unblocked_since = None
            return
        self._unblocked_since = None
        self._begin_reverse(now, why, lat_l, lat_r, front)

    def _begin_reverse(self, now: float, why: str,
                       lat_l: float, lat_r: float, front: float) -> None:
        self._reversing = True
        self._tick_at = None
        self._blocked_acc = 0.0
        self._unblocked_run = 0.0
        self._turn_tight_acc = 0.0
        self._reversed_why = why
        self._armed_before = self._auto_armed
        self._started_at = now
        self._start_pose = self._pose
        self._reversed_m = 0.0
        if not self._set_manual(True):
            # 参数服务没就绪也能倒车：base_bridge 见到 /reverse/active 会自己
            # 放行手动通道。这里只记录，不放弃让位。
            self.get_logger().warn(
                '/base_bridge/set_parameters 未就绪 → 依赖 /reverse/active 放行手动通道',
                throttle_duration_sec=5.0)
        self._active_pub.publish(Bool(data=True))
        if why == 'turn-too-tight':
            raw = self._raw_cmd
            detail = '想转但侧向不够（左 %.2f / 右 %.2f，需 %.2f，前 %.2f）' % (
                lat_l, lat_r, float(self.get_parameter('turn_need_m').value),
                self._front_min())
            if raw is not None:
                detail += ' [Nav2 v=%.2f w=%.2f]' % (raw.linear.x, raw.angular.z)
        else:
            detail = '前方快撞（%.2f m，需 %.2f，已挡 %.1f s）' % (
                front, float(self.get_parameter('front_need_m').value),
                self._blocked_acc)
        self.get_logger().warn(
            '%s，车尾 %.2f m → 先后退，退到能转为止（第 %d 次）' % (
                detail, self._rear_min, self._turn_attempts + 1))

    def _step_reverse(self, now: float) -> None:
        x, y, yaw = self._pose
        self._reversed_m = math.hypot(x - self._start_pose[0],
                                      y - self._start_pose[1])
        lat_l, lat_r = self._lateral_pair()
        done, why = mc.backup_done(
            backed_m=self._reversed_m,
            elapsed_s=now - self._started_at,
            lat_left_m=lat_l,
            lat_right_m=lat_r,
            front_m=self._front_min(),
            rear_min_m=self._rear_min,
            rear_blocked=self._rear_blocked,
            turn_need_m=float(self.get_parameter('turn_need_m').value),
            front_need_m=float(self.get_parameter('front_need_m').value),
            dist_m=float(self.get_parameter('backup_max_m').value),
            max_s=float(self.get_parameter('backup_max_s').value),
            rear_stop_m=float(self.get_parameter('backup_rear_stop_m').value))
        if done:
            if why == 'can-turn-now':
                self._turn_attempts = 0        # 目的达成，下次重新计
            self.get_logger().info(
                f'后退判定[{why}] 左={lat_l:.2f} 右={lat_r:.2f} '
                f'前={self._front_min():.2f} 退={self._reversed_m:.2f}m')
            self._finish(why, min(lat_l, lat_r))
            return
        # 2026-11-18：/reverse/active 必须**持续**发布。原来只在 _begin_reverse
        # 发一次（latch），导航栈刚起来时 DDS 发现还没完成 → 这一条丢了 →
        # base_bridge 认为"不是让位"，把倒车指令全部静默丢弃（实测 12 s 退 0.00 m、
        # 日志里 [manual] rx 一条都没有）。
        self._active_pub.publish(Bool(data=True))
        tgt = self._trail.target_behind(
            float(self.get_parameter('target_back_m').value))
        cmd = Twist()
        cmd.linear.x = -abs(float(self.get_parameter('reverse_speed_mps').value))
        # 2026-11-18 现场（"左转了一下然后停住"）：后退时的转向没有限幅，
        # reverse_steering 的 max_w 是 0.5 rad/s，航向差 19° 就饱和 ——
        # v=-0.08 / w=-0.5 算出的轮速是 左 -14.5 / 右 -145.5 mm/s，
        # 等于**原地打转**，位移始终 0.00 m，最后 timeout 判"退了 0.00 m"。
        # 倒车要以"真的退出去"为第一目标，转向必须压小。
        max_w = float(self.get_parameter('reverse_max_w').value)
        cmd.angular.z = (0.0 if tgt is None else max(
            -max_w, min(max_w,
                        tc.reverse_steering(x, y, yaw, tgt[0], tgt[1]))))
        self._cmd.publish(cmd)
        self.get_logger().info(
            f'[reverse] 发布 v={cmd.linear.x:.3f} w={cmd.angular.z:.3f} '
            f'退={self._reversed_m:.2f}m 左={lat_l:.2f} 右={lat_r:.2f}',
            throttle_duration_sec=1.0)

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
                getattr(self, '_reversed_why', '让位'), why, self._reversed_m))


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
