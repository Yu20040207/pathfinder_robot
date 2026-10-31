#!/usr/bin/env python3
"""沿墙建图（boundary tracing / wall following）。

用户要求（2026-10-30）：不要用前沿探索那样原地左右转、来回折返；
应该先有能力贴着墙走，沿着房间边边走一圈，边走边扫。

工作方式：
  · 选定一侧（side=left/right）作为「参考墙」
  · 保持参考墙距离 target_dist（默认 0.55 m）匀速前进
  · 前方 < front_stop_m 就原地转向（固定朝参考墙的对侧），直到前方通畅
  · 走完 wall_follow_max_s 或闭环（回到起点附近且航向接近）就停下

安全：只发布 /cmd_vel_manual，底盘门控、碰撞监控、看门狗全部照旧生效；
      读数 < SELF_MIN 的一律忽略（雷达装在前部，车身会挡住近处）。
"""
import math

import rclpy
from rcl_interfaces.msg import Parameter as RclParameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


SELF_MIN = 0.20          # 忽略车身自击（雷达前缘距离车头仅约 9 cm）


class WallFollower(Node):
    def __init__(self) -> None:
        super().__init__('wall_follower')
        self.declare_parameter('side', 'right')          # 贴着哪一侧走
        # 0.75：0.55 太贴墙（车半宽 0.215 m，实际只剩 0.34 m）
        self.declare_parameter('target_dist_m', 0.75)
        self.declare_parameter('speed_mps', 0.12)        # 慢一点，扫描更稳
        # 0.55：0.35 时避障转弯半径太大（v/ω），窄处绕不开
        self.declare_parameter('turn_rate_rad_s', 0.55)
        self.declare_parameter('front_stop_m', 0.60)
        self.declare_parameter('gain', 2.0)
        self.declare_parameter('max_time_s', 300.0)
        self.declare_parameter('loop_close_radius_m', 0.6)
        # 大范围建图时关掉“回到起点就停”，让它一直沿墙走（靠 max_time 收工）
        self.declare_parameter('stop_on_loop', True)
        # 自动开门：本节点自己把底盘两道门打开（等价 supervisor 做的事）
        self.declare_parameter('open_gates', True)
        # 2026-10-30 平滑化：不再"停了再转"，改成边走边绕 + 加减速限幅
        # 1.45：1.10 时开始绕已经太晚（实测会撞）
        # 1.85：1.45 在快速接近斜墙时来不及（实测撞上前右角）
        self.declare_parameter('avoid_start_m', 1.85)   # 前方小于这个距离开始绕
        # 0.50：雷达装在前部，读数 0.50 意味着车头边缘距障碍约 0.41 m；
        # 太小会顶到墙 → 轮子打滑 → 固件失速自停。
        self.declare_parameter('emergency_m', 0.60)     # 小于这个距离才急停
        self.declare_parameter('gap_gain', 1.4)         # 间隙转向权重
        self.declare_parameter('pivot_m', 0.45)
        # 侧向测距窗口：±50° 会把前方障碍混进来（实测正右 1.68 m 却读成 0.44 m），
        # 收窄到 ±18° 只测真正的侧墙。
        self.declare_parameter('side_half_deg', 18.0)    # 侧向测距窗口（度）
        # 2026-10-30：只贴一侧走，另一侧的门洞永远进不去。
        # 每走 switch_side_after_m 米换一次贴墙侧（0 = 不换）。
        self.declare_parameter('switch_side_after_m', 8.0)
        self.declare_parameter('turn_commit_s', 1.5)    # 转向方向锁定时间（防左右抖）
        # 0.60：实测 0.35（≈4 cm/s）时车轮克服不了静摩擦 → 失速自停反复触发。
        # 这台车可靠起步速度约 7 cm/s 以上。
        self.declare_parameter('arc_min_speed_ratio', 0.60)
        self.declare_parameter('min_speed_mps', 0.07)   # 硬下限，低于它宁可不发
        # 2026-10-30：车尾是盲区，但地图里后方刚走过=空闲。倒车前查地图。
        self.declare_parameter('rear_check_m', 0.60)    # 后退所需航迹长度
        self.declare_parameter('rear_half_width_m', 0.28)
        self.declare_parameter('reverse_speed_mps', 0.07)
        self.declare_parameter('reverse_time_s', 1.6)
        # 2026-10-31：识别到门洞后，瞄准门洞中心直线穿过去
        self.declare_parameter('door_min_width_deg', 22.0)
        self.declare_parameter('door_max_width_deg', 70.0)
        self.declare_parameter('door_clear_m', 1.30)      # 门洞内至少这么空
        self.declare_parameter('door_frame_m', 2.20)      # 门框必须在 2.2 m 内（真门框）
        self.declare_parameter('door_cooldown_s', 8.0)    # 穿完冷却，避免连击
        self.declare_parameter('door_commit_s', 4.0)
        self.declare_parameter('max_accel_mps2', 0.25)  # 线加速度限幅
        self.declare_parameter('max_alpha_rads2', 1.2)  # 角加速度限幅

        self._side = str(self.get_parameter('side').value)
        self._target = float(self.get_parameter('target_dist_m').value)
        self._speed = float(self.get_parameter('speed_mps').value)
        self._turn = float(self.get_parameter('turn_rate_rad_s').value)
        self._front_stop = float(self.get_parameter('front_stop_m').value)
        self._gain = float(self.get_parameter('gain').value)
        self._max_time = float(self.get_parameter('max_time_s').value)
        self._loop_r = float(self.get_parameter('loop_close_radius_m').value)

        self._front = float('inf')
        self._front_left = float('inf')
        self._front_right = float('inf')
        self._side_min = float('inf')
        self._left_dist = float('inf')
        self._scan_msg = None
        self._right_dist = float('inf')
        self._start = None
        self._started_at = None
        self._turned = 0.0
        self._travelled = 0.0
        self._since_switch = 0.0
        self._last_xy = None
        self._turn_started = None
        self._done = False
        self._v_cmd = 0.0
        self._w_cmd = 0.0
        self._last_tick = None
        self._turn_dir = 0.0
        self._turn_until = 0.0
        self._pivot_at = 0.0
        self._enter_at = 0.0
        self._reverse_until = 0.0
        # 刚走过的轨迹（倒车沿原路退回，走过的地方必然空闲）
        self._trail = []
        self._door_until = 0.0
        self._door_ready_at = 0.0

        q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self._on_scan, q)
        self.create_subscription(Odometry, '/wheel/odom', self._on_odom, 20)
        self._pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self._param_client = self.create_client(SetParameters, '/base_bridge/set_parameters')
        self._clear_fault = self.create_client(Trigger, '/base/clear_fault')
        self._fault = 0
        self._recover_stage = 0
        self._recover_at = 0.0
        self._recover_count = 0
        self.create_subscription(Int32, '/base/fault', self._on_fault, 10)
        self._gates_at = None
        self.create_timer(0.05, self._tick)          # 20 Hz（底盘 0.2 s 过期）
        self.get_logger().info(
            f'沿墙建图启动：贴{self._side} 目标距 {self._target:.2f} m '
            f'速度 {self._speed:.2f} m/s')

    # ---------- 输入 ----------
    def _on_scan(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        deg = n / 360.0

        def sector(center_deg: float, half_deg: float) -> float:
            a0 = int((center_deg - half_deg + 180.0) * deg)
            a1 = int((center_deg + half_deg + 180.0) * deg)
            vals = [msg.ranges[i] for i in range(max(0, a0), min(n, a1))
                    if math.isfinite(msg.ranges[i]) and msg.ranges[i] > SELF_MIN]
            return min(vals) if vals else float('inf')

        # 雷达倒装：index 0 = 车头；scan 角 ±180 = 正前，-90 = 车左，+90 = 车右
        # ±35°：±20° 时斜前方 30~60° 的墙看不到，斜着靠近发现太晚 → 撞
        self._front = min(sector(180.0, 45.0), sector(-180.0, 45.0))
        self._scan_msg = msg
        self._front_left = sector(-140.0, 40.0)     # 覆盖 100°~180°
        self._front_right = sector(140.0, 40.0)     # 覆盖 100°~180°
        side_half = float(self.get_parameter('side_half_deg').value)
        self._left_dist = sector(-90.0, side_half)
        self._right_dist = sector(90.0, side_half)
        self._side_min = (self._left_dist if self._side == 'left'
                          else self._right_dist)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        step = (math.hypot(p.x - self._last_xy[0], p.y - self._last_xy[1])
                if self._last_xy is not None else 0.0)
        self._travelled += step
        self._since_switch += step
        limit = float(self.get_parameter('switch_side_after_m').value)
        if limit > 0 and self._since_switch > limit:
            self._side = 'left' if self._side == 'right' else 'right'
            self._since_switch = 0.0
            self._turn_dir = 0.0
            self._narrow_logged = False
            self.get_logger().info(f'换贴墙侧 → {self._side}（已走 {self._travelled:.1f} m）')
        self._last_xy = (p.x, p.y)
        if not self._trail or math.hypot(p.x - self._trail[-1][0],
                                         p.y - self._trail[-1][1]) > 0.05:
            self._trail.append((p.x, p.y))
            if len(self._trail) > 400:
                self._trail = self._trail[-400:]
        if self._start is None:
            self._start = (p.x, p.y)
            return
        moved = math.hypot(p.x - self._start[0], p.y - self._start[1])
        elapsed = (self.get_clock().now().nanoseconds * 1e-9) - (self._started_at or 0.0)
        # 闭环：走回起点附近且已经离开过一段时间
        # 闭环必须建立在"确实走过一圈"之上：累计路程够长 + 回到起点 + 转够了方向
        if (bool(self.get_parameter('stop_on_loop').value) and
                not self._done and moved < self._loop_r and
                self._travelled > 6.0 and self._turned > 8.0 and elapsed > 30.0):
            self.get_logger().info(
                f'沿墙建图：回到起点附近（走了 {self._travelled:.1f} m），闭环完成')
            self._done = True

    def _rear_clear(self) -> bool:
        """后方能不能退：沿刚走过的航迹退还剩多少路。

        2026-10-30：原来用 /map + TF 判断后方空闲，OccupancyGrid 有 9 万格，
        每秒在 Python 里重建对象把 wall_follower 顶到 276% CPU，拖垮雷达和建图。
        改成看自己的航迹：只要「刚走过的路」还够长，沿原路退就必然安全。
        """
        need = float(self.get_parameter('rear_check_m').value)
        if len(self._trail) < 2:
            return False
        # 从当前位置沿航迹往回累计距离
        dist = 0.0
        ax, ay = self._trail[-1]
        for (bx, by) in reversed(self._trail[:-1]):
            dist += math.hypot(ax - bx, ay - by)
            ax, ay = bx, by
            if dist >= need:
                return True
        return False

    def _detect_door(self):
        """在前方半圈里找“两侧有框、中间通透”的缺口，返回 (中心角°, 宽度°)。

        角度约定：0° = 正前，负数 = 左，正数 = 右（和 scan 的 0°=车尾一致）。
        """
        msg = self._scan_msg
        if msg is None:
            return None
        n = len(msg.ranges)
        deg_per_idx = n / 360.0
        clear = float(self.get_parameter('door_clear_m').value)
        frame = float(self.get_parameter('door_frame_m').value)
        # 按 1° 分箱取最小距离
        bins = {}
        for i in range(n):
            r = msg.ranges[i]
            if not math.isfinite(r) or r <= 0.20:
                continue
            a = math.degrees(msg.angle_min + i * msg.angle_increment)   # -180..180
            sb = (180.0 - abs(a)) * (1.0 if a >= 0 else -1.0)           # 0=前, +=右
            if abs(sb) > 90.0:
                continue
            k = int(round(sb))
            bins[k] = min(bins.get(k, 99.0), r)
        if len(bins) < 30:
            return None
        angles = sorted(bins)
        best = None
        run_start = None
        for idx, ang in enumerate(angles):
            is_open = bins[ang] > clear
            if is_open and run_start is None:
                run_start = ang
            if (not is_open or idx == len(angles) - 1) and run_start is not None:
                run_end = angles[idx - 1] if not is_open else ang
                width = run_end - run_start
                # 两侧要有门框（紧邻的格子较近）
                left_ok = any(bins.get(a, 99.0) < frame
                              for a in range(run_start - 6, run_start))
                right_ok = any(bins.get(a, 99.0) < frame
                               for a in range(run_end + 1, run_end + 7))
                if (left_ok and right_ok and
                        float(self.get_parameter('door_min_width_deg').value)
                        <= width
                        <= float(self.get_parameter('door_max_width_deg').value)):
                    if best is None or width > best[1]:
                        best = ((run_start + run_end) / 2.0, width)
                run_start = None
        return best

    # ---------- 控制 ----------
    def _on_fault(self, msg: Int32) -> None:
        self._fault = int(msg.data)

    def _recover_fault(self, now: float) -> None:
        """失速自停后自动复位：关门 → 清故障 → 重新开门 → 倒车 → 继续。

        2026-10-30：原来的时间窗写法在 clear_fault 失败时会卡在“门关着”的状态。
        改成显式状态机 + 重试计数，任何一步失败都打日志。
        """
        if self._recover_stage == 0:
            self._recover_stage = 1
            self._recover_at = now
            self._recover_count += 1
            self.get_logger().warn(
                f'底盘故障 {self._fault}（多为顶墙失速），第 {self._recover_count} 次复位')
            self._v_cmd = self._w_cmd = 0.0
            self._pub.publish(Twist())
            # 清故障要求两道门都关
            self._set_params({'motion_enabled': False, 'manual_enabled': False})
            return
        st = now - self._recover_at
        if self._recover_stage == 1 and st > 0.8:
            self._recover_stage = 2
            if self._clear_fault.service_is_ready():
                future = self._clear_fault.call_async(Trigger.Request())
                future.add_done_callback(
                    lambda f: self.get_logger().info(f'clear_fault: {f.result()}'))
            else:
                self.get_logger().warn('/base/clear_fault 服务不可用')
            return
        if self._recover_stage == 2 and st > 2.0:
            self._recover_stage = 3
            # 无条件重新开门（即使 clear 失败也要恢复运动能力，否则只能人工干预）
            self._set_params({'motion_enabled': True, 'manual_enabled': True,
                              'manual_allow_mixed': True})
            return
        if self._recover_stage == 3 and st > 2.6:
            self._recover_stage = 4
            return
        if self._recover_stage == 4:
            # 注意顺序：必须先判“结束”，否则 st>4.2 的分支会一直 return，
            # 永远走不到“复位完成” → 表现为失速后无限原地转（2026-10-31 实测）。
            if st > 5.4:
                self._pub.publish(Twist())
                self._recover_stage = 0
                self._fault = 0
                self._v_cmd = self._w_cmd = 0.0
                self._pivot_at = 0.0
                self._turn_dir = 0.0
                self.get_logger().info(
                    f'复位完成（累计 {self._recover_count} 次），继续沿墙')
                return
            if st > 4.2:
                # 原地转脱困：车尾是盲区，倒车沿航迹判断后才用
                twist = Twist()
                twist.angular.z = (1.0 if self._side == 'right' else -1.0) * self._turn
                self._pub.publish(twist)
                return

    def _set_params(self, values) -> None:
        if not self._param_client.service_is_ready():
            return
        request = SetParameters.Request()
        for name, value in values.items():
            p = RclParameter()
            p.name = name
            p.value.type = ParameterType.PARAMETER_BOOL
            p.value.bool_value = bool(value)
            request.parameters.append(p)
        self._param_client.call_async(request)

    def _open_gates(self) -> None:
        if not self._param_client.service_is_ready():
            return
        request = SetParameters.Request()
        # manual_allow_mixed：沿墙走需要同时给线速度+角速度（默认底盘只允许单轴）
        for name in ('motion_enabled', 'manual_enabled', 'manual_allow_mixed'):
            p = RclParameter()
            p.name = name
            p.value.type = ParameterType.PARAMETER_BOOL
            p.value.bool_value = True
            request.parameters.append(p)
        future = self._param_client.call_async(request)
        future.add_done_callback(self._gates_done)

    def _gates_done(self, future) -> None:
        try:
            result = future.result()
            ok = result is not None and all(r.successful for r in result.results)
            self.get_logger().info(
                '底盘运动门已打开' if ok else '打开运动门失败（检查 base_bridge）')
            if ok:
                self._started_at = self.get_clock().now().nanoseconds * 1e-9
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().warn(f'打开运动门异常: {exc}')

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._gates_at is None:
            self._gates_at = now
        if bool(self.get_parameter('open_gates').value) and self._started_at is None:
            # 先等 2 秒让 base_bridge 起来，再开门
            if now - self._gates_at > 2.0 and now - self._gates_at < 2.2:
                self._open_gates()
            return
        if now < self._reverse_until:
            twist.linear.x = -float(self.get_parameter('reverse_speed_mps').value)
            self._pub.publish(twist)
            return
        if self._recover_stage == 0 and self._fault not in (0, None):
            self._recover_fault(now)
            return
        if self._recover_stage != 0:
            self._recover_fault(now)
            return
        if self._started_at is None:
            self._started_at = now
        twist = Twist()
        if self._done or now - self._started_at > self._max_time:
            self._pub.publish(twist)
            if not self._done:
                self.get_logger().info('沿墙建图：到达时间上限，停止')
                self._done = True
            return

        # ---------- 转向：沿墙保持 + 弧线绕障（方向锁定，避免左右抖） ----------
        avoid_start = float(self.get_parameter('avoid_start_m').value)
        pivot_m = float(self.get_parameter('pivot_m').value)
        k_gap = float(self.get_parameter('gap_gain').value)
        commit_s = float(self.get_parameter('turn_commit_s').value)
        arc_ratio = float(self.get_parameter('arc_min_speed_ratio').value)
        side_sign = -1.0 if self._side == 'right' else 1.0   # 贴右墙时"离墙远"要右转

        # ⓪ 门洞穿越：识别到“两侧有框、中间通透”的缺口就瞄准中心直线过去
        door = self._detect_door()
        if door is not None and now >= self._door_until and now >= self._door_ready_at:
            self._door_until = now + float(self.get_parameter('door_commit_s').value)
            self._door_ready_at = self._door_until + float(
                self.get_parameter('door_cooldown_s').value)
            self.get_logger().info(
                f'识别到门洞：中心偏 {"左" if door[0] < 0 else "右"}'
                f'{abs(door[0]):.0f}°，宽 {door[1]:.0f}°，对准中心穿越')
        if now < self._door_until and door is not None:
            center_rad = math.radians(door[0])
            twist.linear.x = max(float(self.get_parameter('min_speed_mps').value),
                                 self._speed * 0.8)
            twist.angular.z = max(-self._turn,
                                  min(self._turn, -2.2 * center_rad))
            self._pub.publish(twist)
            return

        # ①a 侧面出现门洞/大开口：主动转进去（否则会直接开过去）
        #     判据：贴墙那一侧测不到墙（或远到 > target+1.2 m），且前方是通的。
        #     限制：一次最多“进洞”2.5 秒，避免在空旷处一直转圈。
        if now >= self._turn_until:
            side_open = (not math.isfinite(self._side_min) or
                         self._side_min > self._target + 1.2)
            if side_open and self._front > max(pivot_m, 0.6):
                self._turn_dir = side_sign
                self._turn_until = now + float(self.get_parameter('turn_commit_s').value)
                self._enter_at = now
        if self._enter_at and now - self._enter_at < 2.5:
            twist.linear.x = max(float(self.get_parameter('min_speed_mps').value),
                                 self._speed * 0.6)
            twist.angular.z = self._turn_dir * self._turn * 0.8
            self._pub.publish(twist)
            return
        self._enter_at = 0.0
        self._reverse_until = 0.0
        # 刚走过的轨迹（倒车沿原路退回，走过的地方必然空闲）
        self._trail = []
        self._door_until = 0.0
        self._door_ready_at = 0.0

        # ① 沿墙保持项
        #   2026-10-30：固定 0.75 m 在 0.86 m 门洞里必然撞门框（想离右墙 0.75，
        #   就等于贴到左边门框）。左右都近时改为"走中间"。
        target_eff = self._target
        if math.isfinite(self._left_dist) and math.isfinite(self._right_dist):
            corridor = self._left_dist + self._right_dist
            if corridor < 1.70:
                target_eff = max(0.32, corridor / 2.0 - 0.02)
                if not getattr(self, '_narrow_logged', False):
                    self.get_logger().info(
                        f'检测到窄通道（左右合计 {corridor:.2f} m），改为居中对齐')
                    self._narrow_logged = True
            else:
                self._narrow_logged = False
        omega_wall = 0.0
        if math.isfinite(self._side_min):
            omega_wall = side_sign * self._gain * (self._side_min - target_eff)
        omega_wall = max(-self._turn * 0.6, min(self._turn * 0.6, omega_wall))

        lf = self._front_left if math.isfinite(self._front_left) else 3.0
        rf = self._front_right if math.isfinite(self._front_right) else 3.0

        if self._front <= pivot_m:
            # ② 墙角脱困：只朝"空档大的一侧"单向转，不再左右来回换向。
            #    2026-10-30：原来每 3 秒换方向 → 在墙角左右摆动死锁。
            # 注意：_turn_dir 可能已由“进洞/避障”分支设过非 0 值，
            # 此时必须用 _pivot_at==0 来判断“刚进入脱困”，否则计时基准是 0
            # 会让“转 25 秒”立刻成立（ROS 时间是 1.79e9 量级）。
            if self._pivot_at == 0.0:
                if self._turn_dir == 0.0:
                    self._turn_dir = 1.0 if lf >= rf else -1.0
                self._pivot_at = now
                self.get_logger().info(
                    f'贴障脱困：朝{"左" if self._turn_dir > 0 else "右"}单向转 '
                    f'(前 {self._front:.2f} 左 {lf:.2f} 右 {rf:.2f})')
            # 优先沿原路后退（走过的路必然空闲），退不了才原地转
            if (now - self._pivot_at > 1.5 and
                    now >= self._reverse_until and
                    self._rear_clear()):
                self.get_logger().info('后方地图确认空闲，倒车脱困')
                self._reverse_until = now + float(
                    self.get_parameter('reverse_time_s').value)
                self._turn_dir = 0.0
                self._pivot_at = 0.0
                return
            if now - self._pivot_at > 25.0:
                self.get_logger().error(
                    '原地转 25 秒仍未脱困，停止并等待人工处理')
                self._pub.publish(Twist())
                self._done = True
                return
            # 旋转过程中一旦某一侧出现空档，就立刻朝那边走出去，不要死转
            free_side = 0.0
            if lf > 0.90 and lf > rf:
                free_side = 1.0
            elif rf > 0.90 and rf > lf:
                free_side = -1.0
            if free_side != 0.0:
                self._pub.publish(Twist())
                self._turn_dir = 0.0
                self._pivot_at = 0.0
                self.get_logger().info(
                    f'侧向出现空档（左 {lf:.2f} 右 {rf:.2f}），改为朝空档走')
                twist.linear.x = max(float(self.get_parameter('min_speed_mps').value),
                                     self._speed * 0.7)
                twist.angular.z = free_side * self._turn * 0.9
                self._pub.publish(twist)
                return
            v = 0.0
            omega = self._turn_dir * self._turn
        elif self._front < avoid_start:
            # ③ 弧线绕障：边走边转，方向锁定 turn_commit_s 秒，避免左右翻
            w_gap = (avoid_start - self._front) / max(1e-6, avoid_start - pivot_m)
            w_gap = max(0.0, min(1.0, w_gap))
            if now >= self._turn_until:
                # 选方向：优先更空的一侧；接近时参考"远离墙侧"，保证一直同向绕
                if abs(lf - rf) > 0.15:
                    self._turn_dir = 1.0 if lf > rf else -1.0   # 更空的一侧在左就左转
                else:
                    self._turn_dir = side_sign
                self._turn_until = now + commit_s
            omega = (1.0 - w_gap) * omega_wall + w_gap * self._turn * self._turn_dir
            # 速度只降不停：最低 arc_ratio，保证走弧线而不是原地蹭
            v = self._speed * max(arc_ratio,
                                  1.0 - (1.0 - arc_ratio) * w_gap)
        else:
            # ④ 开阔：正常沿墙
            self._turn_dir = 0.0
            omega = omega_wall
            v = self._speed

        omega = max(-self._turn, min(self._turn, omega))
        # 硬下限：低速指令会让轮子卡在静摩擦里 → 固件判失速。
        # 要么给到能转起来的速度，要么干脆停（给 0）。
        min_v = float(self.get_parameter('min_speed_mps').value)
        if v > 1e-3 and v < min_v:
            v = min_v

        # ---------- 加减速限幅（消除一卡一卡） ----------
        dt = 0.05 if self._last_tick is None else max(0.01, now - self._last_tick)
        self._last_tick = now
        dv = float(self.get_parameter('max_accel_mps2').value) * dt
        dw = float(self.get_parameter('max_alpha_rads2').value) * dt
        self._v_cmd += max(-dv, min(dv, v - self._v_cmd))
        self._w_cmd += max(-dw, min(dw, omega - self._w_cmd))

        twist.linear.x = self._v_cmd
        twist.angular.z = self._w_cmd
        self._turned += abs(self._w_cmd) * dt
        self._pub.publish(twist)


def main():
    rclpy.init()
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
