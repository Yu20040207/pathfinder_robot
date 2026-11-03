"""Autonomous mapping supervisor: scene-aware kickstart + hand over to explore_lite.

slam_toolbox 只按平移距离加节点（原地转不出图），所以启动后要先“开出去”让地图长出
轮廓。踢一脚全程用激光判断：
  - 正前方有障碍 → 比较左右两侧空档，朝空的一侧转；
  - 转角不写死：持续转（0.7 rad/s）直到正前方 ≥1.2 m 畅通为止；
  - 转完直行，前方再次被挡就重复，完成 4 段直行后交给 explore_lite。
踢一脚期间发布 /explore/pause=true 挂起探索器，防止其导航目标锁死手动路径。
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class AutoMappingSupervisor(Node):
    def __init__(self):
        super().__init__('auto_mapping_supervisor')
        self.declare_parameter('spin_rad_s', 0.30)
        self.declare_parameter('kick_speed_mps', 0.15)
        self.declare_parameter('kick_turn_rad_s', 0.7)
        self.declare_parameter('startup_delay_s', 15.0)
        # 0.45：0.7 在普通房间里会被判成“一直堵”，导致踢一脚反复前进后退
        self.declare_parameter('front_block_m', 0.45)
        self.declare_parameter('front_clear_m', 1.2)
        self.declare_parameter('legs', 4)
        self.declare_parameter('leg_max_s', 8.0)
        # 2026-10-30：踢一脚最多跑这么久，超时就交给探索器（否则墙角死循环）
        self.declare_parameter('kick_timeout_s', 90.0)
        self.declare_parameter('kick_max_backups', 3)
        # 被包围时后退到"前方够转弯"为止的上限（秒）
        self.declare_parameter('kick_back_max_s', 8.0)
        # 2026-10-31：车尾暂时被挡（超声报警）时最多等这么久再退，
        # 而不是立刻放弃（原地不动等障碍物移开/等车尾转向空档）。
        self.declare_parameter('rear_wait_s', 3.0)
        self._cmd_manual = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self._state_pub = self.create_publisher(String, '/explore/state', 10)
        # 2026-10-31：踢一脚被包围时退到"前面够转弯"为止，车尾由超声守卫兜底
        self._rear_blocked = False
        self._rear_blocked_at = None
        self.create_subscription(Bool, '/base/rear_blocked',
                                 lambda m: setattr(self, '_rear_blocked', bool(m.data)), 10)
        self._pause_pub = self.create_publisher(Bool, '/explore/pause', 10)
        self._param_client = self.create_client(SetParameters, '/base_bridge/set_parameters')
        self._arm_client = self.create_client(Trigger, '/base/arm_auto')
        self._phase = 'waiting'
        self._phase_at = time.monotonic()
        self._leg_at = 0.0
        self._legs_done = 0
        self._kick_started_at = None
        self._backups = 0
        self._last_set_try = 0.0
        self._ready_ok = False
        self._manual_ok = False
        self.create_subscription(Bool, '/base/ready',
                                 lambda m: setattr(self, '_ready_ok', m.data), 10)
        # 2026-11-03：等 initial_spin 转完一圈再开始 kickstart，
        # 否则它 15 s 后自己开门转向，会和自转抢控制权。
        self.declare_parameter('wait_for_spin', True)
        self._spin_done = not bool(self.get_parameter('wait_for_spin').value)
        self.create_subscription(
            Bool, '/mapping/spin_done', self._on_spin_done,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Bool, '/base/manual_enabled',
                                 lambda m: setattr(self, '_manual_ok', m.data), 10)
        self._front_min = float('inf')
        self._left_min = float('inf')
        self._right_min = float('inf')
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self._on_scan, scan_qos)
        # 2026-11-03：首圈自转需要累计航向
        self._yaw = None
        self._last_yaw = None
        self._swept = 0.0
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self._done_pub = self.create_publisher(
            Bool, '/mapping/spin_done',
            QoSProfile(depth=1, durability=__import__('rclpy.qos', fromlist=['DurabilityPolicy']).DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.05, self._tick)

    # ---------- 激光分扇区 ----------
    # 雷达倒装 180°：scan 角度范围 [-180°,+180°]，index 0 = -180°（车头方向），
    # index mid = 0°（车尾方向）。因此：
    #   车头 = 数组两端各 40°；车左 = 角度 -90°(index n/4)；车右 = 角度 +90°(index 3n/4)。
    def _on_scan(self, msg):
        n = len(msg.ranges)
        deg = n / 360.0  # 每度对应索引数

        # 2026-10-30：雷达装在前部，车头前缘只比雷达远约 9 cm，
        # 因此 /scan 里 0.10~0.15 m 的读数大多是**车身/支架自己**。
        # 实测：前左 0.105、前右 0.102、左 0.134、右 0.150 m ——
        # 这些假读数让监督节点误判“被包围”，于是墙角反复前进后退。
        # 统一忽略 0.20 m 以内的读数（车身内切半径 0.215 m）。
        SELF_MIN = 0.20

        def _min(indices):
            finite = [msg.ranges[i] for i in indices
                      if 0 <= i < n and math.isfinite(msg.ranges[i])
                      and SELF_MIN < msg.ranges[i]]
            return min(finite) if finite else float('inf')

        def sector(center_deg, half_deg):
            a0 = int((center_deg - half_deg + 180.0) * deg)
            a1 = int((center_deg + half_deg + 180.0) * deg)
            return _min(range(max(0, a0), min(n, a1)))

        self._front_min = min(_min(range(0, int(40 * deg))),        # +140..+180
                              _min(range(n - int(40 * deg), n)))    # -180..-140
        self._left_min = sector(-90.0, 45.0)    # 车头左侧
        self._right_min = sector(90.0, 45.0)    # 车头右侧

    # ---------- 参数/授权 ----------
    def _on_spin_done(self, msg):
        if msg.data and not self._spin_done:
            self._spin_done = True
            self.get_logger().info('收到 spin_done → 开始 kickstart 计时')

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _set_bool(self, name, value):
        if not self._param_client.service_is_ready():
            return
        request = SetParameters.Request()
        parameter = Parameter()
        parameter.name = name
        parameter.value.type = ParameterType.PARAMETER_BOOL
        parameter.value.bool_value = value
        request.parameters.append(parameter)
        future = self._param_client.call_async(request)

        def done(f):
            try:
                result = f.result()
                if not all(r.successful for r in result.results):
                    self.get_logger().warn(
                        f'set {name}={value} failed: {result.results[0].reason}')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'set {name}={value} error: {exc}')

        future.add_done_callback(done)

    # ---------- 主状态机 ----------
    def _tick(self):
        now = time.monotonic()
        state = 'navigating'
        if self._phase == 'arming':
            # 门控的手动锁存在“stopped→selecting”序列后才清除，
            # 否则踢一脚用过手动后自动链路永远被挡。
            elapsed = now - self._leg_at
            if elapsed < 0.3:
                state = 'stopped'
            elif elapsed < 0.6:
                state = 'selecting'
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)
        if self._phase.startswith('kick_') or self._phase == 'waiting':
            pause = Bool()
            pause.data = True
            self._pause_pub.publish(pause)
        else:
            pause = Bool()
            pause.data = False
            self._pause_pub.publish(pause)

        if self._phase == 'waiting':
            if now - self._phase_at >= float(self.get_parameter('startup_delay_s').value):
                if now - self._last_set_try > 2.0:
                    self._last_set_try = now
                    self.get_logger().info('kickstart: opening gates')
                    self._set_bool('motion_enabled', True)
                    self._set_bool('manual_enabled', True)
                if self._ready_ok and self._manual_ok:
                    self.get_logger().info('门已确认打开 → 开始原地自转一圈')
                    self._phase = 'spin'
                    self._phase_at = now
                    self._swept = 0.0
                    self._last_yaw = None
            return

        # ---------- 2026-11-03：首圈自转（用户要求）----------
        if self._phase == 'spin':
            if self._yaw is None:
                return
            if self._last_yaw is not None:
                d = math.atan2(math.sin(self._yaw - self._last_yaw),
                               math.cos(self._yaw - self._last_yaw))
                self._swept += abs(d)
            self._last_yaw = self._yaw
            if self._swept >= 2 * math.pi:
                self._cmd_manual.publish(Twist())
                self.get_logger().info(
                    f'自转完成（累计 {math.degrees(self._swept):.0f}°）→ 停 2 秒')
                self._phase = 'spin_settle'
                self._phase_at = now
                return
            t = Twist()
            t.angular.z = float(self.get_parameter('spin_rad_s').value)
            self._cmd_manual.publish(t)
            return

        if self._phase == 'spin_settle':
            self._cmd_manual.publish(Twist())
            if now - self._phase_at >= 2.0:
                msg = Bool()
                msg.data = True
                self._done_pub.publish(msg)          # 放开 room_explorer
                self.get_logger().info('自转+停顿结束 → 交给探索器')
                self._phase = 'handover'
                self._phase_at = now
            return

        if self._phase == 'handover':
            return                                    # 不再动车，交给探索器
        if self._phase == 'kick_return':
            # 直行段结束后倒车退回已建区域：探索器需要机器人在地图内部才能规划
            if now - self._leg_at < 6.0:
                twist = Twist()
                twist.linear.x = -0.15
                self._cmd_manual.publish(twist)
                return
            self._cmd_manual.publish(Twist())
            self.get_logger().info('kickstart: handing over to explore_lite')
            self._set_bool('manual_enabled', False)
            self._phase = 'arming'
            self._leg_at = now
            return
        if self._phase.startswith('kick') and self._kick_started_at is None:
            self._kick_started_at = now
        if (self._phase.startswith('kick') and self._kick_started_at is not None and
                now - self._kick_started_at >
                float(self.get_parameter('kick_timeout_s').value)):
            self.get_logger().warn('kickstart: 超时（可能卡在角落），直接交给探索器')
            self._cmd_manual.publish(Twist())
            self._phase = 'kick_return'
            self._leg_at = now
            self._kick_started_at = None
            return
        if self._phase == 'kick_back':
            # 2026-10-31：被包围时改为**自适应后退** —— 退到前方够转弯为止：
            #   · 前方 >= front_clear_m（够掉头/转弯）→ 停
            #   · 车尾超声守卫报警 → 停（不能再退）
            #   · 退满 kick_back_max_s → 停（安全上限）
            clear_m = float(self.get_parameter('front_clear_m').value)
            front_ok = self._front_min >= clear_m
            if not front_ok:
                if self._rear_blocked:
                    # 车尾暂时被挡：原地等着，最多 rear_wait_s 秒再去退
                    # （挡太久说明真的没退路，本次放弃）
                    if self._rear_blocked_at is None:
                        self._rear_blocked_at = now
                        self.get_logger().info('kickstart: 车尾暂被挡，等待让位')
                    if now - self._rear_blocked_at < float(
                            self.get_parameter('rear_wait_s').value):
                        self._cmd_manual.publish(Twist())
                        return
                    self.get_logger().warn('kickstart: 车尾持续被挡，放弃后退')
                else:
                    self._rear_blocked_at = None
                    if now - self._leg_at < float(
                            self.get_parameter('kick_back_max_s').value):
                        twist = Twist()
                        twist.linear.x = -0.10
                        self._cmd_manual.publish(twist)
                        return
            self._cmd_manual.publish(Twist())
            self._backups += 1
            self.get_logger().info(
                f'kickstart: backed out ({self._backups}/'
                f'{int(self.get_parameter("kick_max_backups").value)}), re-orienting')
            if self._backups > int(self.get_parameter('kick_max_backups').value):
                self.get_logger().warn('kickstart: 倒车次数过多，直接交给探索器')
                self._phase = 'kick_return'
                self._leg_at = now
                self._kick_started_at = None
                return
            self._phase = 'kick_orient'
            self._leg_at = now
            return
        if self._phase == 'kick_orient':
            # 看场景选方向：左空档大就左转，否则右转；转到正前方畅通为止
            block = float(self.get_parameter('front_block_m').value)
            if (self._front_min < block and
                    self._left_min < block and self._right_min < block):
                # 三面都堵，原地转不出去（车头会扫墙），先倒车
                self._cmd_manual.publish(Twist())
                self.get_logger().info(
                    f'kickstart: boxed (f {self._front_min:.2f} l {self._left_min:.2f} '
                    f'r {self._right_min:.2f}), backing up')
                self._phase = 'kick_back'
                self._leg_at = now
                return
            if self._front_min >= float(self.get_parameter('front_clear_m').value):
                self._cmd_manual.publish(Twist())
                self.get_logger().info('kickstart: front clear, driving')
                self._phase = 'kick_drive'
                self._leg_at = now
                return
            if now - self._leg_at > 6.0:
                # 转了 6 秒还不通也去直行（可能被包围，直行触发脱困转向）
                self._cmd_manual.publish(Twist())
                self.get_logger().info('kickstart: orient timeout, trying to drive')
                self._phase = 'kick_drive'
                self._leg_at = now
                return
            direction = -1.0 if self._left_min >= self._right_min else 1.0
            side = 'left' if direction < 0 else 'right'
            self.get_logger().info(
                f'kickstart: front {self._front_min:.2f}m blocked, '
                f'left {self._left_min:.2f}m right {self._right_min:.2f}m -> turning {side}',
                throttle_duration_sec=2.0)
            twist = Twist()
            twist.angular.z = direction * float(self.get_parameter('kick_turn_rad_s').value)
            self._cmd_manual.publish(twist)
            return
        if self._phase == 'kick_drive':
            block = float(self.get_parameter('front_block_m').value)
            leg_max = float(self.get_parameter('leg_max_s').value)
            if self._front_min < block:
                self._cmd_manual.publish(Twist())
                self.get_logger().info(f'kickstart: blocked at {self._front_min:.2f}m, orienting')
                self._phase = 'kick_orient'
                self._leg_at = now
                return
            if now - self._leg_at >= leg_max:
                self._cmd_manual.publish(Twist())
                self._legs_done += 1
                self.get_logger().info(f'kickstart: leg {self._legs_done}/'
                                       f'{int(self.get_parameter("legs").value)} done')
                if self._legs_done >= int(self.get_parameter('legs').value):
                    self._cmd_manual.publish(Twist())
                    self.get_logger().info('kickstart: returning into mapped area')
                    self._phase = 'kick_return'
                    self._leg_at = now
                    return
                self._phase = 'kick_orient'
                self._leg_at = now
                return
            twist = Twist()
            twist.linear.x = float(self.get_parameter('kick_speed_mps').value)
            self._cmd_manual.publish(twist)
            return
        if self._phase == 'arming':
            if now - self._leg_at < 1.5:
                return
            if self._arm_client.service_is_ready():
                self.get_logger().info('arming auto chain')
                self._arm_client.call_async(Trigger.Request())
                self._phase = 'armed'
                self._leg_at = now
            return
        if self._phase == 'armed':
            if now - self._leg_at > 5.0:
                self._leg_at = now
                if self._arm_client.service_is_ready():
                    self._arm_client.call_async(Trigger.Request())


def main():
    rclpy.init()
    node = AutoMappingSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
