"""Sole ESP32-S3 serial owner for the physical chassis."""

from collections import deque
import math
import os
import time

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Bool, Float64MultiArray, Int32, String
from std_srvs.srv import Trigger
from fwr_interfaces.action import TurnRelative
from fwr_interfaces.srv import RecoveryPermit

from .base_bridge_core import (BaseBridgeCore, RecoveryPermitCore, ScanWatchdog, SensorStampGuard,
                               scale_velocity_to_wheel_limit, manual_velocity,
                               command_source_allowed, safe_command_gap_action,
                               fault_clear_allowed, nav_reverse_ok)
from .base_protocol import MAX_LINE, parse_state_line
from .base_serial import open_exclusive_serial
from .wheel_odometry_node import DEFAULT_DEVICE
from .turn_controller import TurnController, TurnFinished
from .heading_hold import HeadingHold
from .imu_yaw_rate_control import ImuYawRateController

_DEBUG_REJECT = bool(os.environ.get('FWR_BASE_DEBUG'))
_TX_LOG = bool(os.environ.get('FWR_TX_LOG'))


class BaseBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('base_bridge')
        self.declare_parameter('device', DEFAULT_DEVICE)
        self.declare_parameter('motion_enabled', False)
        self.declare_parameter('manual_enabled', False)
        # 手动指令的保持窗口：RViz 渲染偶发卡顿 >200 ms 会让固件收到 'x' 断一下，
        # 现场就是"一卡一卡"。0.25 s 既能过滤 RViz 卡顿，松手后滑行也只有 ~3 cm。
        # 手动指令的"无消息容忍窗口"。注意它只在**收不到任何消息**时生效；
        # 面板松手会立刻发一帧 0，所以松手照样马上停。
        # 之前 0.25 s：WiFi 偶发 300~600 ms 抖动就插 'x' 停车 → 现场"偶尔卡一下"。
        self.declare_parameter('manual_cmd_hold_s', 1.0)
        # 2026-11-26 用户："停止时一下子停住，惯性太大"。
        # 固件收到 `v 0 0` 会直接 base_stop()（PWM 归零），不减速；这里在 ROS 侧补
        # 一条减速斜坡：0.50 m/s² → 0.40 m/s 停稳约 0.8 s、滑行约 0.16 m。
        # 0 = 关闭（恢复"一帧停住"）。只作用于手动/跟随通道，导航（auto）保持立即停。
        self.declare_parameter('manual_decel_mps2', 0.50)
        # 2026-11-12：掉头对准（frontier_explorer 的 prealign）需要原地转（PIVOT）。
        # 现在只有 frontier_explorer 会调 /turn_relative，且调用前已确认没有导航目标在跑。
        self.declare_parameter('allow_pivot', True)
        # 2026-11-03：固件速度环的**稳态标度误差**补偿。
        # 实测：下发 141 mm/s，固件自己也报 214 mm/s（说明它知道跑快了，
        # 但积分下限为 0、无法向下修正），实际就是 214。
        # 试过放开负积分，结果引起极限环振荡（0↔0.46 m/s，12 次停走），已回退。
        # 因此改在这里按固定比例补偿：乘 0.66 让实际速度≈指令速度。
        # 同时缩放 v 和 ω，转弯半径不变。
        # 2026-11-09 固件按 100:1 电机重标速度环（100 mm/s 跟踪 102.8），
        # 不再需要软件补偿，故改回 1.0；0.66 只适用于旧固件。
        # 2026-11-15（用户要求）：改回 1.0。0.66 时导航明显变慢（相当于把指令打 6.6 折）。
        # 注意：如果 ESP32 还是未重新标定的旧固件，1.0 下实际速度 ≈ 指令×1.5，
        # 若再出现堵转/超速 fault，再考虑 0.8 附近的折中值。
        self.declare_parameter('wheel_speed_scale', 1.0)
        self._device = str(self.get_parameter('device').value)
        self._core = BaseBridgeCore()
        self._core.decel_mm_s2 = 1000.0 * max(
            0.0, float(self.get_parameter('manual_decel_mps2').value))
        self._permit = RecoveryPermitCore()
        self._explore_mode = 'stopped'
        self._auto_armed = False
        self._stopped_at = None
        self._explore_state_at = None
        self._safe_cmd_at = None
        self._last_recovery_xy = None
        self.declare_parameter('sensor_timeout_s', 0.5)
        _st = float(self.get_parameter('sensor_timeout_s').value)
        self._scan = ScanWatchdog(_st)
        self._scan_stamp = SensorStampGuard(_st)
        self._imu_stamp = SensorStampGuard(_st)
        self._fd = None
        self._pending = bytearray()
        self._recent_states = deque(maxlen=12)
        self._last_open_attempt = 0.0
        # 串口（重新）连上后，第一次看到固件报告的非零故障时自动清一次。
        self._clear_fault_on_connect = False
        self._last_auto_clear = 0.0
        self._last_handshake_attempt = 0.0
        self._last_state_req = 0.0
        self._last_sent_command = None
        # 串口保活：记录最后一次收到状态帧的时间（ESP32 只在这条链路上报状态）
        self._last_state_at = 0.0
        # 2026-11-16（用户要求）：雷达断流导致撤授权后，恢复时自动重新授权，
        # 这样任务可以自动继续，不用人再点一次"开始导航"。
        self._arm_requested = False
        self._last_link_poke = 0.0
        self._imu_yaw = None
        self._imu_yaw_rate = 0.0
        self._last_imu_at = None
        self._nav_active = {'pose': False, 'through': False}
        # 2026-11-18：让位节点（沿轨迹倒车/转弯让位）走的是手动通道，
        # 但 command_source_allowed() 规定"导航任务进行中就拒绝手动指令"。
        # 现场后果：Nav2 目标还活着时开始倒车 → 手动指令被拦 → 10 s 只挪 0.02 m，
        # 用户看到的就是"该后退没后退"，随后连续让位 3 次把任务结束。
        # 让位节点自己发布的 /reverse/active 就是"这手动指令是它发的"的凭据。
        self._reverse_active = False
        self.create_subscription(
            Bool, '/reverse/active',
            lambda m: setattr(self, '_reverse_active', bool(m.data)), 10)
        # 2026-11-12：幽灵导航目标会让手动指令被静默丢弃（现场"按了完全不动"）。
        # 记录每个 action status 的更新时间，并在手动通道打开时强制清零。
        self._nav_status_at = {'pose': 0.0, 'through': 0.0}
        self._manual_enabled_prev = False
        self._command_source = None
        self._safe_cmd_zero = True
        self._manual_yaw_rate = ImuYawRateController()
        self.declare_parameter('heading_hold_enabled', True)
        # 沿墙建图需要“边走边转”，默认关闭以保持键盘单轴习惯
        self.declare_parameter('manual_allow_mixed', False)
        # 2026-10-31：后向超声守卫。true 时拒绝任何"后退"指令（v<0）。
        # 只限制后退，不产生前进/倒车指令；不改变自动倒车许可。
        self.declare_parameter('rear_guard_enabled', True)
        # 手动模式是否也受车尾超声守卫限制。默认 False：
        # 手动是有人在看着的低速操作，车尾超声坏了也不该把后退锁死。
        # 自动/导航链路仍然照旧受 rear_guard 保护。
        self.declare_parameter('manual_rear_guard_enabled', False)
        # 手动模式的**单轮**速度上限。0.30 m/s 直行时单轮正好 300 mm/s，
        # 是固件 parse_velocity 的硬上限；再高固件会直接丢弃该帧。
        # 2026-11-26 用户批准：0.30 → 0.40（跟随前进速度随之提到 0.38）。
        # ⚠️ 这是**手动通道共用**的底盘安全参数：手动/倒车也一起变快，
        # 手机摇杆另有 mqtt_bridge 的 max_linear_mps 0.15 钳位。
        self.declare_parameter('manual_max_speed_mps', 0.40)
        # 手动模式是否要求雷达数据新鲜。默认 False：手动是有人看守的低速操作，
        # 雷达偶发 2 s 卡顿（本机已知硬件问题）不该把车掐停 —— 那正是
        # "按住 W 一卡一卡"的直接原因。自动/导航链路仍然强制要求雷达。
        self.declare_parameter('manual_requires_scan', False)
        # 导航倒退限速（绝对值）：车尾 4 点超声覆盖有限，慢一点更稳
        self.declare_parameter('max_reverse_mps', 0.12)
        self.declare_parameter('heading_hold_gain', 1.5)
        self.declare_parameter('heading_hold_limit_rad_s', .12)
        self._heading_hold = HeadingHold(
            float(self.get_parameter('heading_hold_gain').value),
            float(self.get_parameter('heading_hold_limit_rad_s').value))
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 lambda msg: self._on_nav_status('pose', msg),
                                 qos_profile_action_status_default)
        self.create_subscription(GoalStatusArray, '/navigate_through_poses/_action/status',
                                 lambda msg: self._on_nav_status('through', msg),
                                 qos_profile_action_status_default)
        self._wheel_odom = self.create_publisher(Odometry, '/wheel/odom', 10)
        self._joint_states = self.create_publisher(JointState, '/joint_states', 10)
        self._fault = self.create_publisher(Int32, '/base/fault', 10)
        self._motor_debug = self.create_publisher(Float64MultiArray, '/base/motor_debug', 10)
        self._manual_debug = self.create_publisher(Float64MultiArray, '/base/manual_debug', 10)
        self._ready = self.create_publisher(Bool, '/base/ready', 10)
        self._manual_mode = self.create_publisher(Bool, '/base/manual_enabled', 10)
        self._auto_arm_pub = self.create_publisher(Bool, '/base/auto_armed', 10)
        # 2026-10-31：车尾超声守卫（rear_guard 发布；true = 后方近障，禁止后退）
        self._rear_blocked = False
        self.create_subscription(Bool, '/base/rear_blocked', self._on_rear_blocked, 10)
        self.create_subscription(Twist, '/cmd_vel_auto_safe', self._on_velocity, 10)
        self.create_subscription(String, '/explore/state', self._on_explore_state, 10)
        self.create_service(RecoveryPermit, '/base/recovery_permit', self._on_recovery_permit)
        self.create_service(Trigger, '/base/arm_auto', self._arm_auto)
        self.create_service(Trigger, '/base/clear_fault', self._clear_fault)
        self.create_subscription(Twist, '/cmd_vel_manual', self._on_manual_velocity, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self._turn_action = ActionServer(
            self, TurnRelative, 'turn_relative',
            execute_callback=self._execute_turn,
            goal_callback=self._turn_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.create_timer(.05, self._tick)

    def _on_explore_state(self, msg: String) -> None:
        old_mode = self._explore_mode
        self._explore_mode = msg.data
        self._explore_state_at = time.monotonic()
        if msg.data != 'reversing':
            self._permit.abort()
        # 2026-10-29：授权不能一见到 stopped 就撤销——frontier_explorer 空闲时 10 Hz
        # 发 stopped，而“面板授权 → 下发目标”之间有 0.5~1 s 窗口，期间授权必被冲掉
        # （表现为 preflight failed: arm）。改为：stopped 持续 2 s 且没有活动导航目标
        # 才真正注销授权（由 _tick 执行）。
        if msg.data == 'stopped':
            if old_mode != 'stopped':
                self._stopped_at = time.monotonic()
        else:
            self._stopped_at = None

    def _arm_auto(self, _, response):
        # 2026-10-30：面板的“开门 → 授权”是两次独立的服务调用，参数生效有几十毫秒
        # 的延迟；原来一次性判断，导致第二次点导航常常报 preflight failed: arm。
        # 这里改成最多等 1.5 s 再给结论。
        deadline = time.monotonic() + 1.5
        reason = 'base/sensors unavailable'
        while True:
            now = time.monotonic()
            # 2026-11-18 现场（"退完就失控"）：这里原来用 core.ready()，它要求
            # **ESP 状态帧新鲜**；而固件只在串口空闲时才上报状态 —— 底盘正在被
            # 连续下发速度（倒车刚结束就是这种状态）时状态必然变旧，
            # 于是 arm_auto 连报 4 次 "serial not ready" → 自动授权没恢复 →
            # Nav2 再也接不上，只能"重新点开始导航"。
            # 与 accept_cmd 保持一致：只要求"链路已连接 + 握手完成 + 无故障"。
            if not (self._core.connected and self._core.verified
                    and self._core.fault == 0):
                reason = 'serial not ready'
            elif not self._sensors_ready(now):
                reason = 'scan/imu not fresh'
            elif bool(self.get_parameter('manual_enabled').value):
                reason = 'manual enabled'
            else:
                self._auto_armed = True
                # 关键：清掉“stopped 已持续多久”的计时器。
                # 空闲时 frontier_explorer 一直发 stopped，若不清零，授权后下一个 tick
                # 立即判定“stopped 已超过 3 秒” → 刚授权就被撤销（2026-10-30 实测）。
                self._stopped_at = None
                response.success = True
                response.message = 'armed'
                return response
            if now >= deadline:
                break
            time.sleep(0.03)
        self._auto_armed = False
        response.success = False
        response.message = reason
        # 2026-11-15：授权失败时把各项判据打出来，便于现场定位（哪一项不满足）。
        age = (None if self._core._last_state_at is None
               else round(time.monotonic() - self._core._last_state_at, 3))
        self.get_logger().warn(
            'arm 失败：reason=%s core.ready=%s state_age=%s verified=%s '
            'sensors=%s scan_ok=%s imu_age=%s manual=%s' % (
                reason, self._core.ready(time.monotonic()), age, self._core.verified,
                self._sensors_ready(time.monotonic()),
                self._scan.ready(time.monotonic()),
                (None if self._last_imu_at is None
                 else round(time.monotonic() - self._last_imu_at, 2)),
                bool(self.get_parameter('manual_enabled').value)))
        return response

    def _clear_fault(self, _, response):
        now = time.monotonic()
        allowed = fault_clear_allowed(
            bool(self.get_parameter('motion_enabled').value),
            bool(self.get_parameter('manual_enabled').value),
            self._auto_armed, any(self._nav_active.values()),
            self._explore_mode,
            self._fd is not None and self._core.connected and self._core.verified)
        if not allowed:
            response.success = False
            response.message = 'motion paths must be disabled and serial connected'
            return response
        self._core.halt()
        self._permit.abort()
        if not self._core.request_fault_clear(now):
            response.success = False
            # 区分两种失败：没有故障可清（fault=0，属于正常）vs 串口状态不新鲜。
            if not self._core.fault:
                response.message = '当前无故障（fault=0），无需复位'
            else:
                response.message = '底盘状态不新鲜，无法确认清除结果（检查串口）'
            return response
        try:
            if os.write(self._fd, b'xc') != 2:
                raise OSError('short serial write')
        except OSError as exc:
            self._disconnect(str(exc))
            response.success = False
            response.message = 'serial write failed'
            return response
        response.success = True
        response.message = 'clear sent; awaiting fault-free ESP state with motion disabled'
        return response

    def _on_recovery_permit(self, request, response):
        now = time.monotonic()
        healthy = (self._sensors_ready(now) and self._core.ready(now) and
                   self._explore_mode == 'reversing' and
                   self._explore_state_at is not None and
                   now - self._explore_state_at <= .25)
        accepted, reason = self._permit.request(
            request.enable, request.episode_id, now, healthy,
            not any(self._nav_active.values()),
            not bool(self.get_parameter('manual_enabled').value))
        response.accepted, response.reason = accepted, reason
        if not accepted or not request.enable:
            self._core.halt()
        return response

    def _reject_motion(self, reason: str) -> None:
        """Halt and (when FWR_BASE_DEBUG is set) report which gate closed."""
        self._core.halt()
        if _DEBUG_REJECT:
            print(f'[base] reject {reason}', flush=True)

    def _on_velocity(self, msg: Twist) -> None:
        now = time.monotonic()
        self._safe_cmd_at = now
        self._safe_cmd_zero = abs(msg.linear.x) < 1e-9 and abs(msg.angular.z) < 1e-9
        # 2026-11-12（本轮顿挫的根因）：/cmd_vel_auto_safe 由碰撞监控/选择器以 20 Hz
        # 持续发布（空闲时是 0）。原来这里只要 manual_enabled=True 就 _core.halt()，
        # 等于每秒 20 次把手动目标清零 → 手动模式"走一下停一下"（实测 4 s 走/3 s 停）。
        # 手动模式下手动链路优先：自动链路的消息直接忽略（源是 auto 时才真正停机）。
        if bool(self.get_parameter('manual_enabled').value):
            self._auto_armed = False
            self._permit.abort()
            if self._command_source == 'auto':
                self._core.halt()
            return
        if not self._sensors_ready(now) or not self._core.ready(now):
            # 2026-11-12（A）：这里原来会清掉 _auto_armed。雷达只要卡 2 秒，
            # 自动授权就被永久撤销，雷达恢复后 Nav2 的指令仍被拒
            # （[tx] b'x' src=auto tgt=None），恢复旋转也转不了 → Goal failed。
            # 现在只暂停本帧（门控本来就不会输出），授权保留，传感器恢复后自动继续。
            self._permit.abort()
            self._reject_motion('sensors')
            return
        if self._explore_state_at is None or now - self._explore_state_at > .25:
            self._permit.abort()
            self._reject_motion('state-stale')
            return
        if abs(msg.linear.x) < 1e-9 and abs(msg.angular.z) < 1e-9:
            if self._command_source == 'auto':
                self._core.halt()
            return
        if not self._auto_armed:
            self._reject_motion('not-armed')
            return
        if self._explore_mode == 'reversing':
            if any(self._nav_active.values()) or not self._permit.accept_reverse(msg.linear.x, now):
                self._core.halt()
                return
            if abs(msg.angular.z) > .15:
                self._core.halt()
                return
        elif self._explore_mode == 'navigating':
            # 倒车与非法值仍然拒绝；两轮超固件上限时同比缩放 v、w，
            # 保留 Nav2 请求的曲率，避免夹小角速度后转弯半径变大。
            if (not math.isfinite(msg.linear.x) or
                    not math.isfinite(msg.angular.z)):
                self._core.halt()
                return
            nav_v = msg.linear.x
            if nav_v < 0.0:
                # 2026-10-31：允许 Nav2 倒退，但必须"车尾守卫开启且无近障"
                if not nav_reverse_ok(nav_v, self._rear_blocked,
                                      bool(self.get_parameter('rear_guard_enabled').value)):
                    self._core.halt()
                    self.get_logger().warn('导航后退被车尾守卫拦截',
                                           throttle_duration_sec=2.0)
                    return
                cap = abs(float(self.get_parameter('max_reverse_mps').value))
                nav_v = max(nav_v, -cap)
            auto_v, auto_w = scale_velocity_to_wheel_limit(
                nav_v, msg.angular.z, self._core.track_m)
            # v 与 ω 同比缩放，保持曲率（转弯半径）不变
            wheel_scale = float(self.get_parameter('wheel_speed_scale').value)
            auto_v *= wheel_scale
            auto_w *= wheel_scale
        else:
            self._reject_motion('mode-' + str(self._explore_mode))
            return
        try:
            self._manual_yaw_rate.reset()
            if self._explore_mode == 'reversing':
                self._core.accept_cmd(msg.linear.x, msg.angular.z, now)
            else:
                self._core.accept_cmd(auto_v, auto_w, now)
            self._command_source = 'auto'
        except (ValueError, RuntimeError) as exc:
            self._core.halt()
            self.get_logger().warn(f'Rejected filtered automatic velocity: {exc}',
                                   throttle_duration_sec=2.0)

    def _on_manual_velocity(self, msg: Twist) -> None:
        now = time.monotonic()
        debug = Float64MultiArray()
        debug.data = [now, msg.linear.x, msg.angular.z,
                      float(self._core.ready(now)), float(self._sensors_ready(now)),
                      float(bool(self.get_parameter('manual_enabled').value))]
        self._manual_debug.publish(debug)
        # 无条件记录"到达"，用来区分"没收到"和"收到但被丢弃"
        self.get_logger().info(
            f'[manual-in] rx v={msg.linear.x:.3f} w={msg.angular.z:.3f} '
            f'manual={bool(self.get_parameter("manual_enabled").value)} '
            f'nav={any(self._nav_active.values())} rev={self._reverse_active} '
            f'motion={bool(self.get_parameter("motion_enabled").value)} '
            f'conn={self._core.connected} ver={self._core.verified} '
            f'fault={self._core.fault} turn={self._core._turn_owner}',
            throttle_duration_sec=1.0)
        # 2026-11-18 现场（"本该后退却前进了"）：让位倒车走手动通道，但手动通道
        # 是否打开取决于 trail_reverser 异步调用 /base_bridge/set_parameters。
        # 那次调用没落地（服务没就绪/竞争），于是 **手动通道关闭**：
        #   倒车指令全被丢弃（车身不退），而自动通道仍然畅通 → Nav2 继续往前开，
        #   本该后退的动作变成"前进顶到障碍上"。
        # 现在：只要 /reverse/active 为真，手动通道就**由底盘自己放行**，
        # 不再依赖对端那次参数调用；导航态也不再拦截。
        manual_ok = (bool(self.get_parameter('manual_enabled').value)
                     or self._reverse_active)
        if not command_source_allowed(
                'manual', manual_ok,
                any(self._nav_active.values()) and not self._reverse_active):
            self.get_logger().warn('手动指令被导航任务状态拦截（nav_active 残留？）',
                                   throttle_duration_sec=2.0)
            return
        if not self._sensors_ready(now, require_scan=bool(
                self.get_parameter('manual_requires_scan').value),
                allow_reverse=self._reverse_active):
            # 2026-11-18 现场（"撞到障碍物了，没有后退"）：这条 return 以前是**静默**的。
            # 倒车节点发了 12 s 指令、车身一动不动，日志里一个字都没有 ——
            # 真凶是 motion_enabled 已被面板/任务结束关掉，_sensors_ready 直接 False。
            # 2026-11-18：这里原来只在 _reverse_active 为真时告警。
            # 但现场正是 _reverse_active **没收到**（DDS 没连上，单次 latch 丢失）
            # 才导致的丢弃 —— 于是又变成静默。现在只要手动门开着就告警。
            if self._reverse_active or bool(
                    self.get_parameter('manual_enabled').value):
                self.get_logger().warn(
                    '手动/倒车指令被丢弃：运动门未开或传感器未就绪'
                    f'（motion_enabled={bool(self.get_parameter("motion_enabled").value)}，'
                    f'reverse_active={self._reverse_active}）',
                    throttle_duration_sec=2.0)
            return
        if (msg.linear.x < -1e-6 and self._rear_blocked and
                bool(self.get_parameter('rear_guard_enabled').value) and
                bool(self.get_parameter('manual_rear_guard_enabled').value)):
            self._core.halt()
            self.get_logger().warn('后退被车尾超声守卫拦截', throttle_duration_sec=2.0)
            return
        if not manual_velocity(msg.linear.x, msg.angular.z, self._core.track_m,
                                max_linear_mps=float(
                                    self.get_parameter('manual_max_speed_mps').value),
                                # 2026-11-18 现场（"后退一直退不动"）：manual_velocity
                                # 默认只允许**单轴**（键盘安全习惯），v 和 ω 同时非零
                                # 直接判非法 → 底盘 halt()，命令连目标都没设。
                                # 而沿轨迹倒车必然是"边退边修方向"（v=-0.08, ω=-0.15），
                                # 于是每一帧都被拒 —— 机器人一动不动，日志里只有一句
                                # "Rejected out-of-range manual velocity"。
                                # 让位倒车天然需要混合速度，这里对它放行。
                                allow_mixed=(bool(
                                    self.get_parameter('manual_allow_mixed').value)
                                    or self._reverse_active)):
            self._core.halt()
            self.get_logger().warn(
                'Rejected out-of-range manual velocity'
                f'（v={msg.linear.x:.3f} w={msg.angular.z:.3f} '
                f'mixed={bool(self.get_parameter("manual_allow_mixed").value)} '
                f'rev={self._reverse_active}）',
                throttle_duration_sec=2.0)
            return
        try:
            # 2026-11-12：手动模式改为**直通**，不再用 IMU 做航向保持/转向速率修正。
            # 原因：手动直行时航向保持会不断加大左右轮差速（实测 121/118 → 131/108，
            # 等效 ω 从 0.02 涨到 0.145 rad/s 并顶到限幅），车一边走一边被拽，
            # 现场感受就是"一卡一卡"。手动是人在操控，转向应该由人决定。
            self._manual_yaw_rate.reset()
            self._heading_hold.reset()
            corrected_yaw_rate = msg.angular.z
            # 2026-11-12：0.25/0.30 m/s 档"一卡一卡"的根因 ——
            # 航向保持会给一点点 ω，单轮 = v + ω·track/2 就会超过固件 ±300 mm/s，
            # encode_wheel_speed 直接抛异常 → 整帧被丢 → 停一下再走。
            # 这里先按单轮上限等比缩放（保持曲率），永远不产生非法帧。
            v_cmd, w_cmd = scale_velocity_to_wheel_limit(
                msg.linear.x, corrected_yaw_rate, self._core.track_m,
                max_wheel_mps=float(self.get_parameter('manual_max_speed_mps').value))
            self._core.accept_cmd(v_cmd, w_cmd, now,
                                  hold_s=float(self.get_parameter('manual_cmd_hold_s').value))
            self._command_source = 'manual'
            # 诊断：把手动链路的内部状态打出来（每秒 1 行），定位"收到但不执行"
            self.get_logger().info(
                f'[manual] rx v={msg.linear.x:.3f} w={msg.angular.z:.3f} '
                f'out v={v_cmd:.3f} w={w_cmd:.3f} target={self._core._target} '
                f'sensors={self._sensors_ready(now, require_scan=False)} '
                f'scan={self._scan.ready(now)} imu_age={0 if self._last_imu_at is None else round(now - self._last_imu_at, 3)}',
                throttle_duration_sec=1.0)
        except (ValueError, RuntimeError) as exc:
            self.get_logger().warn(f'Rejected manual velocity: {exc}',
                                   throttle_duration_sec=2.0)

    def _on_scan(self, msg: LaserScan) -> None:
        now = time.monotonic()
        if (msg.header.frame_id == 'laser_front' and
                any(math.isfinite(r) and msg.range_min <= r <= msg.range_max
                    for r in msg.ranges) and
                self._scan_stamp.accept(self._stamp_s(msg.header.stamp),
                                        self._ros_now_s(), now)):
            self._scan.observe(msg.ranges, now)

    def _ros_now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _stamp_s(stamp) -> float:
        return stamp.sec + stamp.nanosec / 1e9

    def _sensors_ready(self, now: float, require_scan: bool = True,
                       allow_reverse: bool = False) -> bool:
        # allow_reverse=True：让位节点（/reverse/active）在用**手动通道**倒车。
        # 它属于"用户已经开始的导航任务"的一部分，门被任务结束/面板关掉时
        # 仍应允许它把车退出来 —— 否则车会顶在障碍前一米也退不动。
        # 注意：只放宽运动门，扫描/IMU 的有效性**一律照旧**（数据不新绝对不走）。
        if not allow_reverse and not bool(
                self.get_parameter('motion_enabled').value):
            return False
        if require_scan and not (self._scan.ready(now) and self._scan_stamp.ready(now)):
            return False
        return (self._imu_stamp.ready(now) and self._last_imu_at is not None and
                0 <= now - self._last_imu_at <= float(self.get_parameter('sensor_timeout_s').value))

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        now = time.monotonic()
        if (msg.header.frame_id != 'imu_link' or
                not .9 <= norm <= 1.1 or
                not math.isfinite(msg.angular_velocity.z) or
                msg.orientation_covariance[0] == -1 or
                not self._imu_stamp.accept(self._stamp_s(msg.header.stamp),
                                           self._ros_now_s(), now)):
            return
        self._imu_yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                                   1 - 2*(q.y*q.y + q.z*q.z))
        self._imu_yaw_rate = msg.angular_velocity.z
        self._last_imu_at = time.monotonic()

    def _on_rear_blocked(self, msg: Bool) -> None:
        """车尾超声守卫：true 表示后方近障，禁止后退。"""
        if bool(msg.data) and not self._rear_blocked:
            self.get_logger().warn('车尾近障：禁止后退')
        self._rear_blocked = bool(msg.data)

    def _on_nav_status(self, kind: str, msg: GoalStatusArray) -> None:
        self._nav_status_at[kind] = time.monotonic()
        self._nav_active[kind] = any(
            item.status in (1, 2, 3) for item in msg.status_list)

    def _turn_goal(self, request):
        now = time.monotonic()
        turn_why = []
        if not self._sensors_ready(now):
            turn_why.append('sensors')
        if any(self._nav_active.values()):
            turn_why.append('nav_active')
        if (request.mode == TurnRelative.Goal.PIVOT and
                not bool(self.get_parameter('allow_pivot').value)):
            turn_why.append('pivot_disabled')
        if self._imu_yaw is None or self._last_imu_at is None:
            turn_why.append('no_imu_yaw')
        elif self._imu_stamp.age_at(now) > .15:
            turn_why.append('imu_stale')
        if turn_why:
            self.get_logger().warn('转向目标被拒：' + ','.join(turn_why),
                                   throttle_duration_sec=1.0)
            return GoalResponse.REJECT
        try:
            TurnController(request.angle_deg, request.mode, now)
        except ValueError:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT if self._core.begin_turn(now) else GoalResponse.REJECT

    async def _execute_turn(self, goal_handle):
        now = time.monotonic()
        controller = TurnController(goal_handle.request.angle_deg,
                                    goal_handle.request.mode, now)
        start_yaw = self._imu_yaw
        start_wheels = tuple(self._core._wheel_positions)
        finished = None
        try:
            while rclpy.ok():
                now = time.monotonic()
                if goal_handle.is_cancel_requested:
                    finished = controller.cancel()
                    goal_handle.canceled()
                    break
                if (not self._sensors_ready(now) or any(self._nav_active.values()) or
                        (controller.mode == TurnController.PIVOT and
                         not bool(self.get_parameter('allow_pivot').value))):
                    finished = TurnFinished(False, 'motion_disabled', 0)
                elif not self._core.ready(now) or self._imu_yaw is None:
                    finished = TurnFinished(False, 'base_stale', 0)
                else:
                    radius_m = .210 / (2 * math.pi)
                    left_progress = (self._core._wheel_positions[0] - start_wheels[0]) * radius_m
                    right_progress = (self._core._wheel_positions[1] - start_wheels[1]) * radius_m
                    relative_yaw = self._imu_yaw - start_yaw
                    finished = controller.update(
                        relative_yaw, self._imu_yaw_rate, now,
                        left_progress, right_progress,
                        imu_age_s=self._imu_stamp.age_at(now), fault=self._core.fault)
                if isinstance(finished, TurnFinished):
                    if finished.success:
                        goal_handle.succeed()
                    else:
                        goal_handle.abort()
                    break
                try:
                    self._core.accept_turn_cmd(*finished, now)
                except (RuntimeError, ValueError):
                    finished = TurnFinished(False, 'base_unavailable', 0)
                    goal_handle.abort()
                    break
                feedback = TurnRelative.Feedback()
                feedback.remaining_angle_deg = float(controller.remaining_angle_deg)
                goal_handle.publish_feedback(feedback)
                await self._sleep_ros(.05)
            if finished is None:
                finished = TurnFinished(False, 'shutdown', 0)
                goal_handle.abort()
        finally:
            self._core.end_turn()
            if self._fd is not None:
                try:
                    os.write(self._fd, b'x')
                except OSError:
                    pass
        result = TurnRelative.Result()
        result.success = bool(finished and finished.success)
        result.reason = finished.reason if finished else 'shutdown'
        result.actual_angle_deg = float(finished.actual_angle_deg if finished else 0)
        return result

    async def _sleep_ros(self, delay_s: float) -> None:
        """Yield the rclpy executor; asyncio has no running loop here."""
        future = Future()
        timer = self.create_timer(delay_s, lambda: future.set_result(None)
                                  if not future.done() else None)
        try:
            await future
        finally:
            self.destroy_timer(timer)

    def _open_if_needed(self, now: float) -> None:
        if self._fd is not None or now - self._last_open_attempt < 1.0:
            return
        self._last_open_attempt = now
        try:
            self._fd = open_exclusive_serial(self._device)
            self._pending.clear()
            self._core.connect(now)
            self._permit.abort()
            os.write(self._fd, b'x?')
            self._last_handshake_attempt = now
            self._clear_fault_on_connect = True
            self._last_state_at = now
            self.get_logger().info(f'ESP32 serial opened exclusively: {self._device}')
        except (OSError, ValueError) as exc:
            self.get_logger().warn(f'ESP32 serial unavailable: {exc}', throttle_duration_sec=5.0)

    def _disconnect(self, reason: str) -> None:
        self._auto_armed = False
        if self._fd is not None:
            try:
                os.write(self._fd, b'x')
            except OSError:
                pass
            os.close(self._fd)
            self._fd = None
        self._core.disconnect()
        self._permit.reset()
        self._pending.clear()
        self.get_logger().error(f'ESP32 serial disconnected: {reason}')

    def _consume(self, data: bytes, now: float) -> None:
        self._pending.extend(data)
        if len(self._pending) > 4096:
            self._pending.clear()
            self._disconnect('serial buffer overflow')
            return
        while b'\n' in self._pending:
            line, _, rest = self._pending.partition(b'\n')
            self._pending = bytearray(rest)
            if len(line) > MAX_LINE:
                continue
            decoded = line.decode('ascii', errors='replace').strip()
            if decoded == 'BASE_CONTROL_V2':
                self._core.handshake(decoded)
                continue
            state = parse_state_line(decoded)
            if state is None:
                continue
            self._last_state_at = now
            debug = Float64MultiArray()
            debug.data = [float(state.esp_us), state.left_measured,
                          state.right_measured, state.left_setpoint,
                          state.right_setpoint, state.left_pwm,
                          state.right_pwm, self._imu_yaw_rate, float(state.fault)]
            self._motor_debug.publish(debug)
            self._recent_states.append(
                (state.esp_us, state.left_count, state.right_count,
                 round(state.left_measured), round(state.right_measured),
                 round(state.left_pwm, 2), round(state.right_pwm, 2), state.fault))
            if (state.fault and self._clear_fault_on_connect and
                    not state.active and self._fd is not None):
                # USB 插拔后固件常留一次超速/堵转锁存；仅在刚连上且电机空闲时
                # 自动清一次，避免每次都手动发 /base/clear_fault。
                try:
                    # 固件只在 active=false 时才接受 'c'，必须先发 'x' 停车再清。
                    os.write(self._fd, b'xc')
                    self._core.note_fault_clear_sent()
                    self._clear_fault_on_connect = False
                    self.get_logger().warn(
                        f'cleared latched ESP32 fault {state.fault} after serial connect')
                except OSError:
                    pass
            if state.fault and not self._core.fault:
                self.get_logger().error(
                    f'ESP32 fault {state.fault}; recent states '
                    f'(us,Lcount,Rcount,Lmm/s,Rmm/s,Lpwm,Rpwm,fault): '
                    f'{list(self._recent_states)}')
            previous_esp_us = self._core._last_esp_us
            out = self._core.step(state, now)
            if previous_esp_us is not None and state.esp_us < previous_esp_us:
                if self._fd is not None:
                    os.write(self._fd, b'x?')
            if out.fault:
                print('[base] disarm fault', flush=True)
                self._auto_armed = False
                self._permit.abort()
            if out.odom is not None and self._permit.active:
                xy = (out.odom.x, out.odom.y)
                if self._last_recovery_xy is not None:
                    self._permit.observe_distance(math.dist(xy, self._last_recovery_xy))
                self._last_recovery_xy = xy
                if not self._permit.active:
                    self._core.halt()
            else:
                self._last_recovery_xy = None
            if out.odom is not None:
                self._publish_odometry(out.odom, out.wheel_positions_rad)

    def _publish_odometry(self, sample, wheel_positions_rad) -> None:
        stamp = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = sample.x
        msg.pose.pose.position.y = sample.y
        msg.pose.pose.orientation.z = math.sin(sample.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(sample.yaw / 2)
        msg.twist.twist.linear.x = sample.linear_mps
        msg.twist.twist.angular.z = sample.angular_rps
        for index in (0, 7, 35):
            msg.pose.covariance[index] = .1
            msg.twist.covariance[index] = .1
        for index in (14, 21, 28):
            msg.pose.covariance[index] = 1e6
            msg.twist.covariance[index] = 1e6
        self._wheel_odom.publish(msg)
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ['left_drive_wheel_preview_joint', 'right_drive_wheel_preview_joint']
        joints.position = list(wheel_positions_rad)
        self._joint_states.publish(joints)

    def _tick(self) -> None:
        now = time.monotonic()
        manual_enabled = bool(self.get_parameter('manual_enabled').value)
        self._manual_mode.publish(Bool(data=manual_enabled))
        # ① 状态消息 2 s 没更新 → 视为残留目标，清掉（否则手动指令被一直丢）
        for kind in list(self._nav_active):
            if now - self._nav_status_at.get(kind, 0.0) > 2.0:
                if self._nav_active[kind]:
                    print(f'[base] cleared stale nav_active[{kind}]', flush=True)
                self._nav_active[kind] = False
        # ② 手动通道刚打开的那一帧：强制清零（面板进手动模式前已经停掉导航任务）
        if manual_enabled and not self._manual_enabled_prev:
            if any(self._nav_active.values()):
                print('[base] manual on -> cleared nav_active', flush=True)
            self._nav_active = {'pose': False, 'through': False}
        self._manual_enabled_prev = manual_enabled
        if self._command_source == 'auto':
            gap_action = safe_command_gap_action(
                now - self._safe_cmd_at if self._safe_cmd_at is not None else float('inf'),
                self._safe_cmd_zero)
            if gap_action != 'ok':
                self._core.halt()
            if gap_action == 'disarm':
                print(f'[base] disarm gap {now - (self._safe_cmd_at or 0):.2f}s zero={self._safe_cmd_zero}', flush=True)
                self._auto_armed = False
                self._permit.abort()
                self._command_source = None
        if ((self._command_source == 'manual' and
             not command_source_allowed('manual', manual_enabled,
                                        any(self._nav_active.values()))) or
            (self._command_source == 'auto' and manual_enabled)):
            self._core.halt()
            self._manual_yaw_rate.reset()
            self._command_source = None
        # 导航通道不做减速斜坡（collision_monitor / Nav2 要求"说停就停"）；
        # 手动、跟随走斜坡（用户 2026-11-26 要求软停）。
        self._core.decel_enabled = (self._command_source != 'auto')
        self._core.decel_mm_s2 = 1000.0 * max(
            0.0, float(self.get_parameter('manual_decel_mps2').value))
        self._open_if_needed(now)
        if (self._fd is not None and self._core.fault and
                not self._core.fault_clear_pending and
                not self._auto_armed and self._safe_cmd_zero and
                now - self._last_auto_clear >= 2.0):
            # 门控已开、指令为零且未授权（电机空闲）时，自动清一次锁存的固件故障。
            # USB 插拔或编码器毛刺会留下 fault 1/3，否则面板会一直报“底盘未就绪”。
            try:
                os.write(self._fd, b'xc')
                self._core.note_fault_clear_sent()
                self._last_auto_clear = now
                self.get_logger().warn(
                    f'auto-clearing latched ESP32 fault {self._core.fault} '
                    '(idle, gate open)')
            except OSError:
                pass
        if self._fd is not None:
            try:
                for _ in range(8):
                    try:
                        chunk = os.read(self._fd, 4096)
                    except BlockingIOError:
                        break
                    if not chunk:
                        if not os.path.exists(self._device):
                            self._disconnect('device disappeared')
                        break
                    self._consume(chunk, now)
                    if self._fd is None:
                        break
                if self._fd is not None:
                    # 2026-11-12：ESP32 状态帧静默（现场出现过：进程/句柄都在，但一条
                    # 状态都收不到 → /base/ready=false → 面板"底盘未就绪"、导航停）。
                    # 这里做保活：2 s 没状态就重新握手，6 s 还没恢复就断开重开串口。
                    silence = now - self._last_state_at if self._last_state_at else 0.0
                    if silence > 2.0 and now - self._last_link_poke >= 1.0:
                        self._last_link_poke = now
                        self._core.verified = False
                        try:
                            os.write(self._fd, b'?')
                        except OSError as exc:
                            self._disconnect(str(exc))
                            return
                        print('[base] 状态帧静默 %.1fs → 重新握手' % silence, flush=True)
                    if silence > 6.0:
                        self._disconnect('state frames silent > 6 s')
                        return
                    if (not self._core.verified and
                            now - self._last_handshake_attempt >= 1.0):
                        os.write(self._fd, b'?')
                        self._last_handshake_attempt = now
                    output = self._core.step(None, now)
                    # 手动来源允许在雷达卡顿时继续（manual_requires_scan=False）
                    manual_source = (self._command_source == 'manual' and
                                     not bool(self.get_parameter(
                                         'manual_requires_scan').value))
                    if self._sensors_ready(now, require_scan=not manual_source):
                        if (self._arm_requested and not self._auto_armed
                                and not bool(self.get_parameter('manual_enabled').value)):
                            self._auto_armed = True
                            print('[base] 传感器恢复 → 自动重新授权', flush=True)
                        command = output.command
                        age = (None if self._core._last_cmd_at is None
                               else round(now - self._core._last_cmd_at, 3))
                        if _TX_LOG:
                            with open('/tmp/fwr_tx.log', 'a') as fh:
                                fh.write(f'{now:.3f} {command!r} '
                                         f'tgt={self._core._target} age={age} '
                                         f'src={self._command_source}\n')
                        else:
                            self.get_logger().info(
                                f'[tx] {command!r} src={self._command_source} '
                                f'tgt={self._core._target} age={age} '
                                f'hold={self._core._cmd_hold_s:.2f}',
                                throttle_duration_sec=1.0)
                    else:
                        print('[base] disarm tick-sensors', flush=True)
                        self._core.halt()
                        self._manual_yaw_rate.reset()
                        command = b'x'
                    # 连续相同的 'x' 不重复写：固件只有在串口空闲时才会每 100 ms
                    # 上报 S2 状态，一直写 'x' 会让状态断流、ready() 判假。
                    if not (command == b'x' and self._last_sent_command == b'x'):
                        os.write(self._fd, command)
                    self._last_sent_command = command
            except OSError as exc:
                self._disconnect(str(exc))
        if (self._stopped_at is not None and self._auto_armed and
                now - self._stopped_at > 3.0 and
                not any(self._nav_active.values())):
            print('[base] disarm explore-stopped (persisted 3s)', flush=True)
            self._auto_armed = False
            self._stopped_at = None
        self._fault.publish(Int32(data=self._core.fault))
        self._auto_arm_pub.publish(Bool(data=self._auto_armed))
        self._ready.publish(Bool(data=self._core.ready(now) and
                                 self._sensors_ready(now)))

    def destroy_node(self) -> bool:
        self._turn_action.destroy()
        self._disconnect('node shutdown')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
