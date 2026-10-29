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
                               fault_clear_allowed)
from .base_protocol import MAX_LINE, parse_state_line
from .base_serial import open_exclusive_serial
from .wheel_odometry_node import DEFAULT_DEVICE
from .turn_controller import TurnController, TurnFinished
from .heading_hold import HeadingHold
from .imu_yaw_rate_control import ImuYawRateController

_DEBUG_REJECT = bool(os.environ.get('FWR_BASE_DEBUG'))


class BaseBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('base_bridge')
        self.declare_parameter('device', DEFAULT_DEVICE)
        self.declare_parameter('motion_enabled', False)
        self.declare_parameter('manual_enabled', False)
        self.declare_parameter('allow_pivot', False)
        self._device = str(self.get_parameter('device').value)
        self._core = BaseBridgeCore()
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
        self._imu_yaw = None
        self._imu_yaw_rate = 0.0
        self._last_imu_at = None
        self._nav_active = {'pose': False, 'through': False}
        self._command_source = None
        self._safe_cmd_zero = True
        self._manual_yaw_rate = ImuYawRateController()
        self.declare_parameter('heading_hold_enabled', True)
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
        now = time.monotonic()
        healthy = (self._core.ready(now) and self._sensors_ready(now) and
                   not bool(self.get_parameter('manual_enabled').value))
        self._auto_armed = healthy
        response.success = healthy
        response.message = 'armed' if healthy else 'base/sensors unavailable'
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
        if not self._sensors_ready(now) or not self._core.ready(now):
            self._auto_armed = False
            self._permit.abort()
            self._reject_motion('sensors')
            return
        if self._explore_state_at is None or now - self._explore_state_at > .25:
            self._auto_armed = False
            self._permit.abort()
            self._reject_motion('state-stale')
            return
        if bool(self.get_parameter('manual_enabled').value):
            self._auto_armed = False
            self._permit.abort()
            self._reject_motion('manual-enabled')
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
                    not math.isfinite(msg.angular.z) or msg.linear.x < 0):
                self._core.halt()
                return
            auto_v, auto_w = scale_velocity_to_wheel_limit(
                msg.linear.x, msg.angular.z, self._core.track_m)
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
        if not command_source_allowed('manual', bool(self.get_parameter('manual_enabled').value),
                                      any(self._nav_active.values())):
            return
        if not self._sensors_ready(now):
            return
        if not manual_velocity(msg.linear.x, msg.angular.z, self._core.track_m):
            self._core.halt()
            self.get_logger().warn('Rejected out-of-range manual velocity',
                                   throttle_duration_sec=2.0)
            return
        try:
            if abs(msg.angular.z) > 1e-9:
                corrected_yaw_rate = self._manual_yaw_rate.update(
                    msg.angular.z, self._imu_yaw_rate, now)
                self._heading_hold.reset()
            else:
                self._manual_yaw_rate.reset()
                # 只前进不转向时，用 IMU 航向把车拉回直线（差速车左右轮不等径会画弧）。
                if (bool(self.get_parameter('heading_hold_enabled').value) and
                        self._imu_yaw is not None):
                    corrected_yaw_rate = self._heading_hold.update(
                        msg.linear.x, 0.0, self._imu_yaw, now)
                else:
                    self._heading_hold.reset()
                    corrected_yaw_rate = 0.0
            self._core.accept_cmd(msg.linear.x, corrected_yaw_rate, now)
            self._command_source = 'manual'
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

    def _sensors_ready(self, now: float) -> bool:
        return (bool(self.get_parameter('motion_enabled').value) and
                self._scan.ready(now) and self._scan_stamp.ready(now) and
                self._imu_stamp.ready(now) and self._last_imu_at is not None and
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

    def _on_nav_status(self, kind: str, msg: GoalStatusArray) -> None:
        self._nav_active[kind] = any(
            item.status in (1, 2, 3) for item in msg.status_list)

    def _turn_goal(self, request):
        now = time.monotonic()
        if (not self._sensors_ready(now) or any(self._nav_active.values()) or
                (request.mode == TurnRelative.Goal.PIVOT and
                 not bool(self.get_parameter('allow_pivot').value)) or
                self._imu_yaw is None or self._last_imu_at is None or
                self._imu_stamp.age_at(now) > .15):
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
        self._open_if_needed(now)
        if (self._fd is not None and self._core.fault and
                not self._auto_armed and self._safe_cmd_zero and
                now - self._last_auto_clear >= 2.0):
            # 门控已开、指令为零且未授权（电机空闲）时，自动清一次锁存的固件故障。
            # USB 插拔或编码器毛刺会留下 fault 1/3，否则面板会一直报“底盘未就绪”。
            try:
                os.write(self._fd, b'xc')
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
                    if (not self._core.verified and
                            now - self._last_handshake_attempt >= 1.0):
                        os.write(self._fd, b'?')
                        self._last_handshake_attempt = now
                    output = self._core.step(None, now)
                    if self._sensors_ready(now):
                        command = output.command
                    else:
                        print('[base] disarm tick-sensors', flush=True)
                        self._auto_armed = False
                        self._core.halt()
                        self._manual_yaw_rate.reset()
                        command = b'x'
                    os.write(self._fd, command)
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
