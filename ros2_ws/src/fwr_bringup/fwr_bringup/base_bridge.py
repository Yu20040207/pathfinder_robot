"""Sole ESP32-S3 serial owner for the physical chassis."""

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
from std_msgs.msg import Bool, Int32
from fwr_interfaces.action import TurnRelative

from .base_bridge_core import (BaseBridgeCore, ScanWatchdog, SensorStampGuard,
                               forward_only_velocity)
from .base_protocol import MAX_LINE, parse_state_line
from .base_serial import open_exclusive_serial
from .wheel_odometry_node import DEFAULT_DEVICE
from .turn_controller import TurnController, TurnFinished


class BaseBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('base_bridge')
        self.declare_parameter('device', DEFAULT_DEVICE)
        self.declare_parameter('motion_enabled', False)
        self.declare_parameter('allow_pivot', False)
        self._device = str(self.get_parameter('device').value)
        self._core = BaseBridgeCore()
        self._scan = ScanWatchdog()
        self._scan_stamp = SensorStampGuard()
        self._imu_stamp = SensorStampGuard()
        self._fd = None
        self._pending = bytearray()
        self._last_open_attempt = 0.0
        self._imu_yaw = None
        self._imu_yaw_rate = 0.0
        self._last_imu_at = None
        self._nav_active = {'pose': False, 'through': False}
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 lambda msg: self._on_nav_status('pose', msg),
                                 qos_profile_action_status_default)
        self.create_subscription(GoalStatusArray, '/navigate_through_poses/_action/status',
                                 lambda msg: self._on_nav_status('through', msg),
                                 qos_profile_action_status_default)
        self._wheel_odom = self.create_publisher(Odometry, '/wheel/odom', 10)
        self._joint_states = self.create_publisher(JointState, '/joint_states', 10)
        self._fault = self.create_publisher(Int32, '/base/fault', 10)
        self._ready = self.create_publisher(Bool, '/base/ready', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_velocity, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self._turn_action = ActionServer(
            self, TurnRelative, 'turn_relative',
            execute_callback=self._execute_turn,
            goal_callback=self._turn_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.create_timer(.05, self._tick)

    def _on_velocity(self, msg: Twist) -> None:
        if not self._sensors_ready(time.monotonic()):
            return
        try:
            if not forward_only_velocity(msg.linear.x, msg.angular.z, self._core.track_m):
                self._core.accept_cmd(0.0, 0.0, time.monotonic())
                return
            self._core.accept_cmd(msg.linear.x, msg.angular.z, time.monotonic())
        except (ValueError, RuntimeError) as exc:
            self.get_logger().warn(f'Rejected /cmd_vel: {exc}', throttle_duration_sec=2.0)

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
                0 <= now - self._last_imu_at <= .25)

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
            os.write(self._fd, b'x?')
            self.get_logger().info(f'ESP32 serial opened exclusively: {self._device}')
        except (OSError, ValueError) as exc:
            self.get_logger().warn(f'ESP32 serial unavailable: {exc}', throttle_duration_sec=5.0)

    def _disconnect(self, reason: str) -> None:
        if self._fd is not None:
            try:
                os.write(self._fd, b'x')
            except OSError:
                pass
            os.close(self._fd)
            self._fd = None
        self._core.disconnect()
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
            previous_esp_us = self._core._last_esp_us
            out = self._core.step(state, now)
            if previous_esp_us is not None and state.esp_us < previous_esp_us:
                if self._fd is not None:
                    os.write(self._fd, b'x?')
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
        self._open_if_needed(now)
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
                    output = self._core.step(None, now)
                    if self._sensors_ready(now):
                        command = output.command
                    else:
                        self._core.halt()
                        command = b'x'
                    os.write(self._fd, command)
            except OSError as exc:
                self._disconnect(str(exc))
        self._fault.publish(Int32(data=self._core.fault))
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
