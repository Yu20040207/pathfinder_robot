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
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class AutoMappingSupervisor(Node):
    def __init__(self):
        super().__init__('auto_mapping_supervisor')
        self.declare_parameter('kick_speed_mps', 0.15)
        self.declare_parameter('kick_turn_rad_s', 0.7)
        self.declare_parameter('startup_delay_s', 15.0)
        self.declare_parameter('front_block_m', 0.7)
        self.declare_parameter('front_clear_m', 1.2)
        self.declare_parameter('legs', 4)
        self.declare_parameter('leg_max_s', 8.0)
        self._cmd_manual = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self._state_pub = self.create_publisher(String, '/explore/state', 10)
        self._pause_pub = self.create_publisher(Bool, '/explore/pause', 10)
        self._param_client = self.create_client(SetParameters, '/base_bridge/set_parameters')
        self._arm_client = self.create_client(Trigger, '/base/arm_auto')
        self._phase = 'waiting'
        self._phase_at = time.monotonic()
        self._leg_at = 0.0
        self._legs_done = 0
        self._last_set_try = 0.0
        self._ready_ok = False
        self._manual_ok = False
        self.create_subscription(Bool, '/base/ready',
                                 lambda m: setattr(self, '_ready_ok', m.data), 10)
        self.create_subscription(Bool, '/base/manual_enabled',
                                 lambda m: setattr(self, '_manual_ok', m.data), 10)
        self._front_min = float('inf')
        self._left_min = float('inf')
        self._right_min = float('inf')
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self._on_scan, scan_qos)
        self.create_timer(0.05, self._tick)

    # ---------- 激光分扇区 ----------
    # 雷达倒装 180°：scan 角度范围 [-180°,+180°]，index 0 = -180°（车头方向），
    # index mid = 0°（车尾方向）。因此：
    #   车头 = 数组两端各 40°；车左 = 角度 -90°(index n/4)；车右 = 角度 +90°(index 3n/4)。
    def _on_scan(self, msg):
        n = len(msg.ranges)
        deg = n / 360.0  # 每度对应索引数

        def _min(indices):
            finite = [msg.ranges[i] for i in indices
                      if 0 <= i < n and math.isfinite(msg.ranges[i]) and msg.ranges[i] > 0.01]
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
                    self.get_logger().info('kickstart: gates verified open')
                    self._phase = 'kick_orient'
                    self._phase_at = now
                    self._leg_at = now
            return
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
        if self._phase == 'kick_back':
            # 三面都堵（墙角/贴墙）：倒车 0.5 m 脱困再重新判断
            if now - self._leg_at < 3.5:
                twist = Twist()
                twist.linear.x = -0.15
                self._cmd_manual.publish(twist)
                return
            self._cmd_manual.publish(Twist())
            self.get_logger().info('kickstart: backed out, re-orienting')
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
