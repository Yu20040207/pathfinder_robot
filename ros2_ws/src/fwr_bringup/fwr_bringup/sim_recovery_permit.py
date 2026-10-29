"""Simulation-only permit service; never use as physical bridge."""
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger
from fwr_interfaces.srv import RecoveryPermit
from .base_bridge_core import RecoveryPermitCore, safe_command_gap_action


def select_sim_safe_command(cmd, mode, fresh, permit, now):
    safe = Twist()
    if not fresh:
        return safe
    if abs(cmd.linear.x) < 1e-9 and abs(cmd.angular.z) < 1e-9:
        return safe
    if mode == 'navigating' and cmd.linear.x >= 0:
        return cmd
    if (mode == 'reversing' and permit.accept_reverse(cmd.linear.x, now) and
            abs(cmd.angular.z) <= .15):
        return cmd
    return safe


class SimRecoveryPermit(Node):
    def __init__(self):
        super().__init__('sim_recovery_permit')
        self._core = RecoveryPermitCore()
        self._mode = 'stopped'
        self._mode_at = None
        self._motion_mode_started = None
        self._last_xy = None
        self._odom_at = None
        self._scan_at = None
        self._armed = False
        self._cmd = Twist()
        self._cmd_at = None
        self._output = self.create_publisher(Twist, '/cmd_vel_sim_enforced', 10)
        self._arm_pub = self.create_publisher(Bool, '/base/auto_armed', 10)
        self._budget_pub = self.create_publisher(Float64MultiArray, '/sim/recovery_budget', 10)
        self.create_subscription(Twist, '/cmd_vel_auto_safe', self._command, 10)
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(String, '/explore/state', self._state, 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_service(RecoveryPermit, '/base/recovery_permit', self._request)
        self.create_service(Trigger, '/base/arm_auto', self._arm)
        self.create_timer(.05, self._tick)

    def _state(self, msg):
        old_mode = self._mode
        self._mode = msg.data
        self._mode_at = time.monotonic()
        if self._mode != old_mode:
            self._motion_mode_started = (self._mode_at if self._mode in ('navigating', 'reversing')
                                         else None)
        if self._mode != 'reversing':
            self._core.abort()
        if self._mode == 'stopped' and old_mode != 'stopped':
            self._armed = False

    def _scan(self, _msg):
        self._scan_at = time.monotonic()

    def _odom(self, msg):
        self._odom_at = time.monotonic()
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self._last_xy and self._core.active:
            self._core.observe_distance(math.dist(xy, self._last_xy))
        self._last_xy = xy

    def _request(self, req, response):
        now = time.monotonic()
        ok, reason = self._core.request(req.enable, req.episode_id, now,
                    self._armed and self._mode == 'reversing' and self._mode_at is not None and now - self._mode_at < .25,
                    True, True)
        response.accepted, response.reason = ok, reason
        return response

    def _arm(self, _request, response):
        now = time.monotonic()
        self._armed = (self._mode == 'stopped' and
                       self._scan_at is not None and now - self._scan_at < .25 and
                       self._odom_at is not None and now - self._odom_at < .25)
        response.success = self._armed
        response.message = ('armed' if self._armed else
                            f'sim mode={self._mode}, scan_age={now - self._scan_at if self._scan_at else float("inf"):.3f}, '
                            f'odom_age={now - self._odom_at if self._odom_at else float("inf"):.3f}')
        return response

    def _command(self, msg):
        self._cmd = msg
        self._cmd_at = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        if (self._armed and
                (self._scan_at is None or now - self._scan_at > .25 or
                 self._odom_at is None or now - self._odom_at > .25 or
                 self._mode_at is None or now - self._mode_at > .25)):
            self.get_logger().warn(
                f'disarmed: sensor/state timeout scan={now-self._scan_at if self._scan_at else float("inf"):.3f} '
                f'odom={now-self._odom_at if self._odom_at else float("inf"):.3f} '
                f'state={now-self._mode_at if self._mode_at else float("inf"):.3f}')
            self._armed = False
            self._core.abort()
        if (self._armed and self._motion_mode_started is not None and
                now - self._motion_mode_started > .3 and
                safe_command_gap_action(
                    now - self._cmd_at if self._cmd_at is not None else float('inf'),
                    abs(self._cmd.linear.x) < 1e-9 and abs(self._cmd.angular.z) < 1e-9) == 'disarm'):
            self.get_logger().warn(
                f'disarmed: filtered command timeout cmd={now-self._cmd_at if self._cmd_at else float("inf"):.3f}')
            self._armed = False
            self._core.abort()
        fresh = (self._cmd_at is not None and now - self._cmd_at <= .15 and
                 self._mode_at is not None and now - self._mode_at <= .25)
        safe = select_sim_safe_command(self._cmd, self._mode, fresh and self._armed,
                                       self._core, now)
        self._output.publish(safe)
        self._arm_pub.publish(Bool(data=self._armed))
        self._budget_pub.publish(Float64MultiArray(data=[self._core.segment_m,
                                                      self._core.total_m,
                                                      float(self._core.segments),
                                                      float(self._core.active)]))


def main():
    rclpy.init()
    node = SimRecoveryPermit()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
