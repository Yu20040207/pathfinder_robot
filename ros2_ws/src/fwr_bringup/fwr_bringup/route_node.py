#!/usr/bin/env python3
"""录制/复现人工驾驶路线（learning from demonstration 的实用版）。

两种模式：
  record  人用键盘开一圈（含出门/进门），每走 0.30 m 记一个路点，
          存到 ~/.fwr_routes/<name>.yaml（含 odom 位姿 + 朝向）
  follow  按录好的路点自动行驶（纯追踪），到达终点或超时停止

为什么比反应式沿墙可靠：
  进出门洞这种「一次判错就卡住」的地方，人开一遍就把正确走法记下来了，
  机器照着重放即可，不需要在现场识别门框。

安全：
  · 只发布 /cmd_vel_manual，底盘门控、失速复位照旧
  · 复现时若前方 <0.5 m 有障碍且持续 3 s，暂停并报错（不会硬撞）
"""
import math
import os

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter as RclParameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

SELF_MIN = 0.20


def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RouteNode(Node):
    def __init__(self) -> None:
        super().__init__('route_node')
        self.declare_parameter('mode', 'record')          # record | follow
        self.declare_parameter('name', 'route1')
        self.declare_parameter('spacing_m', 0.30)
        self.declare_parameter('speed_mps', 0.10)
        self.declare_parameter('lookahead_m', 0.55)
        self.declare_parameter('max_speed_mps', 0.14)
        self.declare_parameter('goal_tolerance_m', 0.25)
        self.declare_parameter('open_gates', True)
        self._mode = str(self.get_parameter('mode').value)
        self._path_file = os.path.expanduser(
            f"~/.fwr_routes/{self.get_parameter('name').value}.yaml")
        self._spacing = float(self.get_parameter('spacing_m').value)
        self._speed = float(self.get_parameter('speed_mps').value)
        self._lookahead = float(self.get_parameter('lookahead_m').value)

        self._pose = None            # (x, y, yaw) in odom
        self._points = []            # [(x, y, yaw)]
        self._front = float('inf')
        self._gates_at = None
        self._started = False
        self._blocked_at = None

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/wheel/odom', self._on_odom, 20)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos)
        self._pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self._path_pub = self.create_publisher(Path, '/recorded_route', 5)
        self._param = self.create_client(SetParameters, '/base_bridge/set_parameters')
        self.create_timer(0.05, self._tick)

        if self._mode == 'follow':
            self._load_route()
            self.get_logger().info(
                f'复现模式：载入 {len(self._points)} 个路点 ← {self._path_file}')
        else:
            self.get_logger().info(
                f'录制模式：键盘开一圈，每 {self._spacing:.2f} m 记一个点，'
                f'Ctrl+C 结束并保存到 {self._path_file}')

    # ---------- 输入 ----------
    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pose = (p.x, p.y, quat_to_yaw(msg.pose.pose.orientation))
        if self._mode == 'record' and self._pose is not None:
            if not self._points or math.hypot(p.x - self._points[-1][0],
                                              p.y - self._points[-1][1]) >= self._spacing:
                self._points.append(self._pose)
                self._publish_route()

    def _on_scan(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        deg = n / 360.0
        a0 = int((180 - 25 + 180) * deg) % n
        a1 = int((-180 + 25 + 180) * deg)
        vals = [msg.ranges[i] for i in range(min(a1, n))
                if math.isfinite(msg.ranges[i]) and msg.ranges[i] > SELF_MIN]
        vals += [msg.ranges[i] for i in range(a0, n)
                 if math.isfinite(msg.ranges[i]) and msg.ranges[i] > SELF_MIN]
        self._front = min(vals) if vals else float('inf')

    # ---------- 路线读写 ----------
    def _load_route(self) -> None:
        if not os.path.exists(self._path_file):
            self.get_logger().error(f'找不到路线文件 {self._path_file}')
            return
        data = yaml.safe_load(open(self._path_file))
        self._points = [(float(p['x']), float(p['y']), float(p['yaw']))
                        for p in data['points']]
        self._publish_route()

    def _save_route(self) -> None:
        if not self._points:
            self.get_logger().warn('没有记录到路点，不保存')
            return
        os.makedirs(os.path.dirname(self._path_file), exist_ok=True)
        yaml.safe_dump(
            {'name': self.get_parameter('name').value,
             'spacing_m': self._spacing,
             'points': [{'x': p[0], 'y': p[1], 'yaw': p[2]} for p in self._points]},
            open(self._path_file, 'w'))
        self.get_logger().info(f'已保存 {len(self._points)} 个路点到 {self._path_file}')

    def _publish_route(self) -> None:
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        for (x, y, yaw) in self._points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)
        self._path_pub.publish(path)

    # ---------- 控制 ----------
    def _set_gates(self, enabled: bool) -> None:
        if not self._param.service_is_ready():
            return
        req = SetParameters.Request()
        for name in ('motion_enabled', 'manual_enabled', 'manual_allow_mixed'):
            p = RclParameter()
            p.name = name
            p.value.type = ParameterType.PARAMETER_BOOL
            p.value.bool_value = enabled
            req.parameters.append(p)
        self._param.call_async(req)

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._gates_at is None:
            self._gates_at = now
        if (bool(self.get_parameter('open_gates').value) and not self._started and
                2.0 < now - self._gates_at < 2.2):
            self._set_gates(True)
            self._started = True

        if self._mode != 'follow' or self._pose is None or not self._points:
            return

        # 纯追踪：找当前最近路点，再往前看 lookahead
        x, y, yaw = self._pose
        nearest = min(range(len(self._points)),
                      key=lambda i: math.hypot(self._points[i][0] - x,
                                               self._points[i][1] - y))
        target = self._points[min(len(self._points) - 1, nearest + 3)]
        dx, dy = target[0] - x, target[1] - y
        if math.hypot(dx, dy) < float(self.get_parameter('goal_tolerance_m').value) \
                and nearest >= len(self._points) - 2:
            self.get_logger().info('到达路线终点，停止')
            self._pub.publish(Twist())
            self._points = []
            return

        err = math.atan2(dy, dx) - yaw
        err = math.atan2(math.sin(err), math.cos(err))
        twist = Twist()
        twist.angular.z = max(-1.0, min(1.0, 2.0 * err))
        speed = float(self.get_parameter('speed_mps').value)
        if abs(err) > 0.6:
            speed *= 0.5
        if self._front < 0.5:
            speed = 0.0
            if self._blocked_at is None:
                self._blocked_at = now
                self.get_logger().warn(f'复现时前方 {self._front:.2f} m 有障碍，暂停')
            elif now - self._blocked_at > 3.0:
                self.get_logger().error('前方持续被挡，停止复现，需要人工处理')
                self._points = []
                self._pub.publish(Twist())
                return
        else:
            self._blocked_at = None
        twist.linear.x = speed
        self._pub.publish(twist)


def main():
    rclpy.init()
    node = RouteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._pub.publish(Twist())
        if node._mode == 'record':
            node._save_route()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
