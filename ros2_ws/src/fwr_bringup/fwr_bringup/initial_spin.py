"""自动建图开始前的原地自转：转满一圈，把四周墙面先扫进地图。

为什么需要：slam 从空图开始，车周围只有一条很窄的视锥已知；直接开始前沿
探索时，规划器几乎没有可用地形，容易"往未知里拱"。先慢速自转一圈，
能让 270° 视场的雷达把四周墙面都标出来。

安全设计：
- **只转不走**（linear.x 恒为 0），转速默认 0.30 rad/s（约 21 s 一圈）
- 走**手动通道** `/cmd_vel_manual`，不经过导航栈的规划
- 每帧检查正前方 ±25° 的最近距离，< `abort_m` 立即停车并放弃
- 累计转角达到 `sweep_rad` 才算完成；用 IMU/odom 的 yaw 而不是时间
- 完成后关闭手动模式，再由探索器接管
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class InitialSpin(Node):
    def __init__(self) -> None:
        super().__init__('initial_spin')
        self.declare_parameter('enabled', True)
        self.declare_parameter('rate_rad_s', 0.30)
        self.declare_parameter('sweep_rad', 2 * math.pi)
        self.declare_parameter('abort_m', 0.35)
        self.declare_parameter('max_duration_s', 40.0)
        self.declare_parameter('start_explorer_when_done', True)
        self._pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self._scan = None
        self._yaw = None
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        # 2026-11-03（用户思路）：转完就把控制权交给探索器 —— 用**事件**而不是
        # 固定延迟。做 transient_local，后启动的探索器也能立刻收到。
        from rclpy.qos import DurabilityPolicy, QoSProfile
        self._done_pub = self.create_publisher(
            Bool, '/mapping/spin_done',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._explore = self.create_client(Trigger, '/explore/start')
        self._params = self.create_client(SetParameters,
                                          '/base_bridge/set_parameters')
        self._gate_on = False

    def _set_bool(self, name: str, value: bool) -> None:
        if not self._params.service_is_ready():
            return
        p = Parameter()
        p.name = name
        p.value.type = ParameterType.PARAMETER_BOOL
        p.value.bool_value = value
        req = SetParameters.Request()
        req.parameters = [p]
        self._params.call_async(req)
        self._done = False
        self.create_timer(0.05, self._tick)

    def _on_scan(self, msg) -> None:
        self._scan = msg

    def _on_odom(self, msg) -> None:
        self._yaw = _yaw(msg.pose.pose.orientation)

    def _front_min(self):
        m = self._scan
        if m is None or not m.ranges:
            return None
        span = int(math.radians(25.0) / abs(m.angle_increment))
        mid = len(m.ranges) // 2          # 本车雷达：index 中部 = 车头
        vals = [r for r in m.ranges[max(0, mid - span):mid + span]
                if 0.05 < r < 3.5]
        return min(vals) if vals else None

    def _tick(self) -> None:
        if self._done or not bool(self.get_parameter('enabled').value):
            return
        if self._yaw is None:
            return                        # 等 odom
        if not self._gate_on:
            # 手动通道需要底盘运动门 + 手动模式；只在本节点转动期间打开
            self._set_bool('motion_enabled', True)
            self._set_bool('manual_enabled', True)
            # 2026-11-03（用户要求）：自转期间不用车尾超声。
            # 超声现在只在 state=='reversing' 时轮询，平时没数据 → 守卫
            # fail_safe_block=True 会被误判成"车尾被挡"而拦住动作。
            # 自转只转不走，后方保护没有意义，先关掉；转完立刻恢复。
            self._set_bool('rear_guard_enabled', False)
            self._gate_on = True
            return                        # 下一拍再开始计角度
        if not hasattr(self, '_last_yaw'):
            self._last_yaw = self._yaw
            self._swept = 0.0
            self._started_at = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info('开始自转（只转不走），转满一圈后交给探索器')
        # 2026-11-03 修 bug：原来用 |atan2(sin,cos)| 算"已转角"，它的值域是
        # 0~π，**永远到不了 2π**，所以一圈永远转不完，只能靠 40 s 超时收场。
        # 现在累加每一步的增量（取最短角差），与转向方向无关，能累积到 2π。
        delta = math.atan2(math.sin(self._yaw - self._last_yaw),
                           math.cos(self._yaw - self._last_yaw))
        self._last_yaw = self._yaw
        self._swept += abs(delta)
        swept = self._swept
        front = self._front_min()
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._started_at
        stop_reason = None
        if front is not None and front < float(self.get_parameter('abort_m').value):
            stop_reason = f'前方 {front:.2f} m 过近'
        elif elapsed > float(self.get_parameter('max_duration_s').value):
            stop_reason = f'超时 {elapsed:.0f}s'
        elif swept >= float(self.get_parameter('sweep_rad').value):
            stop_reason = f'转满 {math.degrees(swept):.0f}°'
        if stop_reason:
            self._pub.publish(Twist())    # 停
            self._done = True
            self._set_bool('manual_enabled', False)   # 交还自动通道
            self._set_bool('rear_guard_enabled', True)  # 恢复车尾守卫
            msg = Bool()
            msg.data = True
            self._done_pub.publish(msg)               # 通知探索器可以接管
            self.get_logger().info(f'自转结束（{stop_reason}）')
            if (bool(self.get_parameter('start_explorer_when_done').value)
                    and self._explore.service_is_ready()):
                self._explore.call_async(Trigger.Request())
                self.get_logger().info('已触发 /explore/start，交给前沿探索')
            return
        cmd = Twist()
        cmd.angular.z = float(self.get_parameter('rate_rad_s').value)
        self._pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = InitialSpin()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
