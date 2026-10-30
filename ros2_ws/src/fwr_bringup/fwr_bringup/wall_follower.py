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
        self.declare_parameter('target_dist_m', 0.55)
        self.declare_parameter('speed_mps', 0.12)        # 慢一点，扫描更稳
        self.declare_parameter('turn_rate_rad_s', 0.35)
        self.declare_parameter('front_stop_m', 0.60)
        self.declare_parameter('gain', 1.6)
        self.declare_parameter('max_time_s', 300.0)
        self.declare_parameter('loop_close_radius_m', 0.6)
        # 自动开门：本节点自己把底盘两道门打开（等价 supervisor 做的事）
        self.declare_parameter('open_gates', True)

        self._side = str(self.get_parameter('side').value)
        self._target = float(self.get_parameter('target_dist_m').value)
        self._speed = float(self.get_parameter('speed_mps').value)
        self._turn = float(self.get_parameter('turn_rate_rad_s').value)
        self._front_stop = float(self.get_parameter('front_stop_m').value)
        self._gain = float(self.get_parameter('gain').value)
        self._max_time = float(self.get_parameter('max_time_s').value)
        self._loop_r = float(self.get_parameter('loop_close_radius_m').value)

        self._front = float('inf')
        self._side_min = float('inf')
        self._start = None
        self._started_at = None
        self._turned = 0.0
        self._travelled = 0.0
        self._last_xy = None
        self._turn_started = None
        self._done = False

        q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self._on_scan, q)
        self.create_subscription(Odometry, '/wheel/odom', self._on_odom, 20)
        self._pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self._param_client = self.create_client(SetParameters, '/base_bridge/set_parameters')
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
        self._front = min(sector(180.0, 35.0), sector(-180.0, 35.0))
        self._side_min = sector(-90.0, 50.0) if self._side == 'left' else sector(90.0, 50.0)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        if self._last_xy is not None:
            self._travelled += math.hypot(p.x - self._last_xy[0],
                                          p.y - self._last_xy[1])
        self._last_xy = (p.x, p.y)
        if self._start is None:
            self._start = (p.x, p.y)
            return
        moved = math.hypot(p.x - self._start[0], p.y - self._start[1])
        elapsed = (self.get_clock().now().nanoseconds * 1e-9) - (self._started_at or 0.0)
        # 闭环：走回起点附近且已经离开过一段时间
        # 闭环必须建立在"确实走过一圈"之上：累计路程够长 + 回到起点 + 转够了方向
        if (not self._done and moved < self._loop_r and
                self._travelled > 6.0 and self._turned > 8.0 and elapsed > 30.0):
            self.get_logger().info(
                f'沿墙建图：回到起点附近（走了 {self._travelled:.1f} m），闭环完成')
            self._done = True

    # ---------- 控制 ----------
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
        if self._started_at is None:
            self._started_at = now
        twist = Twist()
        if self._done or now - self._started_at > self._max_time:
            self._pub.publish(twist)
            if not self._done:
                self.get_logger().info('沿墙建图：到达时间上限，停止')
                self._done = True
            return

        if self._front < self._front_stop:
            # 前方堵：先原地转向（朝远离参考墙的一侧）
            if self._turn_started is None:
                self._turn_started = now
            st = now - self._turn_started
            sign = 1.0 if self._side == 'right' else -1.0
            if st > 8.0:
                # 转了 8 秒还没转开（墙角）：倒车 1 秒脱困
                twist.linear.x = -self._speed
                self._pub.publish(twist)
                if st > 9.0:
                    self._turn_started = None
                return
            twist.angular.z = sign * self._turn
            self._turned += abs(twist.angular.z) * 0.05
            self._pub.publish(twist)
            return
        self._turn_started = None

        if not math.isfinite(self._side_min):
            # 那一侧完全看不到墙（门口/开阔）：直行找墙
            twist.linear.x = self._speed
            self._pub.publish(twist)
            return

        # 保持距离：远了朝墙转，近了背墙转
        err = self._side_min - self._target          # >0 离墙太远
        corr = self._gain * err
        corr = max(-self._turn, min(self._turn, corr))
        # 贴右侧走：离墙远（err>0）需要右转（负角速度）；贴左侧反之
        twist.angular.z = (-corr if self._side == 'right' else corr)
        # 前进速度随前方余量降低，转弯时减速
        slow = max(0.4, min(1.0, (self._front - self._front_stop) / 0.8))
        twist.linear.x = self._speed * slow * max(0.5, 1.0 - abs(twist.angular.z))
        self._turned += abs(twist.angular.z) * 0.05
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
