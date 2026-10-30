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
        self.declare_parameter('target_dist_m', 0.55)
        self.declare_parameter('speed_mps', 0.12)        # 慢一点，扫描更稳
        self.declare_parameter('turn_rate_rad_s', 0.35)
        self.declare_parameter('front_stop_m', 0.60)
        self.declare_parameter('gain', 1.6)
        self.declare_parameter('max_time_s', 300.0)
        self.declare_parameter('loop_close_radius_m', 0.6)
        # 自动开门：本节点自己把底盘两道门打开（等价 supervisor 做的事）
        self.declare_parameter('open_gates', True)
        # 2026-10-30 平滑化：不再"停了再转"，改成边走边绕 + 加减速限幅
        self.declare_parameter('avoid_start_m', 1.10)   # 前方小于这个距离开始绕
        # 0.50：雷达装在前部，读数 0.50 意味着车头边缘距障碍约 0.41 m；
        # 太小会顶到墙 → 轮子打滑 → 固件失速自停。
        self.declare_parameter('emergency_m', 0.50)     # 小于这个距离才急停
        self.declare_parameter('gap_gain', 1.4)         # 间隙转向权重
        self.declare_parameter('pivot_m', 0.38)         # 只有小于这个距离才原地转
        self.declare_parameter('turn_commit_s', 1.5)    # 转向方向锁定时间（防左右抖）
        self.declare_parameter('arc_min_speed_ratio', 0.35)  # 弧线转弯时最低速度比例
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
        self._start = None
        self._started_at = None
        self._turned = 0.0
        self._travelled = 0.0
        self._last_xy = None
        self._turn_started = None
        self._done = False
        self._v_cmd = 0.0
        self._w_cmd = 0.0
        self._last_tick = None
        self._turn_dir = 0.0
        self._turn_until = 0.0

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
        self._front = min(sector(180.0, 20.0), sector(-180.0, 20.0))
        self._front_left = sector(-150.0, 25.0)     # 车头左侧 30° 左右
        self._front_right = sector(150.0, 25.0)     # 车头右侧
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
        if self._recover_stage == 4 and st > 4.2:
            # 倒车脱困
            twist = Twist()
            twist.linear.x = -self._speed
            self._pub.publish(twist)
            return
        if self._recover_stage == 4 and st > 5.2:
            self._pub.publish(Twist())
            self._recover_stage = 0
            self._fault = 0
            self._v_cmd = self._w_cmd = 0.0
            self.get_logger().info(
                f'复位完成（累计 {self._recover_count} 次），继续沿墙')
            if self._recover_count >= 3:
                self.get_logger().warn('已连续复位 3 次，若再次失速建议人工检查环境')
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

        # ① 沿墙保持项
        omega_wall = 0.0
        if math.isfinite(self._side_min):
            omega_wall = side_sign * self._gain * (self._side_min - self._target)
        omega_wall = max(-self._turn * 0.6, min(self._turn * 0.6, omega_wall))

        lf = self._front_left if math.isfinite(self._front_left) else 3.0
        rf = self._front_right if math.isfinite(self._front_right) else 3.0

        if self._front <= pivot_m:
            # ② 贴脸：原地转（必须），方向沿用已锁定方向
            if self._turn_dir == 0.0:
                self._turn_dir = 1.0 if lf >= rf else -1.0
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
