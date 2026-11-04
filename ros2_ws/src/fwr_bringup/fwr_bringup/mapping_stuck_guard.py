"""建图卡住自动修复。

用户诉求（2026-11-04）：「自动建图卡住之后我要你修复这种情况，下次才不会出现」。

判定"卡住"（三个条件同时满足，持续 window_s）：
  1. 里程计几乎没动（< move_eps_m）
  2. 地图几乎没有新增已知格（< map_eps_cells）
  3. 建图本应在进行（/explore/state 处于 navigating/running，或没有该话题）

分级自愈（每次卡住只升一级，升级后等一个窗口再看）：
  第1级：清空全局/局部代价地图（治"幽灵障碍把自由区切碎"）
  第2级：重启探索循环（/explore/stop → /explore/start，强制重新选点）
  第3级：原地转一小段（调 Nav2 的 /spin 动作，走完整安全链）
  连续 self.max_escalations 次仍无效 → 打印明确结论并停手（不再空转）

安全：**不发任何前进/倒车指令**；第3级只旋转，且通过标准导航链路（受碰撞监控约束）。
"""
from __future__ import annotations

import math
import time

import rclpy
from nav2_msgs.action import Spin
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, String
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger


class MappingStuckGuard(Node):
    def __init__(self) -> None:
        super().__init__('mapping_stuck_guard')
        self.declare_parameter('enabled', True)
        self.declare_parameter('window_s', 45.0)        # 观察窗口
        self.declare_parameter('move_eps_m', 0.15)      # 窗口内位移小于此值 = 没动
        self.declare_parameter('map_eps_cells', 150)    # 窗口内新增已知格小于此值 = 地图没长
        self.declare_parameter('max_escalations', 3)
        self.declare_parameter('spin_angle_rad', 0.8)   # 第3级旋转角度（约 46°）
        self.declare_parameter('cooldown_s', 20.0)
        self.declare_parameter('max_fault_clear', 5)

        self._pose = None
        self._cells = None
        self._explore_state = ''
        self._win_at = time.monotonic()
        self._win_pose = None
        self._win_cells = None
        self._level = 0
        self._last_action_at = 0.0

        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, q)
        self.create_subscription(String, '/explore/state', self._on_state, 10)
        self._clear = [self.create_client(ClearEntireCostmap, s) for s in (
            '/global_costmap/global_costmap/clear_entirely_global_costmap',
            '/local_costmap/local_costmap/clear_entirely_local_costmap')]
        self._stop = self.create_client(Trigger, '/explore/stop')
        self._start = self.create_client(Trigger, '/explore/start')
        self._spin = ActionClient(self, Spin, '/spin')
        # 2026-11-04：窄走廊里四轮刮擦转向会被墙别住 → 固件判堵转(fault 1)锁死
        # → 底盘 disarm → ready=False → 一切导航停摆。这里负责自动复位。
        self._fault = 0
        self._ready = True
        self.create_subscription(Int32, '/base/fault',
                                 lambda m: setattr(self, '_fault', int(m.data)), 10)
        self.create_subscription(Bool, '/base/ready',
                                 lambda m: setattr(self, '_ready', bool(m.data)), 10)
        self._clear_fault = self.create_client(Trigger, '/base/clear_fault')
        self._params = self.create_client(SetParameters, '/base_bridge/set_parameters')
        self._fault_tries = 0
        self._fault_at = 0.0
        self._pub = self.create_publisher(String, '/mapping/stuck_guard', 10)
        # clear_fault 的前提之一是 explore_mode == 'stopped'；
        # 建图时 supervisor 一直在发 'navigating'，所以复位前必须先发 stopped
        self._state_pub = self.create_publisher(String, '/explore/state', 10)
        self.create_timer(2.0, self._tick)
        self.get_logger().info(
            '建图卡住守卫已启动（窗口 %.0f s，位移<%.2f m 且地图+<%d 格 → 逐级自愈）' % (
                self.get_parameter('window_s').value,
                self.get_parameter('move_eps_m').value,
                self.get_parameter('map_eps_cells').value))

    # ---------- 输入 ----------
    def _on_odom(self, msg: Odometry) -> None:
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._cells = sum(1 for v in msg.data if v >= 0)

    def _on_state(self, msg: String) -> None:
        self._explore_state = msg.data

    # ---------- 主循环 ----------
    def _tick(self) -> None:
        if not bool(self.get_parameter('enabled').value):
            return
        now = time.monotonic()
        if self._fault:
            self._recover_fault(now, self._fault)
            return
        if self._win_pose is None:
            self._win_pose = self._pose
            self._win_cells = self._cells
            self._win_at = now
            return
        if now - self._win_at < float(self.get_parameter('window_s').value):
            return

        moved = 0.0
        if self._pose and self._win_pose:
            moved = math.hypot(self._pose[0] - self._win_pose[0],
                               self._pose[1] - self._win_pose[1])
        grew = (self._cells or 0) - (self._win_cells or 0)
        self._win_pose, self._win_cells, self._win_at = self._pose, self._cells, now

        if not self._fault:
            self._fault_tries = 0
        if self._explore_state not in ('', 'navigating', 'running', 'starting'):
            self._level = 0                       # 建图没在跑，不算卡住
            return
        if moved >= float(self.get_parameter('move_eps_m').value):
            self._level = 0                       # 在动，正常
            return
        if grew >= int(self.get_parameter('map_eps_cells').value):
            self._level = 0                       # 地图在长，正常
            return

        self._recover(now, moved, grew)

    def _set_bool(self, name: str, value: bool) -> None:
        if not self._params.service_is_ready():
            return
        req = SetParameters.Request()
        msg = ParamMsg()
        msg.name = name
        msg.value.type = ParameterType.PARAMETER_BOOL
        msg.value.bool_value = value
        req.parameters.append(msg)
        self._params.call_async(req)

    def _recover_fault(self, now: float, code: int) -> None:
        """底盘故障锁死 → 关掉所有运动门 → 调 /base/clear_fault 复位。

        固件的 fault_clear 要求所有运动路径都关闭（motion/manual/auto 全为假），
        所以这里绝不发运动指令，只做"关门 + 复位"。
        """
        if now - self._fault_at < 5.0:
            return
        self._fault_at = now
        if self._fault_tries >= int(self.get_parameter('max_fault_clear').value):
            self.get_logger().error('底盘故障 %d 复位 %d 次仍失败，停止尝试' % (
                code, self._fault_tries))
            return
        self._fault_tries += 1
        names = {1: '堵转', 2: '方向错误', 3: '超速', 4: '编码器', 5: '时序', 6: '跟踪'}
        self.get_logger().warn('底盘故障 %d（%s）→ 关闭运动门并请求复位（第 %d 次）' % (
            code, names.get(code, '未知'), self._fault_tries))
        self._set_bool('motion_enabled', False)
        self._set_bool('manual_enabled', False)
        # 关键：把探索状态置为 stopped，并把探索循环停掉（clear_fault 的前置条件）
        for _ in range(3):
            self._state_pub.publish(String(data='stopped'))
        if self._stop.service_is_ready():
            self._stop.call_async(Trigger.Request())
        if self._clear_fault.service_is_ready():
            self._clear_fault.call_async(Trigger.Request())
        self._pub.publish(String(data='fault %d -> clear_fault' % code))

    def _recover(self, now: float, moved: float, grew: int) -> None:
        if now - self._last_action_at < float(self.get_parameter('cooldown_s').value):
            return
        self._last_action_at = now
        limit = int(self.get_parameter('max_escalations').value)
        if self._level >= limit:
            msg = '建图仍卡住（已自愈 %d 次，位移 %.2f m / 地图 %+d 格），不再空转' % (
                limit, moved, grew)
            self.get_logger().error(msg)
            self._pub.publish(String(data=msg))
            return
        self._level += 1
        if self._level == 1:
            asked = 0
            for c in self._clear:
                if c.service_is_ready():
                    c.call_async(ClearEntireCostmap.Request())
                    asked += 1
            msg = '卡住检测（位移 %.2f m / 地图 %+d 格）→ 第1级：清空代价地图（%d 个服务）' % (
                moved, grew, asked)
        elif self._level == 2:
            if self._stop.service_is_ready():
                self._stop.call_async(Trigger.Request())
            if self._start.service_is_ready():
                self._start.call_async(Trigger.Request())
            msg = '仍在卡住 → 第2级：重启探索循环（stop → start）'
        elif self._level == 3:
            angle = float(self.get_parameter('spin_angle_rad').value)
            if self._spin.server_is_ready():
                goal = Spin.Goal()
                goal.target_yaw = angle
                goal.time_allowance.sec = 10
                self._spin.send_goal_async(goal)
                msg = '仍在卡住 → 第3级：原地转 %.1f rad（走标准安全链）' % angle
            else:
                msg = '仍在卡住 → 第3级：/spin 不可用，跳过'
        else:
            msg = '仍在卡住 → 无更高一级手段'
        self.get_logger().warn(msg)
        self._pub.publish(String(data=msg))


def main() -> None:
    rclpy.init()
    node = MappingStuckGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
