#!/usr/bin/env python3
"""房间级探索器：先把当前房间扫完整（前沿清空），再出门去下一个房间。

为什么需要它（2026-10-31 现场反馈）：
  全局前沿探索会"跳"着走 —— 刚进房间就发现别处有个更大的前沿，于是掉头出去，
  房间的墙只扫了一半，地图上留下没封闭的缺口。

做法：
  1. 把空闲区域"腐蚀" door_closing_m 米 —— 门洞（0.8~0.9 m 宽）会断开，
     房间被分成独立连通块
  2. 找出机器人所在的那一块 = 当前房间
  3. 在房间里找"空闲挨着未知"的前沿格；有就派导航目标（离机器人最近的）
  4. 房间里的前沿全部消灭后，才允许去房间外的前沿（出门）

接口：
  · 订阅 /map（transient_local）、TF map→base_footprint
  · 通过 Nav2 的 navigate_to_pose 发目标
  · 参数：door_closing_m / goal_min_m / goal_max_m / replan_s / use_room_gate
"""
import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Parameter as RclParameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.ndimage import binary_erosion, binary_dilation, label


class RoomExplorer(Node):
    def __init__(self) -> None:
        super().__init__('room_explorer')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('door_closing_m', 0.42)   # 腐蚀半径：门洞断开
        self.declare_parameter('goal_min_m', 0.8)        # 目标离机器人最近距离
        self.declare_parameter('goal_max_m', 12.0)       # 最远
        self.declare_parameter('replan_s', 3.0)          # 多久重新评估一次
        self.declare_parameter('finish_unknown_m2', 0.6)  # 房间剩余未知小于此值算扫完
        self.declare_parameter('goal_reached_m', 0.5)
        self.declare_parameter('use_room_gate', True)
        # 房间扫完却找不到前沿（房间在图上被封死）时：沿房间四角巡逻换角度重扫，
        # 往往能发现之前没扫到的门口。0 = 关闭该兜底行为。
        self.declare_parameter('patrol_on_stuck', True)
        # 2026-10-31 方案 A：同一目标连续失败 N 次后，用"手动通道"短退脱困。
        # 走手动通道是因为：手动后退只受车尾超声守卫约束（nav 通道还要恢复许可），
        # 而且这条通路已经实车验证过（有障停、无障走）。
        self.declare_parameter('escape_after_failures', 2)
        self.declare_parameter('escape_speed_mps', 0.08)
        # 2026-10-31 用户要求：卡住就退，**退到前面够转弯为止**，退多远由现场决定。
        # 停止条件（任一）：
        #   · 前方净空 >= escape_front_clear_m（够掉头/转弯）
        #   · 车尾超声守卫报警（后方近障，不能再退）
        #   · 退满 escape_max_m 或 escape_max_s（安全上限）
        self.declare_parameter('escape_front_clear_m', 0.85)
        self.declare_parameter('escape_max_m', 1.20)
        self.declare_parameter('escape_max_s', 8.0)    # 关掉 = 退化为全局探索

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self._map = None
        self._map_at = 0.0
        self.create_subscription(OccupancyGrid, str(self.get_parameter('map_topic').value),
                                 self._on_map, qos)
        self._buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._buf, self)
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_xy = None
        self._goal_sent_at = 0.0
        self._blacklist = {}     # (x,y) -> 失败次数
        self._odom = None
        self._front = float('inf')
        self._rear_blocked = False
        self._vel_pub = self.create_publisher(Twist, '/cmd_vel_manual', 20)
        self.create_subscription(Odometry, '/wheel/odom', self._on_odom, 20)
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self._on_scan, scan_qos)
        self.create_subscription(Bool, '/base/rear_blocked', self._on_rear_blocked, 10)
        self._param_cli = self.create_client(SetParameters, '/base_bridge/set_parameters')
        # 2026-10-31：发目标前先在全局代价地图上验证，避开"注定失败"的目标
        # （否则要浪费 2 次失败 + 一次脱困才发现它到不了）
        self._costmap = None
        cm_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._on_costmap, cm_qos)
        self._escape_until = 0.0
        self._escape_start = None
        self._escape_from = None
        self._escaping = False
        self._last_log = 0.0
        self.create_timer(float(self.get_parameter('replan_s').value), self._tick)
        self.get_logger().info('房间级探索器启动：先把当前房间扫完再出门')

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg
        self._map_at = self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, msg: LaserScan) -> None:
        """只取正前方 ±30° 的最小距离，用于判断"够不够转弯"。"""
        n = len(msg.ranges)
        if n == 0:
            return
        deg = n / 360.0
        vals = []
        for c in (180.0, -180.0):
            a0 = int((c - 30 + 180) * deg) % n
            a1 = int((c + 30 + 180) * deg) % n
            if a0 <= a1:
                idx = range(a0, a1)
            else:
                idx = list(range(a0, n)) + list(range(0, a1))
            for i in idx:
                r = msg.ranges[i]
                if math.isfinite(r) and r > 0.20:
                    vals.append(r)
        self._front = min(vals) if vals else float('inf')

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._costmap = msg

    def _goal_traversable(self, x: float, y: float, radius_cells: int = 2):
        """在全局代价地图上找目标附近可通行的点；找不到返回 None。

        代价 99/100 = 致命（内切/障碍），>=90 也视为不可去。
        """
        m = self._costmap
        if m is None:
            return (x, y)          # 还没代价地图，先放行
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        w, h = m.info.width, m.info.height
        data = np.array(m.data, dtype=np.int16).reshape(h, w)
        cx, cy = int((x - ox) / res), int((y - oy) / res)
        best = None
        best_d = 1e9
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < w and 0 <= ny < h):
                    continue
                if int(data[ny, nx]) >= 90:
                    continue
                d = dx * dx + dy * dy
                if d < best_d:
                    best_d = d
                    best = (ox + nx * res, oy + ny * res)
        return best

    def _on_rear_blocked(self, msg: Bool) -> None:
        self._rear_blocked = bool(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._odom = (p.x, p.y)

    def _set_manual(self, enabled: bool) -> None:
        if not self._param_cli.service_is_ready():
            return
        req = SetParameters.Request()
        for name, val in (('motion_enabled', True), ('manual_enabled', enabled),
                          ('manual_allow_mixed', True)):
            prm = RclParameter()
            prm.name = name
            prm.value.type = ParameterType.PARAMETER_BOOL
            prm.value.bool_value = val
            req.parameters.append(prm)
        self._param_cli.call_async(req)

    def _start_escape(self, now: float) -> None:
        if self._odom is None:
            return
        self._escaping = True
        self._escape_start = now
        self._escape_from = self._odom
        self._set_manual(True)
        self.get_logger().warn(
            f'连续失败 {int(self.get_parameter("escape_after_failures").value)} 次 → '
            f'短退脱困（≤{float(self.get_parameter("escape_max_m").value):.2f} m）')

    def _run_escape(self, now: float) -> bool:
        """执行短退；返回 True 表示还在退。"""
        if not self._escaping:
            return False
        moved = 0.0
        if self._odom is not None and self._escape_from is not None:
            moved = math.hypot(self._odom[0] - self._escape_from[0],
                               self._odom[1] - self._escape_from[1])
        front_ok = self._front >= float(self.get_parameter('escape_front_clear_m').value)
        done = (front_ok or self._rear_blocked or
                moved >= float(self.get_parameter('escape_max_m').value) or
                now - self._escape_start >= float(self.get_parameter('escape_max_s').value))
        if done:
            self._vel_pub.publish(Twist())
            self._set_manual(False)
            self._escaping = False
            self._escape_start = None
            why = ('前方已够转弯' if self._front >= float(
                self.get_parameter('escape_front_clear_m').value) else
                '车尾近障停退' if self._rear_blocked else '到达上限')
            self.get_logger().info(
                f'短退完成（{why}）：位移 {moved:.2f} m，前方净空 {self._front:.2f} m')
            return False
        tw = Twist()
        tw.linear.x = -abs(float(self.get_parameter('escape_speed_mps').value))
        self._vel_pub.publish(tw)
        return True

    # ---------- 工具 ----------
    def _robot_xy(self):
        try:
            tr = self._buf.lookup_transform('map', 'base_footprint',
                                            rclpy.time.Time(),
                                            timeout=rclpy.duration.Duration(seconds=0.2))
            return tr.transform.translation.x, tr.transform.translation.y
        except Exception:                               # noqa: BLE001
            return None

    def _send_goal(self, x: float, y: float) -> None:
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('navigate_to_pose 不在线')
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = rclpy.time.Time().to_msg()   # 0 = 最新，避免外推错误
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        fut = self._client.send_goal_async(goal)
        self._goal_xy = (x, y)
        self._goal_sent_at = self.get_clock().now().nanoseconds * 1e-9
        fut.add_done_callback(self._goal_done)

    def _goal_done(self, future) -> None:
        try:
            handle = future.result()
            if handle is not None and handle.accepted:
                handle.get_result_async().add_done_callback(self._result_done)
            else:
                self._goal_xy = None
        except Exception:                               # noqa: BLE001
            self._goal_xy = None

    def _result_done(self, future) -> None:
        # 记录失败次数：同一目标失败两次就拉黑，避免死循环重发
        try:
            res = future.result()
            ok = res is not None and res.status == 4
        except Exception:                               # noqa: BLE001
            ok = False
        if not ok and self._goal_xy is not None:
            key = (round(self._goal_xy[0], 1), round(self._goal_xy[1], 1))
            self._blacklist[key] = self._blacklist.get(key, 0) + 1
            if self._blacklist[key] >= 2:
                self.get_logger().warn(f'目标 {key} 连续失败，拉黑')
                if (self._blacklist[key] == int(
                        self.get_parameter('escape_after_failures').value)):
                    self._start_escape(self.get_clock().now().nanoseconds * 1e-9)
        self._goal_xy = None

    def _patrol_goal(self, free, room_mask, res, ox, oy, robot):
        """房间内找一个"离机器人最远、且没被拉黑/可通行"的角落当巡逻点。

        2026-10-31：原来只取最远的一个点，失败拉黑后仍会算出同一个点，
        形成"发目标→失败→拉黑→再发同一个"的死循环。现在按评分降序尝试多个候选。
        """
        cells = np.argwhere(free & room_mask)
        if len(cells) == 0:
            return None
        rx, ry = robot
        cy, cx = int((ry - oy) / res), int((rx - ox) / res)
        d2 = (cells[:, 0] - cy) ** 2 + (cells[:, 1] - cx) ** 2
        far = cells[d2 >= np.percentile(d2, 90)]
        if len(far) == 0:
            return None
        cen = cells.mean(axis=0)
        score = np.hypot(far[:, 0] - cen[0], far[:, 1] - cen[1])
        order = np.argsort(-score)
        tried = 0
        for idx in order:
            if tried >= 12:          # 最多试 12 个候选
                break
            tried += 1
            pick = far[int(idx)]
            gx, gy = ox + pick[1] * res, oy + pick[0] * res
            if math.hypot(gx - rx, gy - ry) < 0.8:
                continue
            key = (round(gx, 1), round(gy, 1))
            if self._blacklist.get(key, 0) >= 2:
                continue
            validated = self._goal_traversable(gx, gy)
            if validated is None:
                continue
            return validated
        return None

    # ---------- 主循环 ----------
    def _tick(self) -> None:
        m = self._map
        robot = self._robot_xy()
        if m is None or robot is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        # 2026-10-31：地图 5 秒没更新 = 建图已结束（按了停止按钮）或 Cartographer 挂了。
        # 这时继续发目标只会让 Nav2 原地打转，必须停手。
        if self._map_at > 0.0 and now - self._map_at > 5.0:
            if not getattr(self, '_frozen_logged', False):
                self.get_logger().warn('地图已停止更新（建图结束？），探索器暂停')
                self._frozen_logged = True
            self._goal_xy = None
            return
        self._frozen_logged = False
        # 脱困短退期间不派新目标
        if self._escaping:
            self._run_escape(now)      # 注意：这里必须真的调用（原来写成 or 短路了）
            return
        # 目标还没走完就等
        if self._goal_xy is not None:
            if math.hypot(self._goal_xy[0] - robot[0],
                          self._goal_xy[1] - robot[1]) < float(
                              self.get_parameter('goal_reached_m').value):
                self._goal_xy = None
            elif now - self._goal_sent_at > 60.0:
                self.get_logger().warn('目标 60 秒没到，换一个')
                self._goal_xy = None
            else:
                return

        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        w, h = m.info.width, m.info.height
        d = np.array(m.data, dtype=np.int16).reshape(h, w)
        free = d == 0
        if not free.any():
            return
        rx, ry = robot
        cxp, cyp = int((rx - ox) / res), int((ry - oy) / res)
        if not (0 <= cxp < w and 0 <= cyp < h):
            return

        # 1) 房间分割：腐蚀掉门洞宽度的一半 → 房间独立
        r = max(1, int(round(float(self.get_parameter('door_closing_m').value) / res)))
        core = binary_erosion(free, np.ones((2 * r + 1, 2 * r + 1)))
        lab, _ = label(core, structure=np.ones((3, 3)))
        room_id = lab[cyp, cxp]
        if room_id == 0:
            # 机器人不在任何"房间芯"里（在门口/窄处）→ 找最近的芯
            ys, xs = np.where(lab > 0)
            if len(ys) == 0:
                return
            idx = int(np.argmin((ys - cyp) ** 2 + (xs - cxp) ** 2))
            room_id = lab[ys[idx], xs[idx]]
        # 把房间芯膨胀回去（恢复房间的被腐蚀边界）
        room_mask = binary_dilation(lab == room_id, np.ones((2 * r + 1, 2 * r + 1)))

        # 2) 房间内的前沿：空闲且邻接未知
        unknown = d < 0
        pad = np.pad(unknown, 1, constant_values=False)
        nb_unknown = np.zeros_like(unknown)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                nb_unknown |= pad[1 + dy:1 + dy + h, 1 + dx:1 + dx + w]
        frontier = free & nb_unknown
        in_room = frontier & room_mask
        use_gate = bool(self.get_parameter('use_room_gate').value)

        cand = np.argwhere(in_room if (use_gate and in_room.any()) else frontier)
        free_area = float(free.sum()) * res * res
        if len(cand) == 0:
            # 地图刚起来时 free 很小、前沿为 0 是正常的，不能立刻判结束。
            if free_area < 5.0:
                self._empty_count = 0
                return
            self._empty_count = getattr(self, '_empty_count', 0) + 1
            if (self._empty_count >= 2
                    and bool(self.get_parameter('patrol_on_stuck').value)):
                # 房间里没有前沿（可能在图上被封死）→ 去房间角落换角度重扫
                goal = self._patrol_goal(free, room_mask, res, ox, oy, robot)
                if goal is not None and math.hypot(goal[0] - rx,
                                                   goal[1] - ry) > 0.8:
                    if now - self._last_log > 10.0:
                        self.get_logger().info(
                            f'房间内暂无前沿（空闲 {free_area:.0f} m²），'
                            f'巡逻换角度：( {goal[0]:.2f},{goal[1]:.2f})')
                        self._last_log = now
                    self._send_goal(*goal)
                    return
            if self._empty_count == 8 and not getattr(self, '_done_logged', False):
                self.get_logger().info(
                    f'连续 8 次没有前沿且巡逻无可去点（空闲 {free_area:.0f} m²），'
                    f'探索结束')
                self._done_logged = True
            return
        self._empty_count = 0
        self._done_logged = False

        # 3) 选目标：房间内离机器人最近、且距离合适的那一类
        dist = np.hypot((cand[:, 0] - cyp) * res, (cand[:, 1] - cxp) * res)
        lo = float(self.get_parameter('goal_min_m').value)
        hi = float(self.get_parameter('goal_max_m').value)
        sel = np.where((dist >= lo) & (dist <= hi))[0]
        if len(sel) == 0:
            sel = np.argsort(dist)[:20]
        # 房间内优先：选距离分位数 30% 的点，避免总奔最远
        order = sel[np.argsort(dist[sel])]
        idx = int(order[min(len(order) - 1, max(0, int(len(order) * 0.3)))])
        # 跳过被拉黑的目标
        tries = 0
        while tries < len(order):
            gy, gx = int(cand[idx][0]), int(cand[idx][1])
            key = (round(ox + gx * res, 1), round(oy + gy * res, 1))
            if self._blacklist.get(key, 0) < 2:
                break
            tries += 1
            idx = int(order[min(len(order) - 1, idx + 1)])
        gx_m, gy_m = ox + gx * res, oy + gy * res
        scope = '房间内' if (use_gate and in_room.any()) else '房间外'
        if now - self._last_log > 10.0:
            self.get_logger().info(
                f'{scope}前沿：目标 ({gx_m:.2f},{gy_m:.2f}) '
                f'距 {math.hypot(gx_m - rx, gy_m - ry):.1f} m，'
                f'房间内前沿 {int(in_room.sum())} 格 / 全局 {int(frontier.sum())} 格')
            self._last_log = now
        validated = self._goal_traversable(gx_m, gy_m)
        if validated is None:
            key = (round(gx_m, 1), round(gy_m, 1))
            self._blacklist[key] = 99
            if now - self._last_log > 5.0:
                self.get_logger().info(
                    f'前沿 ({gx_m:.1f},{gy_m:.1f}) 在代价地图上不可通行，直接跳过')
                self._last_log = now
            return
        self._send_goal(*validated)


def main():
    rclpy.init()
    node = RoomExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
