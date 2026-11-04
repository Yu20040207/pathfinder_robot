"""自动建图的视点规划器：用最少的站位把整层"看"完。

与现有导航的关系：
- **只调用** 现成的 `/explore/start_fixed_goal` 服务派目标（单点导航，已实车验证）
- 不修改任何导航参数/行为树/安全链；`enabled` 默认 **false**，不开开关 = 行为完全不变

策略（详见 docs/plans/2026-11-04-auto-mapping-viewpoint-plan.md）：
  1. 半径受限：只在 radius 米内选视点 → 天然"这间房没看完不出门"
  2. 选"新增可见格 / 路径长度"最高的点（栅格 BFS 真实距离，不穿墙）
  3. 半径内没有 → 扩大半径 → 仍没有 → 去前沿（门口）扩展
  4. 到达视点后停留 dwell 秒收集扫描，然后继续
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from fwr_interfaces.srv import StartFixedGoal

from . import viewpoint_core as vc


class ViewpointPlanner(Node):
    def __init__(self) -> None:
        super().__init__('viewpoint_planner')
        # ---- 开关（默认关：不开则完全不参与建图）----
        self.declare_parameter('enabled', False)
        # ---- 几何 ----
        self.declare_parameter('sensor_range_m', 6.0)
        self.declare_parameter('rays', 180)
        self.declare_parameter('fill_cells', 2)
        self.declare_parameter('candidate_step_m', 1.0)
        self.declare_parameter('clearance_m', 0.25)
        self.declare_parameter('inset_m', 0.40)
        # ---- 半径策略 ----
        self.declare_parameter('radius_start_m', 4.0)
        self.declare_parameter('radius_max_m', 8.0)
        self.declare_parameter('cover_target', 0.90)
        self.declare_parameter('min_gain_cells', 400)
        # ---- 节奏 ----
        self.declare_parameter('dwell_s', 3.0)
        self.declare_parameter('hop_timeout_s', 45.0)
        self.declare_parameter('arrive_m', 0.45)
        self.declare_parameter('replan_period_s', 1.0)
        self.declare_parameter('map_topic', '/map')

        self._map: OccupancyGrid | None = None
        self._covered: np.ndarray | None = None
        self._visible_cache: np.ndarray | None = None
        self._cache_key = None
        self._visited: set[tuple[int, int]] = set()
        self._target: tuple[int, int] | None = None
        self._target_at = 0.0
        self._radius = float(self.get_parameter('radius_start_m').value)
        self._dwell_until = 0.0
        self._state = 'idle'

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, str(self.get_parameter('map_topic').value),
                                 self._on_map, qos)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._goal_client = self.create_client(StartFixedGoal, '/explore/start_fixed_goal')
        self._state_pub = self.create_publisher(String, '/viewpoint_planner/state', 10)
        period = float(self.get_parameter('replan_period_s').value)
        self.create_timer(max(0.2, period), self._tick)
        self.get_logger().info(
            '视点规划器已启动（enabled=%s；半径 %.1f→%.1f m；目标覆盖 %.0f%%）' % (
                self.get_parameter('enabled').value,
                self.get_parameter('radius_start_m').value,
                self.get_parameter('radius_max_m').value,
                100 * float(self.get_parameter('cover_target').value)))

    # ---------- 输入 ----------
    def _on_map(self, msg: OccupancyGrid) -> None:
        if self._map is None or (msg.info.width, msg.info.height) != (
                self._map.info.width, self._map.info.height):
            self._covered = None
            self._cache_key = None
        self._map = msg

    def _robot_cell(self):
        try:
            tf = self._tf_buffer.lookup_transform('map', 'base_footprint',
                                                 rclpy.time.Time())
        except Exception:
            return None, None
        m = self._map
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        col = int((x - m.info.origin.position.x) / m.info.resolution)
        row = int((y - m.info.origin.position.y) / m.info.resolution)
        return (row, col), (x, y)

    # ---------- 主循环 ----------
    def _tick(self) -> None:
        if not bool(self.get_parameter('enabled').value):
            return
        if self._map is None:
            self._set_state('waiting-map')
            return
        cell, _ = self._robot_cell()
        if cell is None:
            self._set_state('waiting-tf')
            return
        now = time.monotonic()
        if now < self._dwell_until:          # 到点停留收集扫描
            self._accumulate_visibility(cell)
            self._set_state('dwell')
            return

        grids = self._grids()
        if grids is None:
            self._set_state('waiting-map')
            return
        free, blocked, unknown, drivable = grids
        if self._covered is None or self._covered.shape != free.shape:
            self._covered = np.zeros(free.shape, dtype=bool)
        self._accumulate_visibility(cell)

        ratio = vc.coverage_ratio(self._covered.ravel(), free.ravel())
        if ratio >= float(self.get_parameter('cover_target').value):
            if self._goto_frontier(drivable, free, unknown, cell, now):
                return
            self._set_state('done %.0f%%' % (100 * ratio))
            return

        if self._target is not None:
            d = math.hypot(cell[0] - self._target[0], cell[1] - self._target[1])
            arrive = float(self.get_parameter('arrive_m').value) / self._map.info.resolution
            if d <= arrive:
                self._dwell_until = now + float(self.get_parameter('dwell_s').value)
                self._target = None
                self._set_state('arrived')
                return
            if now - self._target_at > float(self.get_parameter('hop_timeout_s').value):
                self.get_logger().warn('视点 %.1f s 未到达，放弃该点' %
                                       float(self.get_parameter('hop_timeout_s').value))
                self._visited.add(self._target)
                self._target = None
            else:
                self._set_state('going')
                return

        picked = self._pick(cell, free, blocked, unknown, drivable)
        if picked is None:
            if self._radius < float(self.get_parameter('radius_max_m').value):
                self._radius = min(self._radius * 2.0,
                                   float(self.get_parameter('radius_max_m').value))
                self.get_logger().info('半径内无可选视点 → 半径扩大到 %.1f m' % self._radius)
                self._set_state('expand-radius')
                return
            if self._goto_frontier(drivable, free, unknown, cell, now):
                return
            self._set_state('done %.0f%%' % (100 * ratio))
            return
        self._visited.add(picked)
        self._dispatch(picked, now)

    # ---------- 内部 ----------
    def _grids(self):
        m = self._map
        data = np.asarray(m.data, dtype=np.int16)
        w, h = m.info.width, m.info.height
        if data.size != w * h:
            return None
        free = vc.free_mask(data, w, h)
        blocked = vc.blocked_mask(data, w, h)
        unknown = vc.unknown_mask(data, w, h)
        clearance = int(round(float(self.get_parameter('clearance_m').value) / m.info.resolution))
        drivable = vc.erode(free, max(1, clearance))
        return free, blocked, unknown, drivable

    def _accumulate_visibility(self, cell) -> None:
        grids = self._grids()
        if grids is None:
            return
        free, blocked, unknown, _ = grids
        if self._covered is None or self._covered.shape != free.shape:
            self._covered = np.zeros(free.shape, dtype=bool)
        key = (cell, float(self.get_parameter('sensor_range_m').value),
               int(self.get_parameter('rays').value))
        if self._cache_key == key and self._visible_cache is not None:
            seen = self._visible_cache
        else:
            range_cells = int(float(self.get_parameter('sensor_range_m').value)
                              / self._map.info.resolution)
            seen = vc.visible_cells(blocked, unknown, cell, range_cells,
                                    int(self.get_parameter('rays').value),
                                    int(self.get_parameter('fill_cells').value))
            self._cache_key, self._visible_cache = key, seen
        self._covered.ravel()[seen] = True

    def _pick(self, cell, free, blocked, unknown, drivable):
        m = self._map
        res = m.info.resolution
        step = max(1, int(round(float(self.get_parameter('candidate_step_m').value) / res)))
        cands = vc.candidate_viewpoints(drivable, step)
        if not cands:
            return None
        dist = vc.bfs_distance(drivable, cell)
        range_cells = int(float(self.get_parameter('sensor_range_m').value) / res)
        fill = int(self.get_parameter('fill_cells').value)
        vis = [vc.visible_cells(blocked, unknown, c, range_cells,
                                int(self.get_parameter('rays').value), fill)
               for c in cands]
        radius_cells = int(self._radius / res)
        idx = vc.pick_next_viewpoint(
            cands, vis, self._covered.ravel(), dist, radius_cells,
            visited=self._visited,
            min_gain_cells=int(self.get_parameter('min_gain_cells').value))
        return None if idx is None else cands[idx]

    def _goto_frontier(self, drivable, free, unknown, cell, now) -> bool:
        """去门口扩展：目标必须落在已知区一侧（不能打进未知区）。"""
        fronts = vc.frontier_cells(free, unknown)
        if not fronts:
            return False
        dist = vc.bfs_distance(drivable, cell)
        inset = max(1, int(round(float(self.get_parameter('inset_m').value)
                                 / self._map.info.resolution)))
        tgt = vc.expansion_target(fronts, drivable, dist, inset_cells=inset)
        if tgt is None:
            return False
        self._dispatch(tgt, now, label='frontier')
        return True

    def _dispatch(self, cell, now, label: str = 'viewpoint') -> None:
        m = self._map
        res = m.info.resolution
        x = m.info.origin.position.x + (cell[1] + 0.5) * res
        y = m.info.origin.position.y + (cell[0] + 0.5) * res
        self._target = cell
        self._target_at = now
        if not self._goal_client.service_is_ready():
            self.get_logger().warn('单点导航服务未就绪，暂不派目标')
            self._set_state('no-service')
            return
        req = StartFixedGoal.Request()
        req.x = float(x)
        req.y = float(y)
        req.yaw = 0.0
        self.get_logger().info('%s → (%.2f, %.2f)' % (label, x, y))
        self._goal_client.call_async(req)
        self._set_state('dispatch %s' % label)

    def _set_state(self, text: str) -> None:
        if text == self._state:
            return
        self._state = text
        self._state_pub.publish(String(data=text))


def main() -> None:
    rclpy.init()
    node = ViewpointPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
