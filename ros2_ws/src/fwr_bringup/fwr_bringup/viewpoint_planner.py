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
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType
from rcl_interfaces.srv import SetParameters

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
        # 门控（与 auto_mapping_supervisor 用同一套服务/参数，不新造接口）
        self.declare_parameter('open_gate', True)      # False = 只算不发（纯调试）
        self.declare_parameter('gate_timeout_s', 60.0)  # 无活动多久自动关门
        # 地图连续多少秒没有新增已知格，才认为"真的探完了"
        self.declare_parameter('map_stable_s', 20.0)
        # 地图已知格少于这个数时，认为还没长开，需要"起步"把地图撑开
        # （350 m² 的场地约 130k 格；用它的一小部分做门槛）
        self.declare_parameter('bootstrap_max_cells', 4000)

        self._map: OccupancyGrid | None = None
        self._covered: np.ndarray | None = None
        self._visible_cache: np.ndarray | None = None
        self._cache_key = None
        self._visited: set[tuple[int, int]] = set()
        self._target: tuple[int, int] | None = None
        self._target_at = 0.0
        self._radius = float(self.get_parameter('radius_start_m').value)
        self._last_map_cells = -1
        self._map_stable_since = 0.0
        self._dwell_until = 0.0
        self._state = 'idle'

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, str(self.get_parameter('map_topic').value),
                                 self._on_map, qos)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # 2026-11-04 用户要求（方案A）：不再依赖 frontier_explorer 提供的服务，
        # 自己管门控 + 自己调 Nav2 动作，这样可以关掉旧探索器而互不影响。
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._param_client = self.create_client(SetParameters, '/base_bridge/set_parameters')
        self._arm_client = self.create_client(Trigger, '/base/arm_auto')
        self._ready = False
        self._manual = False
        self._running = True
        self.create_subscription(Bool, '/base/ready',
                                 lambda m: setattr(self, '_ready', bool(m.data)), 10)
        self.create_subscription(Bool, '/base/manual_enabled',
                                 lambda m: setattr(self, '_manual', bool(m.data)), 10)
        self.create_subscription(
            Bool, '/mapping/run', lambda m: setattr(self, '_running', bool(m.data)),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._goal_future = None
        self._goal_handle = None
        self._gate_open = False
        self._active_since = 0.0
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
        if not bool(self.get_parameter('enabled').value) or not self._running:
            if self._gate_open:
                self._cancel_active_goal()
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

        # 地图是否还在长？
        cells_now = int(free.sum())
        if cells_now != self._last_map_cells:
            self._last_map_cells = cells_now
            self._map_stable_since = now
        stable = (now - self._map_stable_since) >= float(
            self.get_parameter('map_stable_s').value)

        # ⚠️ 覆盖率不能单独当"完成"判据：刚启动时地图只有一小块，
        #    一个视点就"覆盖"了它的 100%（2026-11-04 实测）。
        #    真正的判据是：**没有前沿（门口）且地图不再增长**。
        has_frontier = bool(vc.frontier_cells(free, unknown))

        # 正在执行的目标
        if self._target is not None:
            d = math.hypot(cell[0] - self._target[0], cell[1] - self._target[1])
            arrive = float(self.get_parameter('arrive_m').value) / self._map.info.resolution
            if d <= arrive:
                self._dwell_until = now + float(self.get_parameter('dwell_s').value)
                self._target = None
                self._set_state('arrived')
                return
            if now - self._target_at > float(self.get_parameter('hop_timeout_s').value):
                self.get_logger().warn('视点 %.1f s 未完成，取消并放弃该点' %
                                       float(self.get_parameter('hop_timeout_s').value))
                self._visited.add(self._target)
                self._cancel_active_goal()
            else:
                self._set_state('going')
                return

        # 诊断：每 10 秒打一次内部数字（排查"不动"时非常有用）
        if now - getattr(self, '_dbg_at', 0.0) > 10.0:
            self._dbg_at = now
            step = max(1, int(round(float(self.get_parameter('candidate_step_m').value)
                                    / self._map.info.resolution)))
            self.get_logger().info(
                '诊断: 自由=%d 可行驶=%d 候选=%d 前沿=%s 半径=%.0f 已访问=%d' % (
                    int(free.sum()), int(drivable.sum()),
                    len(vc.candidate_viewpoints(drivable, step)),
                    has_frontier, self._radius, len(self._visited)))

        # ① 先找半径内值得去的视点
        picked = self._pick(cell, free, blocked, unknown, drivable)
        if picked is not None:
            self._visited.add(picked)
            self._dispatch(picked, now)
            return

        # ①b 起步：地图还很小（刚启动）时没有任何点"值得去"，
        #     但也不能就此判定完成 —— 先走到本半径内最远的可达点，把地图撑开。
        #     （原来这事由 auto_mapping_supervisor 的 kickstart 做，它靠倒车退回，
        #      用户明确说不需要倒车；这里改成纯前进+转弯。）
        boot = self._bootstrap_target(cell, drivable, free)
        if boot is not None:
            self._visited.add(boot)
            self._dispatch(boot, now, label='bootstrap')
            return

        # ② 半径内没有 → 逐级扩大
        if self._radius < float(self.get_parameter('radius_max_m').value):
            self._radius = min(self._radius * 2.0,
                               float(self.get_parameter('radius_max_m').value))
            self._set_state('expand-radius %.0f m' % self._radius)
            return

        # ③ 半径到顶 → 去门口扩展（目标落在已知区一侧）
        if has_frontier and self._goto_frontier(drivable, free, unknown, cell, now):
            self._radius = float(self.get_parameter('radius_start_m').value)
            return

        # ④ 真的没地方去了：只有地图也停止增长才算完成
        if stable:
            ratio = vc.coverage_ratio(self._covered.ravel(), free.ravel())
            self._set_state('done %.0f%%' % (100 * ratio))
            return
        self._set_state('waiting-map-growth')

    @staticmethod
    def _snap(cell, drivable):
        """把格子吸附到最近的可行驶格。

        ⚠️ 2026-11-04 实测：机器人所在格常常**不在**可行驶掩膜里（被腐蚀掉、
        或刚好处在地图边缘），此时 BFS 全返回 -1、所有候选都"不可达" →
        规划器完全不动作（车一直不动）。
        """
        if drivable[cell[0], cell[1]]:
            return cell
        h, w = drivable.shape
        ys, xs = np.nonzero(drivable)
        if ys.size == 0:
            return None
        i = int(np.argmin((ys - cell[0]) ** 2 + (xs - cell[1]) ** 2))
        return (int(ys[i]), int(xs[i]))

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
        # 地图还没长开时，自由区是窄条，按 0.25 m 腐蚀会一格不剩 → 没有候选 →
        # 不动 → 地图更不长（实测死锁）。逐步放宽内缩，保证总有点可去。
        for relaxed in (max(1, clearance // 2), 1, 0):
            if int(drivable.sum()) >= 200:
                break
            drivable = vc.erode(free, relaxed) if relaxed else free.copy()
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
            # 连候选都撒不出来：退一步用更细的网格再试
            cands = vc.candidate_viewpoints(drivable, 1)
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

    def _bootstrap_target(self, cell, drivable, free):
        """地图还没长开时，走到本半径内最远的可达点（只前进/转弯，不倒车）。"""
        if int(free.sum()) > int(self.get_parameter('bootstrap_max_cells').value):
            return None                       # 地图已经够大，交给正常选点
        res = self._map.info.resolution
        step = max(1, int(round(float(self.get_parameter('candidate_step_m').value) / res)))
        cands = vc.candidate_viewpoints(drivable, step)
        if not cands:
            # 连候选都撒不出来：退一步用更细的网格再试
            cands = vc.candidate_viewpoints(drivable, 1)
        if not cands:
            return None
        seed = self._snap(cell, drivable)
        if seed is None:
            return None
        dist = vc.bfs_distance(drivable, seed)
        limit = int(float(self.get_parameter('radius_start_m').value) / res)
        best, best_d = None, -1
        for c in cands:
            if c in self._visited:
                continue
            d = int(dist[c[0], c[1]])
            if 0 < d <= limit and d > best_d:
                best, best_d = c, d
        return best

    def _goto_frontier(self, drivable, free, unknown, cell, now) -> bool:
        """去门口扩展：目标必须落在已知区一侧（不能打进未知区）。"""
        fronts = vc.frontier_cells(free, unknown)
        if not fronts:
            return False
        seed = self._snap(cell, drivable)
        if seed is None:
            return False
        dist = vc.bfs_distance(drivable, seed)
        inset = max(1, int(round(float(self.get_parameter('inset_m').value)
                                 / self._map.info.resolution)))
        tgt = vc.expansion_target(fronts, drivable, dist, inset_cells=inset)
        if tgt is None:
            return False
        self._dispatch(tgt, now, label='frontier')
        return True

    # ---------- 门控（与 auto_mapping_supervisor 同一套接口）----------
    def _set_bool_param(self, name: str, value: bool) -> None:
        if not self._param_client.service_is_ready():
            return
        req = SetParameters.Request()
        msg = ParamMsg()
        msg.name = name
        msg.value.type = ParameterType.PARAMETER_BOOL
        msg.value.bool_value = value
        req.parameters.append(msg)
        self._param_client.call_async(req)

    def _open_gate(self) -> bool:
        """开运动门：motion_enabled=true → 等 ready → 申请 arm_auto。"""
        if not bool(self.get_parameter('open_gate').value):
            return True                      # 调试模式：不碰门控
        self._set_bool_param('motion_enabled', True)
        if not self._ready or self._manual:
            return False
        if not self._arm_client.service_is_ready():
            return False
        self._arm_client.call_async(Trigger.Request())
        self._gate_open = True
        return True

    def _close_gate(self) -> None:
        if not bool(self.get_parameter('open_gate').value):
            return
        self._set_bool_param('motion_enabled', False)
        self._gate_open = False

    def _dispatch(self, cell, now, label: str = 'viewpoint') -> None:
        m = self._map
        res = m.info.resolution
        x = m.info.origin.position.x + (cell[1] + 0.5) * res
        y = m.info.origin.position.y + (cell[0] + 0.5) * res
        self._target = cell
        self._target_at = now
        self._active_since = now
        if not self._open_gate():
            # kickstart 期间 manual_enabled=True，自动链路被门控清零，这里只需安静等待
            self._set_state('waiting-gate')
            self._target = None
            return
        if not self._nav_client.server_is_ready():
            self.get_logger().warn('导航动作服务器未就绪，暂不派目标')
            self._set_state('no-server')
            self._target = None
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        self.get_logger().info('%s → (%.2f, %.2f)' % (label, x, y))
        self._goal_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        self._goal_future.add_done_callback(self._on_goal_response)
        self._set_state('dispatch %s' % label)

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn('派目标失败: %s' % exc)
            self._target = None
            self._close_gate()
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn('目标被拒绝')
            self._target = None
            self._close_gate()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_feedback(self, _feedback) -> None:
        self._active_since = time.monotonic()

    def _on_goal_result(self, future) -> None:
        status = None
        try:
            status = future.result().status
        except Exception:                             # noqa: BLE001
            pass
        ok = status == 4                              # STATUS_SUCCEEDED
        self.get_logger().info('视点目标结束: %s' % ('成功' if ok else '失败'))
        self._goal_handle = None
        self._goal_future = None
        if ok:
            self._dwell_until = time.monotonic() + float(
                self.get_parameter('dwell_s').value)
        else:
            self._close_gate()                        # 失败立刻关门，等下一轮重开
        self._target = None

    def _cancel_active_goal(self) -> None:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:                         # noqa: BLE001
                pass
        self._goal_handle = None
        self._goal_future = None
        self._target = None
        self._close_gate()

    def _set_state(self, text: str) -> None:
        if text == self._state:
            return
        self._state = text
        self._state_pub.publish(String(data=text))
        # 也打到日志：状态话题是"变化才发"，晚连的观察者看不到历史
        self.get_logger().info('状态 → %s' % text)


def main() -> None:
    rclpy.init()
    node = ViewpointPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
