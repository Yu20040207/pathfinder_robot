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
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
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
        # 最短目标距离：小于它就不派（Nav2 的 xy_goal_tolerance 是 0.4 m，
        # 太近的目标会被判"已到达"，车一步不走 —— 2026-11-04 实测死锁）
        self.declare_parameter('min_goal_m', 1.2)
        # 每批打包几个视点（1 = 退回逐点走，会一停一停）
        self.declare_parameter('batch_size', 4)
        # 雷达起步：地图还没长开时，转向最开阔方向并前进一小段，
        # 让 slam_toolbox 整合新扫描（它要求移动 >=0.5 m 才会更新地图）
        self.declare_parameter('bootstrap_forward_m', 1.0)
        self.declare_parameter('bootstrap_speed_mps', 0.12)
        self.declare_parameter('bootstrap_turn_rate', 0.5)
        self.declare_parameter('bootstrap_min_clear_m', 1.3)
        # 手动起步通道（/cmd_vel_manual）默认关闭：实测底盘 sensors_ready 间歇为 0，
        # 指令会被丢弃、转向超时。起步改用 Nav2 目标（_scan_bootstrap_cell）。
        self.declare_parameter('bootstrap_manual_enabled', False)
        self.declare_parameter('bootstrap_turn_timeout_s', 12.0)
        self.declare_parameter('bootstrap_fwd_timeout_s', 20.0)

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
        # 2026-11-04：一停一停的根治办法 —— 把多个视点打包成"途经点"，
        # 经过时不停车（多点导航已实车验证这套机制）
        self._nav_many = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        self._batch = []
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
        self._scan = None
        self._cmd_manual = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self._boot = None            # 起步动作状态机
        # 雷达算方向必须忽略后方扇区（倒装 + 后部被屏蔽，读到的是车身）
        self.create_subscription(
            LaserScan, '/scan', self._on_scan,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._goal_future = None
        self._goal_handle = None
        self._gate_open = False
        self._active_since = 0.0
        self._state_pub = self.create_publisher(String, '/viewpoint_planner/state', 10)
        period = float(self.get_parameter('replan_period_s').value)
        self.create_timer(max(0.2, period), self._tick)
        # 起步动作必须高频发布：底盘有指令超时，1 Hz 推不动（2026-11-04 实测转向超时）
        self.create_timer(0.05, self._bootstrap_tick)
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

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _open_direction(self):
        """从 /scan 找最开阔方向，返回 (角度 rad, 净空 m)。

        只考虑有效视场（跳过后方 ±50°：雷达倒装，那里读到的是车身）。
        """
        if self._scan is None:
            return None
        r = np.asarray(self._scan.ranges, dtype=float)
        n = r.size
        if n == 0:
            return None
        ang = self._scan.angle_min + np.arange(n) * self._scan.angle_increment
        ok = np.isfinite(r) & (r > 0.12)
        rear = np.abs(ang) < math.radians(50.0)      # 后方扇区，忽略
        usable = ok & ~rear
        if not usable.any():
            return None
        sector = math.radians(10.0)
        best_deg, best_clear = None, -1.0
        edges = np.arange(-math.pi, math.pi, sector)
        for a0 in edges:
            m = usable & (ang >= a0) & (ang < a0 + sector)
            if not m.any():
                continue
            clear = float(np.percentile(r[m], 30))    # 用 30% 分位，避免单点尖刺
            if clear > best_clear:
                best_clear, best_deg = clear, a0 + sector / 2.0
        if best_deg is None or best_clear < float(
                self.get_parameter('bootstrap_min_clear_m').value):
            return None
        return best_deg, best_clear

    def _start_bootstrap(self, now: float) -> bool:
        """雷达起步：转向最开阔方向 → 前进 1 m。只前进/原地转，**不倒车**。

        ⚠️ 2026-11-04 实测：只开 motion_enabled 不够 —— `/cmd_vel_manual` 还要求
        `manual_enabled=true`，否则底盘直接丢弃手动指令（车一动不动，转向超时）。
        """
        # 先取消在跑的导航目标：底盘规定"有导航目标时手动指令一律丢弃"
        if self._goal_handle is not None:
            self._cancel_goal_only()
            self._set_state('bootstrap-cancel-goal')
            return True                        # 下一拍再开手动门

        # 手动通道要求：motion_enabled=True 且 manual_enabled=True（不能走 _open_gate，
        # 那个函数要求 manual 为假 —— 二者互斥，会死锁在 waiting-gate）
        if not self._ready:
            self._set_bool_param('motion_enabled', True)
            self._set_state('waiting-ready')
            return True
        if not self._manual:
            self._set_bool_param('motion_enabled', True)
            self._set_bool_param('manual_enabled', True)
            self._set_state('waiting-manual-gate')
            return True                     # 下一拍再确认
        chosen = self._open_direction()
        if chosen is None:
            return False
        target_ang, clear = chosen
        x, y, yaw = self._robot_xyyaw()
        if yaw is None:
            return False
        # 角度差（-pi..pi）
        err = math.atan2(math.sin(target_ang - yaw), math.cos(target_ang - yaw))
        self._boot = {'phase': 'turn', 't0': now, 'yaw0': yaw,
                      'x0': x, 'y0': y, 'clear': clear}
        if abs(err) < math.radians(12.0):
            self._boot['phase'] = 'forward'
            self._boot['t0'] = now
        self.get_logger().info(
            '雷达起步：最开阔方向 %.0f°（净空 %.2f m），先%s' % (
                math.degrees(target_ang), clear,
                '前进' if self._boot['phase'] == 'forward' else '转向'))
        self._set_state('bootstrap-scan')
        return True

    def _robot_xyyaw(self):
        try:
            tr = self._tf_buffer.lookup_transform('odom', 'base_footprint',
                                                 rclpy.time.Time())
        except Exception:                              # noqa: BLE001
            return None, None, None
        q = tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tr.transform.translation.x, tr.transform.translation.y, yaw

    def _bootstrap_tick(self) -> None:
        if self._boot is None:
            return
        self._run_bootstrap(time.monotonic())

    def _run_bootstrap(self, now: float) -> None:
        """执行起步动作（每拍推进一点）。"""
        if self._boot is None:
            return
        # 底盘判据 _sensors_ready() 里包含 motion_enabled；起步期间必须保持为真，
        # 否则手动指令会被当成"传感器不新鲜"丢弃（2026-11-04 实测 sensors_ready=0）
        if bool(self.get_parameter('open_gate').value):
            self._set_bool_param('motion_enabled', True)
        cmd = Twist()
        if self._boot['phase'] == 'turn':
            _, _, yaw = self._robot_xyyaw()
            if yaw is None:
                return
            chosen = self._open_direction()
            if chosen is None:
                self._finish_bootstrap('方向丢失')
                return
            err = math.atan2(math.sin(chosen[0] - yaw), math.cos(chosen[0] - yaw))
            if abs(err) < math.radians(12.0):
                self._boot['phase'] = 'forward'
                self._boot['t0'] = now
                self._boot['x0'], self._boot['y0'] = self._robot_xyyaw()[:2]
                return
            cmd.angular.z = max(-float(self.get_parameter('bootstrap_turn_rate').value),
                                min(float(self.get_parameter('bootstrap_turn_rate').value),
                                    err))
            if now - self._boot['t0'] > float(
                    self.get_parameter('bootstrap_turn_timeout_s').value):
                self._finish_bootstrap('转向超时')
                return
            self._cmd_manual.publish(cmd)
            return
        else:
            x, y, _ = self._robot_xyyaw()
            if x is None:
                return
            moved = math.hypot(x - self._boot['x0'], y - self._boot['y0'])
            # 前方净空不够就停（安全）
            chosen = self._open_direction()
            clear = chosen[1] if chosen else 0.0
            if (moved >= float(self.get_parameter('bootstrap_forward_m').value)
                    or clear < 0.5
                    or now - self._boot['t0'] > float(
                        self.get_parameter('bootstrap_fwd_timeout_s').value)):
                self._finish_bootstrap('前进 %.2f m 完成' % moved)
                return
            cmd.linear.x = float(self.get_parameter('bootstrap_speed_mps').value)
        self._cmd_manual.publish(cmd)

    def _finish_bootstrap(self, why: str) -> None:
        self._cmd_manual.publish(Twist())
        self._set_bool_param('manual_enabled', False)   # 交还自动链路
        self._boot = None
        self._radius = float(self.get_parameter('radius_start_m').value)
        self.get_logger().info('雷达起步结束：%s' % why)
        self._set_state('bootstrap-done')

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
                self._cancel_goal_only()
            else:
                self._set_state('going')
                return

        # 起步动作进行中 → 决策暂停（动作由 20 Hz 定时器推进）
        if self._boot is not None:
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

        # ① 先打包一串视点（途经不停车，根治"走一段停一下"）
        if int(self.get_parameter('batch_size').value) > 1:
            batch = self._plan_batch(cell, free, blocked, unknown, drivable)
            if batch:
                self._dispatch_batch(batch, now)
                return
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
        if boot is None:
            # 地图太小时没有够远的候选 → 用雷达找开阔方向，投影出一个 Nav2 目标
            boot = self._scan_bootstrap_cell(cell, drivable, free)
        if boot is not None:
            self._visited.add(boot)
            self._dispatch(boot, now, label='bootstrap')
            return

        # 地图太小 / 没有够远的目标 → 用雷达起步（转向最开阔方向 + 前进 1 m）
        if (int(free.sum()) <= int(self.get_parameter('bootstrap_max_cells').value)
                and bool(self.get_parameter('bootstrap_manual_enabled').value)):
            if self._start_bootstrap(now):
                return

        # ② 半径内没有 → 逐级扩大
        if self._radius < float(self.get_parameter('radius_max_m').value):
            self._radius = min(self._radius * 2.0,
                               float(self.get_parameter('radius_max_m').value))
            self._set_state('expand-radius %.0f m' % self._radius)
            return

        # ③ 只要还有前沿（门/没看过的区域）→ 必须去，**绝不判完成**
        #    距离要求逐级放宽：1.2 → 0.84 → 0.54 m（太近会被 Nav2 判已达，但
        #    总比原地不动强；放宽到 0.5 m 仍不行才认为这一片暂时去不了）
        if has_frontier:
            base = float(self.get_parameter('min_goal_m').value)
            for relax in (1.0, 0.7, 0.45):
                if self._goto_frontier(drivable, free, unknown, cell, now,
                                       min_dist_m=base * relax):
                    self._radius = float(self.get_parameter('radius_start_m').value)
                    return
            # 有前沿但一个合法目标都找不到 → 用雷达起步强行往前推进
            # （否则车会一直原地不动，地图永远不长）
            boot = self._scan_bootstrap_cell(cell, drivable, free)
            if boot is not None:
                self._visited.add(boot)
                self._dispatch(boot, now, label='bootstrap-forced')
                return
            self._set_state('frontier-blocked')
            return

        # ④ 没有前沿了：还要地图也停止增长，才算真的探完
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
        # 太近的目标会被 Nav2 判"已到达"（xy_goal_tolerance 0.4 m），必须过滤
        min_cells = float(self.get_parameter('min_goal_m').value) / res
        keep = [i for i, c in enumerate(cands)
                if dist[c[0], c[1]] >= min_cells]
        if not keep:
            return None
        cands = [cands[i] for i in keep]
        vis = [vis[i] for i in keep]
        idx = vc.pick_next_viewpoint(
            cands, vis, self._covered.ravel(), dist, radius_cells,
            visited=self._visited,
            min_gain_cells=int(self.get_parameter('min_gain_cells').value))
        return None if idx is None else cands[idx]

    def _scan_bootstrap_cell(self, cell, drivable, free):
        """地图还小时，用 /scan 找最开阔方向，投影出 1.5 m 外的 Nav2 目标。

        2026-11-04：手动通道（/cmd_vel_manual）在这台车上不稳定（底盘
        `sensors_ready` 间歇为 0，指令被丢弃），所以起步也走**已验证的自动通道**。
        """
        chosen = self._open_direction()
        if chosen is None:
            return None
        ang, clear = chosen
        res = self._map.info.resolution
        want = min(clear - 0.4, 1.5)              # 留 0.4 m 余量，最多走 1.5 m
        if want < 1.0:
            return None
        steps = int(want / res)
        # 雷达角度 → 地图坐标（scan 角度 0° = 车尾，需 +pi 转成车体朝向）
        heading = ang + math.pi
        yaw = self._robot_heading()
        if yaw is None:
            return None
        h, w = drivable.shape
        # 沿射线往回退，取**最远的已知空闲格**（不要求刚好 1.5 m，
        # 否则地图边缘稍微短一点就整个起步失败）
        dx = math.cos(yaw + heading)
        dy = math.sin(yaw + heading)
        ty = tx = None
        for k in range(steps, 3, -1):
            yy, xx = int(round(cell[0] + dy * k)), int(round(cell[1] + dx * k))
            if not (0 <= yy < h and 0 <= xx < w):
                continue
            if free[yy, xx]:
                ty, tx = yy, xx
                break
        if ty is None:
            return None
        self.get_logger().info('雷达起步：开阔方向 %.0f°（净空 %.1f m）→ 目标格 (%d,%d)' % (
            math.degrees(ang), clear, ty, tx))
        return (ty, tx)

    def _robot_heading(self):
        try:
            tr = self._tf_buffer.lookup_transform('map', 'base_footprint',
                                                 rclpy.time.Time())
        except Exception:                          # noqa: BLE001
            return None
        q = tr.transform.rotation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

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
        near = int(float(self.get_parameter('min_goal_m').value) / res)
        best, best_d = None, -1
        for c in cands:
            if c in self._visited:
                continue
            d = int(dist[c[0], c[1]])
            if near <= d <= limit and d > best_d:
                best, best_d = c, d
        return best

    def _plan_batch(self, cell, free, blocked, unknown, drivable) -> list:
        """在当前半径内贪心选一串视点（用累积可见性避免重复），供打包下发。"""
        res = self._map.info.resolution
        step = max(1, int(round(float(self.get_parameter('candidate_step_m').value) / res)))
        cands = vc.candidate_viewpoints(drivable, step)
        if not cands:
            return []
        seed = self._snap(cell, drivable)
        if seed is None:
            return []
        dist = vc.bfs_distance(drivable, seed)
        min_cells = float(self.get_parameter('min_goal_m').value) / res
        keep = [i for i, c in enumerate(cands) if dist[c[0], c[1]] >= min_cells]
        if not keep:
            return []
        cands = [cands[i] for i in keep]
        range_cells = int(float(self.get_parameter('sensor_range_m').value) / res)
        fill = int(self.get_parameter('fill_cells').value)
        rays = int(self.get_parameter('rays').value)
        vis = [vc.visible_cells(blocked, unknown, c, range_cells, rays, fill) for c in cands]
        covered = self._covered.ravel().copy()
        radius_cells = int(self._radius / res)
        batch = []
        prev = seed
        for _ in range(max(1, int(self.get_parameter('batch_size').value))):
            # ⚠️ 2026-11-04：每一步的距离必须相对**上一个点**算，否则选出来的
            # 顺序会是 远→近→远，规划器只能连成 Z 字形，车来回拐（用户截图确认）
            step_dist = vc.bfs_distance(drivable, prev)
            idx = vc.pick_next_viewpoint(
                cands, vis, covered, step_dist, radius_cells,
                visited=self._visited | set(batch),
                min_gain_cells=int(self.get_parameter('min_gain_cells').value))
            if idx is None:
                break
            batch.append(cands[idx])
            covered[vis[idx]] = True
            prev = cands[idx]
        return batch

    def _dispatch_batch(self, cells, now) -> None:
        """把一串视点打包成 NavigateThroughPoses（途经点不停车）。"""
        if not cells:
            return
        if len(cells) == 1 or not self._nav_many.server_is_ready():
            # 只有一个点、或批量服务不可用 → 退回单点
            self._visited.add(cells[0])
            self._dispatch(cells[0], now, label='viewpoint')
            return
        if self._manual:
            # 手动门还开着（起步残留）→ 自动链路被关，必须先关掉
            self._set_bool_param('manual_enabled', False)
            self._set_state('closing-manual-gate')
            return
        if not self._open_gate():
            self._set_state('waiting-gate')
            return
        res = self._map.info.resolution
        ox = self._map.info.origin.position.x
        oy = self._map.info.origin.position.y
        goal = NavigateThroughPoses.Goal()
        for (cy, cx) in cells:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = float(ox + (cx + 0.5) * res)
            ps.pose.position.y = float(oy + (cy + 0.5) * res)
            ps.pose.orientation.w = 1.0
            goal.poses.append(ps)
        self._batch = list(cells)
        self._target = cells[-1]
        self._target_at = now
        self._active_since = now
        self.get_logger().info('打包 %d 个视点 → 途经不停车' % len(cells))
        self._goal_future = self._nav_many.send_goal_async(goal)
        self._goal_future.add_done_callback(self._on_goal_response)
        self._set_state('batch %d' % len(cells))

    def _goto_frontier(self, drivable, free, unknown, cell, now,
                       min_dist_m: float = None) -> bool:
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
        # 前沿目标要够远（太近会被 Nav2 判"已到达"），但这里**按距离从近到远逐个尝试**，
        # 而不是一刀切返回 False —— 否则"有前沿但被过滤掉"会导致假完成（2026-11-04）
        want = float(min_dist_m if min_dist_m is not None
                     else self.get_parameter('min_goal_m').value)
        nearest = int(want / self._map.info.resolution)
        reachable = [f for f in fronts if dist[f[0], f[1]] >= 0]
        reachable.sort(key=lambda f: dist[f[0], f[1]])
        far = [f for f in reachable if dist[f[0], f[1]] >= nearest]
        tgt = vc.expansion_target(far, drivable, dist, inset_cells=inset) if far else None
        if tgt is None and reachable:
            # 再退一步：用离车最近的前沿（放宽距离要求）
            tgt = vc.expansion_target(reachable, drivable, dist, inset_cells=inset)
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
        self._scan = None
        self._cmd_manual = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self._boot = None            # 起步动作状态机
        # 雷达算方向必须忽略后方扇区（倒装 + 后部被屏蔽，读到的是车身）
        self.create_subscription(
            LaserScan, '/scan', self._on_scan,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._goal_future = None
        if ok:
            for c in self._batch:
                self._visited.add(c)
            self._batch = []
            self._dwell_until = time.monotonic() + float(
                self.get_parameter('dwell_s').value)
        else:
            self._batch = []
            self._close_gate()                        # 失败立刻关门，等下一轮重开
        self._target = None

    def _cancel_goal_only(self) -> None:
        """只取消当前导航目标（不动门控）。

        ⚠️ 底盘仲裁：`manual` 指令要求 **没有导航目标在跑**（nav_active=False）。
        不先取消，手动起步指令会被全部丢掉（2026-11-04 实测：发了 11 帧全被丢）。
        """
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:                      # noqa: BLE001
                pass
        self._goal_handle = None
        self._goal_future = None
        self._target = None

    def _cancel_active_goal(self) -> None:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:                         # noqa: BLE001
                pass
        self._goal_handle = None
        self._scan = None
        self._cmd_manual = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self._boot = None            # 起步动作状态机
        # 雷达算方向必须忽略后方扇区（倒装 + 后部被屏蔽，读到的是车身）
        self.create_subscription(
            LaserScan, '/scan', self._on_scan,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
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
