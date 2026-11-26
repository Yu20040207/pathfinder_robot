"""雷达「腿部检测 + 多目标跟踪」的纯逻辑（不含 ROS，便于单测）。

2026-11-24 新建（P1）。用户需求：人绕到车侧/车后仍要跟。相机只有 60~70° 视野，
人一转到侧面/背面就丢；360° 雷达是唯一能补这个洞的传感器。

⚠️ 雷达**不认识「脚」**，它只知道「两团窄点簇、间距像两条小腿」。
所以这里的判定是三道证据串联（缺第三道就一定会锁到桌腿）：

1. **几何**：两团窄簇（宽 0.03~0.30 m）、中心距 0.15~0.60 m、宽度相近；
2. **时间**：连续 ``confirm_frames`` 帧（默认 3 帧 = 0.3 s）都能匹配上，
   且相邻帧位移 < 0.30 m；
3. **身份**：相机在同一方位也看到人（``cam`` 参数）。相机没给证据时
   （``cam is None``）只有 ``confirm_frames_camless``（默认 6 帧）才允许建轨迹，
   且 10 s 不动就丢弃 —— 这条专门用来剔椅子腿、桌腿。

坐标系
------
本模块**不关心**是哪个 frame：调用方把点变换到同一个二维系（node 里是 ``odom``）
再喂进来即可。车在动的时候必须用 ``odom``/``map``，用 ``base_link`` 会把
「车在转」当成「人在动」。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

Point = Tuple[float, float]

DEFAULTS = {
    # —— 点级过滤 ——
    # 0.20 m 以内基本是车身自击残留；6.0 m 之外腿点太稀（0.5° 角分辨率下
    # 6 m 处相邻点间距 5.2 cm，还能连成簇，再远就断断续续）。
    'min_range_m': 0.20,
    'max_range_m': 6.0,
    # 车身矩形（车头 0.23 / 车尾 0.23 / 半宽 0.215）+ 3 cm 余量，矩形内的点全丢
    'body_front_m': 0.23,
    'body_rear_m': 0.23,
    'body_half_width_m': 0.215,
    'body_margin_m': 0.03,
    # —— 聚类 ——
    # 相邻点间距 > 0.10 m 就断开。1 m 处相邻点 8.7 mm、3 m 处 26 mm，
    # 都不会误断；两条腿之间通常 0.15 m 以上，一定断开。
    'cluster_break_m': 0.10,
    'leg_min_points': 3,
    # 小腿截面（含裤脚/鞋）实测宽度量级：静止站立 0.10~0.15 m，
    # 大步走时投影到 0.20 m 左右，取上限 0.30 m。
    'leg_w_min': 0.03,
    'leg_w_max': 0.30,
    # —— 腿对 ——
    # 两小腿中心距：静止 0.20~0.35 m，走路 0.15~0.55 m；0.60 m 以上基本是
    # 门框（0.7~1.2 m）或两个人。
    'pair_gap_min': 0.15,
    'pair_gap_max': 0.60,
    'pair_width_ratio_max': 2.0,
    # 两条腿到车的距离差：并排站/走时很小；差得多说明是「一前一后的两个物体」
    'pair_range_diff_m': 0.25,
    # —— 单团宽簇 = 两腿粘在一起（2026-11-24 实机 bag 实测）——
    # 人正对车、距离 >1.5 m 时，两条小腿在雷达里合成**一团**宽 0.25~0.29 m
    # （1.42 m 处还能分成两团，1.90 m 就粘住了）。只认"腿对"的话，
    # 跟随最常用的 1.5~3 m 距离上会完全看不到人 → 必须把宽簇也算人。
    'merged_w_min': 0.20,
    'merged_w_max': 0.45,
    # 宽簇宽度 ≈ 腿间距 + 2×单腿宽；单腿宽按 0.08 m 取，减 0.16 m 得间距估计
    'merged_leg_width_m': 0.16,
    'merged_min_points': 4,
    # 两对中心相距小于它就认为在说同一个人（治"一个人变两个"）
    'pair_dedup_m': 0.45,
    # 两对撞车时优先信「腿对」还是「宽簇」（实测两种都支持，默认信腿对）
    'prefer_two_leg_pairs': True,
    # 确认阶段允许丢帧：这段时间内攒够 confirm_frames 帧就算确认
    'pending_hold_s': 0.30,
    # —— 时间确认 / 跟踪 ——
    'confirm_frames': 3,
    'confirm_frames_camless': 6,
    'confirm_snap_m': 0.30,       # 相邻帧同一目标的位移上限
    'assoc_gate_m': 0.40,         # 卡尔曼预测与观测的关联门限
    'coast_frames': 3,            # 丢几帧还继续预测（0.3 s）
    # 2.0 s 才删（实测 1.5 s 时人换个位置就会丢一条轨迹；P2 的停车判据另有 lost_stop）
    'drop_after_s': 2.0,
    # 8 条，配合"优先删不动的那条"（见 _born）
    'max_tracks': 8,
    # —— 静止误报剔除（椅子/桌腿/门框）——
    # ⚠️ 别调太小：没有相机时，"人站着不动 10 秒"会被当成静止家具丢掉
    'static_drop_s': 10.0,
    'static_move_m': 0.15,
    # 静止剔除**必须在有相机数据时**才启用：没有相机时"站着不动的人"和
    # "椅子腿"在雷达里完全一样（2026-11-24 bag 实测：站 9 s 就被当家具丢掉，
    # 还进了 300 s 黑名单 → 人彻底消失）。有相机才有资格说"没人，是家具"。
    'static_drop_requires_cam': True,
    # —— 相机身份门控 ——
    'cam_gate_deg': 18.0,       # 相机方位门限（°）：35° 会把"人旁边的一切"都背书掉
    'cam_gate_range_m': 1.0,    # 距离门限（m）：按框高估出的距离对不上就不背书
    'cam_back_s': 1.0,          # 背书**有效期**：超过这么久没再被相机确认就不算"人"
    # P1（采集数据）阶段为 False：相机没看到也允许建轨迹，方便看全；
    # 接入 P2 融合后建议改 True —— 新目标必须有相机背书，否则一律当家具。
    'birth_requires_cam': False,
    # 被判为「静止家具」丢掉的位置，在这么长时间内不再建轨迹
    # （否则椅子腿会每 10 s 轮回一条新轨迹，日志一直刷）
    'reject_hold_s': 300.0,
    'reject_radius_m': 0.60,
    # —— 卡尔曼噪声 ——
    'q_accel': 0.5,               # 人步行加速度量级 m/s²
    'meas_sigma_r': 0.03,         # 径向测量噪声 m
}


# --------------------------------------------------------------------------
# 小矩阵工具（4x4 状态，2x2 观测；只为避免依赖 numpy）
# --------------------------------------------------------------------------
Matrix = List[List[float]]


def _mat_mul(a: Matrix, b: Matrix) -> Matrix:
    return [[sum(a[i][k] * b[k][j] for k in range(len(b)))
             for j in range(len(b[0]))] for i in range(len(a))]


def _transpose(a: Matrix) -> Matrix:
    return [[a[i][j] for i in range(len(a))] for j in range(len(a[0]))]


def _mat_add(a: Matrix, b: Matrix) -> Matrix:
    return [[a[i][j] + b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def _mat_sub(a: Matrix, b: Matrix) -> Matrix:
    return [[a[i][j] - b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def _mat_vec(a: Matrix, v: Sequence[float]) -> List[float]:
    return [sum(a[i][k] * v[k] for k in range(len(v))) for i in range(len(a))]


def _inv2(m: Matrix) -> Matrix:
    det = m[0][0] * m[1][1] - m[0][1] * m[1][0]
    if abs(det) < 1e-12:
        det = 1e-12
    return [[m[1][1] / det, -m[0][1] / det],
            [-m[1][0] / det, m[0][0] / det]]


def _eye(n: int) -> Matrix:
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def ang_norm(a: float) -> float:
    """把角度规整到 (-pi, pi]。"""
    return math.atan2(math.sin(a), math.cos(a))


# --------------------------------------------------------------------------
# 点云处理
# --------------------------------------------------------------------------
def scan_to_points(ranges: Sequence[float], angle_min: float,
                   angle_increment: float,
                   params: Optional[dict] = None) -> List[Point]:
    """LaserScan → 二维点列表（雷达自身坐标系），已丢掉无效点与车身矩形内的点。

    索引必须是**扫描角顺序**（``cluster_points`` 依赖相邻点连通性）。
    """
    p = _p(params)
    pts: List[Point] = []
    for i, r in enumerate(ranges):
        if r is None:
            continue
        try:
            r = float(r)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(r) or r <= 0.0:
            continue
        if r < p['min_range_m'] or r > p['max_range_m']:
            continue
        a = angle_min + i * angle_increment
        x = r * math.cos(a)
        y = r * math.sin(a)
        if _in_body(x, y, p):
            continue
        pts.append((x, y))
    return pts


def _in_body(x: float, y: float, p: dict) -> bool:
    m = p['body_margin_m']
    return (-p['body_rear_m'] - m <= x <= p['body_front_m'] + m
            and -p['body_half_width_m'] - m <= y
            <= p['body_half_width_m'] + m)


def to_world(points: Iterable[Point], robot_x: float, robot_y: float,
             robot_yaw: float) -> List[Point]:
    """机器人系点 → 世界（odom/map）系。"""
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    return [(robot_x + c * px - s * py, robot_y + s * px + c * py)
            for px, py in points]


def world_to_robot(points: Iterable[Point], robot_x: float, robot_y: float,
                   robot_yaw: float) -> List[Point]:
    """世界系点 → 机器人系（给可视化/方向判断用）。"""
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    out = []
    for wx, wy in points:
        dx, dy = wx - robot_x, wy - robot_y
        out.append((c * dx + s * dy, -s * dx + c * dy))
    return out


def cluster_points(points: Sequence[Point],
                   params: Optional[dict] = None) -> List[List[Point]]:
    """按相邻点间距断开成簇（点必须是扫描角顺序）。"""
    p = _p(params)
    break_m = p['cluster_break_m']
    clusters: List[List[Point]] = []
    cur: List[Point] = []
    prev: Optional[Point] = None
    for pt in points:
        if prev is not None:
            if math.hypot(pt[0] - prev[0], pt[1] - prev[1]) > break_m:
                if cur:
                    clusters.append(cur)
                cur = []
        cur.append(pt)
        prev = pt
    if cur:
        clusters.append(cur)
    return clusters


def cluster_width(points: Sequence[Point]) -> float:
    """簇的“宽度”＝最大两点间距（点很少，O(n²) 可接受）。"""
    best = 0.0
    n = len(points)
    for i in range(n):
        xi, yi = points[i]
        for j in range(i + 1, n):
            xj, yj = points[j]
            d = math.hypot(xj - xi, yj - yi)
            if d > best:
                best = d
    return best


def cluster_centroid(points: Sequence[Point]) -> Point:
    n = max(1, len(points))
    return (sum(p[0] for p in points) / n, sum(p[1] for p in points) / n)


@dataclass
class Leg:
    """一条候选小腿。

    ``r`` / ``bearing`` 是**相对车**（调用方传 ``origin``）算的，不是相对坐标系原点。
    2026-11-24 实机踩过：车在 odom 里位于 (-5.36, 0.18)，拿原点算距离会显示 7.45 m
    （而雷达上限只有 6.0 m），且相机背书的角度门控全错。
    """
    x: float
    y: float
    width: float
    n: int
    r: float          # 到车的距离
    bearing: float    # 车体系方位角


def find_legs(points: Sequence[Point], params: Optional[dict] = None,
              origin: Point = (0.0, 0.0)) -> List[Leg]:
    """聚类 + 筛出「窄而小」的候选腿簇。

    ``origin`` = 车的位姿（同坐标系下的 x, y）：距离与方位角都相对它算。
    """
    p = _p(params)
    ox, oy = origin
    legs: List[Leg] = []
    for cl in cluster_points(points, p):
        if len(cl) < p['leg_min_points']:
            continue
        w = cluster_width(cl)
        if w < p['leg_w_min'] or w > p['leg_w_max']:
            continue
        cx, cy = cluster_centroid(cl)
        dx, dy = cx - ox, cy - oy
        legs.append(Leg(cx, cy, w, len(cl), math.hypot(dx, dy),
                        math.atan2(dy, dx)))
    legs.sort(key=lambda leg: leg.bearing)
    return legs


@dataclass
class Pair:
    """一对小腿（= 候选的一个人）。坐标取两腿中点（odom 系）。"""
    x: float
    y: float
    gap: float
    w_a: float
    w_b: float
    r: float          # 到车的距离
    bearing: float    # 车体系方位角（相机门控用它比较）
    legs: Tuple[Leg, ...] = ()
    # True = 两条腿粘成一团（单团宽簇），几何置信度低于真正的腿对
    merged: bool = False

    @property
    def width(self) -> float:
        return max(self.w_a, self.w_b)


def match_pairs(legs: Sequence[Leg], params: Optional[dict] = None,
                origin: Point = (0.0, 0.0)) -> List[Pair]:
    """把所有候选腿两两配对，只留几何上像「两条腿」的，并贪心去重。

    去重规则：一个腿簇只能属于一对；先按「间距接近 0.35 m」优先。
    """
    p = _p(params)
    ox, oy = origin
    cands: List[Tuple[float, float, Pair]] = []
    for i in range(len(legs)):
        for j in range(i + 1, len(legs)):
            a, b = legs[i], legs[j]
            gap = math.hypot(b.x - a.x, b.y - a.y)
            if gap < p['pair_gap_min'] or gap > p['pair_gap_max']:
                continue
            lo, hi = min(a.width, b.width), max(a.width, b.width)
            if hi > lo * p['pair_width_ratio_max'] + 1e-9:
                continue
            if abs(a.r - b.r) > p['pair_range_diff_m']:
                continue
            mx, my = (a.x + b.x) / 2.0, (a.y + b.y) / 2.0
            mdx, mdy = mx - ox, my - oy
            pair = Pair(mx, my, gap, a.width, b.width,
                        math.hypot(mdx, mdy), math.atan2(mdy, mdx), (a, b))
            # 排序代价：间距偏离 0.35 m 的量 + 两腿宽度差（越小越像一个人）
            cost = abs(gap - 0.35) + abs(a.width - b.width)
            cands.append((cost, i * 1000 + j, pair))
    cands.sort(key=lambda item: item[0])
    used = set()
    out: List[Pair] = []
    for _cost, key, pair in cands:
        i, j = divmod(key, 1000)
        if i in used or j in used:
            continue
        used.add(i)
        used.add(j)
        out.append(pair)
    return out


def merged_candidates(points: Sequence[Point], params: Optional[dict] = None,
                      origin: Point = (0.0, 0.0)) -> List[Pair]:
    """把「两条腿粘成一团」的宽簇也当成人。

    实测（2026-11-24）：1.42 m 处两腿分成两团（各宽 0.10 m），1.90 m 处合成
    一团宽 0.25~0.29 m。后者是跟随最常用的距离，必须认，否则人一退到 1.5 m
    以外就"消失"。

    ``gap`` 是**估计值**（``宽 − merged_leg_width_m``），只用于日志/统计，
    真正的确认靠时间 + 相机背书。
    """
    p = _p(params)
    ox, oy = origin
    out: List[Pair] = []
    for cl in cluster_points(points, p):
        if len(cl) < p['merged_min_points']:
            continue
        w = cluster_width(cl)
        if w < p['merged_w_min'] or w > p['merged_w_max']:
            continue
        cx, cy = cluster_centroid(cl)
        dx, dy = cx - ox, cy - oy
        gap = max(p['pair_gap_min'], w - p['merged_leg_width_m'])
        out.append(Pair(cx, cy, gap, w, w, math.hypot(dx, dy),
                        math.atan2(dy, dx), (), True))
    return out


def dedup_pairs(pairs: Sequence[Pair], params: Optional[dict] = None) -> List[Pair]:
    """同一个人的两对只留一个（默认优先「腿对」，其次按间距接近 0.32 m）。"""
    p = _p(params)
    prefer_two = p['prefer_two_leg_pairs']

    def rank(pr: Pair) -> Tuple[int, float]:
        kind = 0 if (pr.merged is False and prefer_two) else (
            1 if not prefer_two and pr.merged else
            (1 if pr.merged is False else 2))
        return (kind, abs(pr.gap - 0.32))

    kept: List[Pair] = []
    for pr in sorted(pairs, key=rank):
        if any(math.hypot(pr.x - q.x, pr.y - q.y) <= p['pair_dedup_m']
               for q in kept):
            continue
        kept.append(pr)
    return kept


def detect_pairs(points: Sequence[Point], params: Optional[dict] = None,
                 origin: Point = (0.0, 0.0)) -> Tuple[List[Pair], List[Leg]]:
    """完整的一步：候选腿 → 腿对 + 单团宽簇 → 去重。

    返回 ``(pairs, legs)``，``legs`` 只用于日志/自检。
    """
    p = _p(params)
    legs = find_legs(points, p, origin)
    cands = match_pairs(legs, p, origin)
    cands += merged_candidates(points, p, origin)
    return dedup_pairs(cands, p), legs


# --------------------------------------------------------------------------
# 卡尔曼滤波（匀速模型）
# --------------------------------------------------------------------------
class Kalman2D:
    """状态 [x, y, vx, vy]，观测 [x, y]。"""

    def __init__(self, x: float, y: float, now: float,
                 q_accel: float = 0.5, sigma_r: float = 0.03) -> None:
        self.state = [float(x), float(y), 0.0, 0.0]
        self.P: Matrix = [[0.50, 0.0, 0.0, 0.0],
                          [0.0, 0.50, 0.0, 0.0],
                          [0.0, 0.0, 1.00, 0.0],
                          [0.0, 0.0, 0.0, 1.00]]
        self.t = float(now)
        self.q = float(q_accel)
        self.r = float(sigma_r) ** 2

    @property
    def x(self) -> float:
        return self.state[0]

    @property
    def y(self) -> float:
        return self.state[1]

    @property
    def vx(self) -> float:
        return self.state[2]

    @property
    def vy(self) -> float:
        return self.state[3]

    def predict(self, now: float) -> None:
        dt = max(0.0, float(now) - self.t)
        if dt <= 0.0:
            return
        self.t = float(now)
        F: Matrix = [[1.0, 0.0, dt, 0.0],
                     [0.0, 1.0, 0.0, dt],
                     [0.0, 0.0, 1.0, 0.0],
                     [0.0, 0.0, 0.0, 1.0]]
        self.state = _mat_vec(F, self.state)
        q, dt2, dt3, dt4 = self.q, dt * dt, dt ** 3, dt ** 4
        Q: Matrix = [[q * dt4 / 4.0, 0.0, q * dt3 / 2.0, 0.0],
                     [0.0, q * dt4 / 4.0, 0.0, q * dt3 / 2.0],
                     [q * dt3 / 2.0, 0.0, q * dt2, 0.0],
                     [0.0, q * dt3 / 2.0, 0.0, q * dt2]]
        self.P = _mat_add(_mat_mul(_mat_mul(F, self.P), _transpose(F)), Q)

    def update(self, zx: float, zy: float) -> None:
        H: Matrix = [[1.0, 0.0, 0.0, 0.0],
                     [0.0, 1.0, 0.0, 0.0]]
        R: Matrix = [[self.r, 0.0], [0.0, self.r]]
        y0 = float(zx) - self.state[0]
        y1 = float(zy) - self.state[1]
        S = _mat_add(_mat_mul(_mat_mul(H, self.P), _transpose(H)), R)
        K = _mat_mul(_mat_mul(self.P, _transpose(H)), _inv2(S))
        self.state = [self.state[i] + K[i][0] * y0 + K[i][1] * y1
                      for i in range(4)]
        self.P = _mat_mul(_mat_sub(_eye(4), _mat_mul(K, H)), self.P)


@dataclass
class Track:
    id: int
    kf: Kalman2D
    born_at: float
    last_seen: float
    hits: int = 1
    misses: int = 0
    moved: float = 0.0        # 累计路程（诊断用）
    cam_seen: bool = False      # 出生时/任何时刻被相机确认过（用于"静止家具"判定）
    cam_at: float = 0.0         # **最后一次**被相机确认的时刻（背书有效期靠它）
    x0: float = 0.0           # 出生位置
    y0: float = 0.0

    def cam_fresh(self, now: float, hold_s: float = 1.0) -> bool:
        """此刻还算不算"相机正在确认的这个人"。

        ⚠️ 原来的 `cam_seen` 是**一票永久**：只要曾经和人同方位出现过一次
        （35° 门限 + 方位恒为 0 的 bug 让它几乎必然发生），椅子/货箱就会被永久
        当成"相机背书过的人"，跟随于是奔着那团东西去（2026-11-26 现场截图）。
        """
        return bool(self.cam_at) and (now - self.cam_at) <= hold_s

    @property
    def net_move(self) -> float:
        """离出生点多远（米）。判断"是不是家具"必须用它：
        累积路程会被测量噪声一直推高（每秒 1~2 cm × 100 帧 > 1 m），
        家具因此永远删不掉——2026-11-24 bag 里椅子腿轨迹就是这么赖着不走的。"""
        return math.hypot(self.kf.x - self.x0, self.kf.y - self.y0)

    @property
    def x(self) -> float:
        return self.kf.x

    @property
    def y(self) -> float:
        return self.kf.y

    def to_dict(self, now: float) -> dict:
        age = max(0.0, now - self.born_at)
        fresh = self.cam_fresh(now)
        static = (not fresh and age > 3.0 and self.net_move < 0.15)
        return {'id': self.id, 'x': round(self.kf.x, 3), 'y': round(self.kf.y, 3),
                'vx': round(self.kf.vx, 3), 'vy': round(self.kf.vy, 3),
                'age_s': round(age, 2), 'misses': self.misses,
                'cam': fresh, 'cam_age_s': (round(now - self.cam_at, 2)
                                            if self.cam_at else None),
                'static': static}


class LegTracker:
    """多目标跟踪器：吃 ``Pair`` 列表，吐已确认的轨迹。

    ``cam`` 参数（可缺省）::

        {'bearing': 弧度, 'range': 米或 None}

    相机给不出证据（``cam=None``）时仍然允许建轨迹，但要 ``confirm_frames_camless``
    帧（默认 6 帧）—— 这是给 P1 采集数据用的；接入视觉后建议把
    ``require_cam_for_birth`` 打开，新目标必须由相机背书。
    """

    def __init__(self, params: Optional[dict] = None) -> None:
        self.p = _p(params)
        self.tracks: List[Track] = []
        self._pending: List[dict] = []
        self._rejects: List[Tuple[float, float, float]] = []   # (x, y, 失效时刻)
        self._next_id = 1

    # ---------------------------------------------------------------- 内部
    def _cam_ok(self, bearing: float, cam: Optional[dict]) -> Optional[bool]:
        """None = 没有相机数据；True/False = 相机是否背书。"""
        if not cam:
            return None
        cb = cam.get('bearing')
        if cb is None:
            return None
        if abs(ang_norm(bearing - float(cb))) > \
                math.radians(self.p['cam_gate_deg']):
            return False
        return True

    def _cam_range_ok(self, r: float, cam: Optional[dict]) -> bool:
        if not cam or cam.get('range') is None:
            return True
        return abs(r - float(cam['range'])) <= self.p['cam_gate_range_m']

    # ---------------------------------------------------------------- 主入口
    def step(self, pairs: Sequence[Pair], now: float,
             cam: Optional[dict] = None) -> List[dict]:
        p = self.p
        now = float(now)

        # 1) 预测
        for tr in self.tracks:
            tr.kf.predict(now)

        # 2) 关联（贪心最近邻 + 门限）
        cands: List[Tuple[float, int, int]] = []
        for ti, tr in enumerate(self.tracks):
            for pi, pr in enumerate(pairs):
                d = math.hypot(pr.x - tr.x, pr.y - tr.y)
                if d <= p['assoc_gate_m']:
                    cands.append((d, ti, pi))
        cands.sort(key=lambda c: c[0])
        used_t, used_p = set(), set()
        for _d, ti, pi in cands:
            if ti in used_t or pi in used_p:
                continue
            used_t.add(ti)
            used_p.add(pi)
            self._apply_measurement(self.tracks[ti], pairs[pi], now, cam)

        # 3) 未关联的轨迹：丢帧计数；超时删除
        for ti, tr in enumerate(self.tracks):
            if ti not in used_t:
                tr.misses += 1
        self.tracks = [tr for tr in self.tracks
                       if (now - tr.last_seen) <= p['drop_after_s']]
        # 静止且相机从没看见过 → 视为家具（椅子腿/桌腿），丢轨迹并把位置拉黑
        cam_present = cam is not None
        keep: List[Track] = []
        for tr in self.tracks:
            static_junk = (not tr.cam_fresh(now)
                           and (not p['static_drop_requires_cam'] or cam_present)
                           and (now - tr.born_at) > p['static_drop_s']
                           and tr.net_move < p['static_move_m'])
            if static_junk:
                self._rejects.append((tr.x, tr.y, now + p['reject_hold_s']))
            else:
                keep.append(tr)
        self.tracks = keep
        self._rejects = [r for r in self._rejects if r[2] > now]

        # 4) 未关联的腿对 → 进 pending，连续 confirm 帧才建轨迹
        self._feed_pending([pairs[pi] for pi in range(len(pairs))
                            if pi not in used_p], now, cam)

        # 5) 输出（只输出已确认的）
        out = [tr.to_dict(now) for tr in self.tracks]
        out.sort(key=lambda d: d['id'])
        return out

    def _apply_measurement(self, tr: Track, pr: Pair, now: float,
                           cam: Optional[dict]) -> None:
        before = (tr.kf.x, tr.kf.y)
        tr.kf.update(pr.x, pr.y)
        tr.moved += math.hypot(tr.kf.x - before[0], tr.kf.y - before[1])
        tr.last_seen = now
        tr.hits += 1
        tr.misses = 0
        if self._cam_ok(pr.bearing, cam) and self._cam_range_ok(pr.r, cam):
            tr.cam_seen = True
            tr.cam_at = now

    def _feed_pending(self, pairs: Sequence[Pair], now: float,
                      cam: Optional[dict]) -> None:
        """未关联的腿对先攒够「连续 N 帧」再转正（这是防误报的核心）。

        每帧只允许一个 pending 抢一个腿对（最近邻 + ``confirm_snap_m``），
        抢到就 ``hits += 1``；某帧没抢到就清零重新数。
        """
        p = self.p
        matched: set = set()
        for pnd in self._pending:
            best, best_d = None, 1e9
            for pi, pr in enumerate(pairs):
                if pi in matched:
                    continue
                d = math.hypot(pr.x - pnd['x'], pr.y - pnd['y'])
                if d < best_d and d <= p['confirm_snap_m']:
                    best, best_d = pi, d
            if best is None:
                # 允许偶尔丢帧（雷达对人腿的检出是断续的）：
                # pending_hold_s 内不清零，超时才重新数
                if (now - pnd['seen_at']) > p['pending_hold_s']:
                    pnd['hits'] = 0
                continue
            matched.add(best)
            pr = pairs[best]
            pnd['moved'] += math.hypot(pr.x - pnd['x'], pr.y - pnd['y'])
            pnd['x'], pnd['y'] = pr.x, pr.y
            pnd['hits'] += 1
            pnd['seen_at'] = now
            cam_ok = self._cam_ok(pr.bearing, cam)
            if p['birth_requires_cam'] and cam_ok is False:
                continue                       # 相机看着别处 → 这不是人
            if not cam_ok and self._in_reject_zone(pr.x, pr.y):
                pnd['hits'] = 0                # 已判定为家具的位置，不再建轨迹
                continue
            need = p['confirm_frames'] if cam_ok else p['confirm_frames_camless']
            if pnd['hits'] >= need:
                self._born(pr, now, bool(cam_ok), pnd['moved'])
                pnd['done'] = True

        # 刷新 pending：丢掉已转正 / 丢帧太久的
        kept = []
        for pnd in self._pending:
            if pnd.get('done'):
                continue
            if pnd['hits'] > 0 or (now - pnd['seen_at']) <= p['drop_after_s']:
                kept.append(pnd)
        self._pending = kept

        # 本帧还没归属的腿对 → 新候选
        for pi, pr in enumerate(pairs):
            if pi in matched:
                continue
            self._pending.append({'x': pr.x, 'y': pr.y, 'hits': 1,
                                  'born': now, 'seen_at': now, 'moved': 0.0})

    def _in_reject_zone(self, x: float, y: float) -> bool:
        for rx, ry, _until in self._rejects:
            if math.hypot(x - rx, y - ry) <= self.p['reject_radius_m']:
                return True
        return False

    def _born(self, pr: Pair, now: float, cam_seen: bool,
              moved: float = 0.0) -> None:
        p = self.p
        if len(self.tracks) >= p['max_tracks']:
            # 满了先挤掉"最像家具"的：没相机背书且几乎没动过的。
            # 不能只按"最老"挤——椅子腿会不断轮回出生，把真人挤掉（实测 132 次出生）。
            def junk_rank(t: Track):
                return (0 if not t.cam_seen else 1, t.net_move)
            victim = min(self.tracks, key=junk_rank)
            # 新来的也不像人（没相机、没动过）就别挤了，否则家具会无限轮回
            if (not cam_seen and moved < p['static_move_m']
                    and not victim.cam_seen
                    and victim.net_move < p['static_move_m']):
                return
        self.tracks.append(Track(
            id=self._next_id,
            kf=Kalman2D(pr.x, pr.y, now, p['q_accel'], p['meas_sigma_r']),
            born_at=now, last_seen=now, cam_seen=cam_seen,
            cam_at=(now if cam_seen else 0.0),
            x0=pr.x, y0=pr.y))
        self._next_id += 1

    @property
    def pending_count(self) -> int:
        return len([p for p in self._pending if p.get('hits', 0) > 0])


def _p(params: Optional[dict]) -> dict:
    """合并默认参数（不修改传入的 dict）。"""
    out = dict(DEFAULTS)
    if params:
        for k, v in params.items():
            if k in out:
                out[k] = v
    return out


# --------------------------------------------------------------------------
# 自检文字（P1 现场演示用：一句话看懂当前看到了什么）
# --------------------------------------------------------------------------
def describe(clusters: int, pairs: Sequence[Pair],
             tracks: Sequence[dict]) -> str:
    if not pairs:
        pair_txt = '无腿对'
    else:
        pr = min(pairs, key=lambda q: q.r)
        pair_txt = ('最佳腿对 间距 %.2f m 宽 %.2f/%.2f m 距离 %.2f m'
                    % (pr.gap, pr.w_a, pr.w_b, pr.r))
    if not tracks:
        trk_txt = '轨迹 0'
    else:
        order = sorted(tracks, key=lambda d: (bool(d.get('static')), d['id']))
        n_static = sum(1 for d in tracks if d.get('static'))
        parts = []
        for tr in order:
            if tr.get('static') and parts:
                continue          # 家具候选只计数，不再逐个打印
            v = math.hypot(tr['vx'], tr['vy'])
            tag = '~' if tr.get('static') else '#'
            parts.append('%s%d(%.2f,%.2f) v=%.2f%s'
                         % (tag, tr['id'], tr['x'], tr['y'], v,
                            '' if tr.get('cam') else '?cam'))
        trk_txt = '轨迹 %d：%s' % (len(tracks) - n_static, ' '.join(parts) or '无')
        if n_static:
            trk_txt += ' | 静止疑似家具 %d' % n_static
    return '簇 %d 个 | %s | %s' % (clusters, pair_txt, trk_txt)
