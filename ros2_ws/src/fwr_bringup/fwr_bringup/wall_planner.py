"""沿墙建的局部路径规划器（纯函数，方便单测）。

思路（DWA / 弧线族方法）：
  1. 把 /scan 转成机体坐标系下的障碍点
  2. 生成一族候选弧线（不同曲率 → 不同转弯半径）
  3. 逐条检查是否撞障碍（含机器人半径膨胀）
  4. 对通过检查的弧线打分：
       · 离障碍越远越好
       · 终点与「跟随侧墙」的目标距离越接近越好
       · 尽量往前走、少拐急弯
       · 与上一次选择一致（迟滞，防止左右抖）
  5. 返回最优弧线的位姿序列（机体坐标系）和曲率

坐标约定：x 向前、y 向左、theta 逆时针为正（标准 ROS base 坐标系）。
"""
import math

# 机器人外接半径（0.60x0.43 车身 → 半对角约 0.37，取 0.32 留一点余量）
ROBOT_RADIUS_M = 0.32


def scan_to_points(ranges, angle_min, angle_increment, max_range=4.0,
                   self_min=0.20):
    """把 LaserScan 转成机体坐标系点列 [(x, y)]。

    雷达倒装：scan 角 0° = 车尾，±180° = 车头。
    转成 base 系后：车头 = +x，车左 = +y。
    """
    pts = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= self_min or r > max_range:
            continue
        a = angle_min + i * angle_increment            # -pi..pi，0=车尾
        sb = (math.pi - abs(a)) * (1.0 if a >= 0 else -1.0)   # 0=车头，+ = 右
        # base 系：车头 +x，车左 +y；右侧对应 -y
        pts.append((r * math.cos(sb), -r * math.sin(sb)))
    return pts


def _build_obstacle_grid(points, cell=0.10, extent=5.0):
    grid = {}
    for x, y in points:
        if abs(x) > extent or abs(y) > extent:
            continue
        grid[(int(round(x / cell)), int(round(y / cell)))] = True
    return grid, cell


def _min_clearance(grid, cell, x, y, limit=1.0):
    """(x,y) 到最近障碍格的距离（只查 limit 内，找不到就返回 limit）。"""
    ci, cj = int(round(x / cell)), int(round(y / cell))
    span = int(limit / cell) + 1
    best = limit
    for di in range(-span, span + 1):
        for dj in range(-span, span + 1):
            if (ci + di, cj + dj) in grid:
                d = math.hypot(di * cell, dj * cell)
                if d < best:
                    best = d
    return best


def arc_poses(kappa, length=2.0, step=0.10):
    """给定曲率生成弧线位姿序列 [(x, y, theta)]。"""
    x = y = th = 0.0
    out = [(x, y, th)]
    n = int(length / step)
    for _ in range(n):
        th += kappa * step
        x += step * math.cos(th)
        y += step * math.sin(th)
        out.append((x, y, th))
    return out


def choose_arc(points, follow_side='right', target_dist=0.75,
               prev_kappa=0.0, kappas=None, length=2.0, step=0.10,
               robot_radius=ROBOT_RADIUS_M, clearance_weight=1.0,
               wall_weight=2.0, straight_weight=0.35, smooth_weight=0.5,
               goal_bias=0.6):
    """挑一条最好的弧线。

    返回 (poses, kappa) 或 (None, None)（全部被挡）。
    """
    if not points:
        return None, None
    if kappas is None:
        # 半径 0.25 m(急) … 直行 … 向左 0.25 m
        kappas = [4.0, 2.5, 1.6, 0.9, 0.45, 0.0, -0.45, -0.9, -1.6, -2.5, -4.0]
    grid, cell = _build_obstacle_grid(points)

    # 跟随侧墙的实际距离（在机体前方 2 m 内取最近点）
    side_sign = -1.0 if follow_side == 'right' else 1.0   # 右墙 → y 为负
    wall_d = None
    for (px, py) in points:
        if px < 0.15 or px > 2.0:
            continue
        if side_sign < 0 and py < -0.05:
            d = -py
        elif side_sign > 0 and py > 0.05:
            d = py
        else:
            continue
        wall_d = d if wall_d is None else min(wall_d, d)
    wall_d = wall_d if wall_d is not None else target_dist

    best = None
    for k in kappas:
        poses = arc_poses(k, length, step)
        ok = True
        min_clear = 9.9
        for (x, y, _th) in poses:
            if x < 0.25 and abs(y) < 0.35:
                continue          # 车体本身所在位置，不查
            c = _min_clearance(grid, cell, x, y, limit=1.2)
            if c < robot_radius:          # 撞了（含膨胀）
                ok = False
                break
            min_clear = min(min_clear, c)
        if not ok:
            continue

        xe, ye, the = poses[-1]
        # 沿墙项：离墙太远就奖励向墙侧偏，太近就奖励离开
        # need > 0 = 离墙太远，需要朝墙靠近
        # 贴右墙时墙在 -y，「靠近」= ye 变负 → need(+)*ye(-) 为负，要取负号抵消
        need = target_dist - wall_d
        sign = -1.0 if follow_side == 'right' else 1.0
        wall_term = -wall_weight * need * ye * sign
        # 间隙奖励封顶在目标距离：离墙更远不再加分，否则永远往空地跑
        score = (clearance_weight * min(min_clear, target_dist)
                 + wall_term
                 + straight_weight * (xe / max(1e-6, length))
                 - smooth_weight * abs(k - prev_kappa))
        if best is None or score > best[0]:
            best = (score, poses, k)
    if best is None:
        return None, None
    return best[1], best[2]
