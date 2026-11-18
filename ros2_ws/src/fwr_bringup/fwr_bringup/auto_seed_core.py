"""Global scan-to-map match used to seed AMCL after every stack start."""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt


def occupancy_from_grid(data, width, height):
    """Return a boolean occupied mask from nav_msgs OccupancyGrid data."""
    grid = np.array(data, dtype=np.int16).reshape(height, width)
    return grid >= 65


def distance_field(occupied, resolution):
    return distance_transform_edt(~occupied) * resolution


def score_pose(distance, resolution, origin_x, origin_y, points, x, y, yaw):
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    height, width = distance.shape
    xf = cos_yaw * points[:, 0] - sin_yaw * points[:, 1] + x
    yf = sin_yaw * points[:, 0] + cos_yaw * points[:, 1] + y
    ix = ((xf - origin_x) / resolution).astype(int)
    # ⚠️ 2026-11-05 修：OccupancyGrid 的第 0 行就是 origin_y（**不需要翻转**）。
    # 这里原先是 `height - 1 - (...)`（按图片行序处理）→ 地图被上下镜像 →
    # 自动播种把车放到"镜像位置"（实测每次启动落在相距 7~13 m 的不同点，
    # 而正确约定下的最佳匹配与用户手动摆的位置只差 5 cm）。
    iy = ((yf - origin_y) / resolution).astype(int)
    inside = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    if inside.sum() < len(points) * 0.5:
        return 9.9
    return float(distance[iy[inside], ix[inside]].mean())


def clearance_mask(grid_values, resolution, clearance_m=0.10, free_value=0):
    """返回"车能停在这里"的布尔掩码（H×W）。

    要求该格及其 ±clearance 邻域**全部是空闲格**，与 ``_pose_is_sane`` 的判定一致
    （地图边缘也算不合法）。

    2026-11-18 现场：地图上有多处几乎同分的匹配（走廊对称 + 跨墙），
    全局搜索只取唯一最优，取到了墙里 → 事后检查拒绝 → 没有位姿 → 播不了种。
    与其事后拒绝，不如**搜索时就不让候选落在停不进去的位置**。
    """
    values = np.asarray(grid_values)
    free = values == free_value
    k = max(1, int(round(clearance_m / resolution)))
    from scipy.ndimage import binary_erosion
    return binary_erosion(free, structure=np.ones((2 * k + 1, 2 * k + 1), dtype=bool),
                          border_value=0)


def find_best_pose(occupied, resolution, origin_x, origin_y, bounds, points,
                   coarse_step=.3, coarse_yaw_deg=10., fine_step=.04,
                   fine_yaw_deg=1.0, max_points=80, valid_mask=None,
                   base_tf=(0.0, 0.0, 0.0)):
    """Coarse-to-fine global search. Returns (score, x, y, yaw_rad).

    ``bounds`` is (min_x, max_x, min_y, max_y) in map coordinates.

    ⚠️ 2026-11-05：原实现是**逐位姿 Python 循环**（全图 46x30 m、0.2 m 网格、5° 偏航
    ≈ 250 万个候选 × 每点一次函数调用）——实际跑不完，于是"一键重新定位"按下去以后
    永远没有结果。现在改为：
      · 候选位置只在**空闲格**上取（再按 coarse_step 抽稀）
      · 扫描点抽稀到 max_points
      · 每个偏航角一次性**向量化**算完所有候选
    实测同一张图 1~2 s 出结果。
    """
    distance = distance_field(occupied, resolution)
    height, width = occupied.shape
    free = ~occupied
    step_px = max(1, int(round(coarse_step / resolution)))
    ys, xs = np.nonzero(free[::step_px, ::step_px])
    ys = ys * step_px
    xs = xs * step_px
    if len(xs) == 0:
        return (9.9, 0.0, 0.0, 0.0)
    wx = origin_x + xs * resolution
    wy = origin_y + ys * resolution
    pts = points
    if len(pts) > max_points:
        idx = np.linspace(0, len(pts) - 1, max_points).astype(int)
        pts = pts[idx]
    n = len(pts)

    best = (9.9, 0.0, 0.0, 0.0)
    for yaw in np.radians(np.arange(-180.0, 180.0, coarse_yaw_deg)):
        c, s = math.cos(yaw), math.sin(yaw)
        px = c * pts[:, 0] - s * pts[:, 1]
        py = s * pts[:, 0] + c * pts[:, 1]
        ix = ((wx[:, None] + px[None, :] - origin_x) / resolution).astype(np.int32)
        iy = ((wy[:, None] + py[None, :] - origin_y) / resolution).astype(np.int32)
        inside = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
        counts = inside.sum(axis=1)
        ok = counts >= n * 0.9                 # 至少 90% 的点落在地图内
        if not ok.any():
            continue
        ixc = np.clip(ix, 0, width - 1)
        iyc = np.clip(iy, 0, height - 1)
        d = np.where(inside, distance[iyc, ixc], 0.0).sum(axis=1) / np.maximum(counts, 1)
        d = np.where(ok, d, 9.9)
        if valid_mask is not None:
            # ⚠️ 掩码必须按**车体位置**判断，不能按雷达位置：
            # 前雷达装在车头前方 0.21 m，雷达"站得下"不等于车"停得下"。
            # 之前按雷达位置过滤，结果搜索仍给出车体嵌在墙里的位姿。
            tx, ty, tyaw = base_tf
            byaw = yaw - tyaw
            c2, s2 = math.cos(byaw), math.sin(byaw)
            bxw = wx - (c2 * tx - s2 * ty)
            byw = wy - (s2 * tx + c2 * ty)
            mx = np.clip(((bxw - origin_x) / resolution).astype(np.int32), 0, width - 1)
            my = np.clip(((byw - origin_y) / resolution).astype(np.int32), 0, height - 1)
            d = np.where(valid_mask[my, mx], d, 9.9)
        k = int(np.argmin(d))
        if d[k] < best[0]:
            best = (float(d[k]), float(wx[k]), float(wy[k]), float(yaw))
    if best[0] > 9.0:
        return best
    # 粗搜索第一名附近做局部细化
    score, x, y, yaw = refine_pose(occupied, resolution, origin_x, origin_y, points,
                                   best[1], best[2], best[3],
                                   span_m=coarse_step, span_yaw_deg=coarse_yaw_deg,
                                   step_m=fine_step, step_yaw_deg=fine_yaw_deg,
                                   valid_mask=valid_mask, base_tf=base_tf)
    return (score, x, y, yaw)


def base_pose_from_laser(x, y, yaw, laser_offset_m=.21):
    """Convert a matched laser pose to the base_footprint pose."""
    base_yaw = yaw - math.pi
    return (x - laser_offset_m * math.cos(base_yaw),
            y - laser_offset_m * math.sin(base_yaw), base_yaw)


def pose_laser_to_base(x, y, yaw, tx, ty, tyaw):
    """把"匹配得到的雷达位姿"换算成 base 位姿（通用的 2D 刚体变换）。

    (tx, ty, tyaw) 是 **base_footprint ← 扫描坐标系** 的平面变换（用 tf2 取）。

    推导：匹配位姿满足 p_map = R(yaw)·p_scan + (x, y)；
    p_base = R(tyaw)·p_scan + (tx, ty) → p_scan = R(-tyaw)·(p_base - t)。
    代回并整理得
        base_yaw = yaw - tyaw
        base_pos = (x, y) - R(base_yaw)·(tx, ty)

    历史背景（2026-11-18）：原来这里写死 `yaw - pi` 和 `-0.21 m`，只对"前雷达单路、
    倒装 180°、装在 base 前方 0.21 m"成立。改用扫描合并节点（/scan_merged，
    frame=base_link，0°=车头）后这两个修正都变成了额外的 180° 旋转和 21 cm 偏移，
    但残差是在换算**前**算的，日志照样显示"残差 0.042 m 很漂亮"，实际位姿是错的。
    现在改为从 TF 取真实外参，任何扫描坐标系都适用。
    """
    base_yaw = yaw - tyaw
    c, sn = math.cos(base_yaw), math.sin(base_yaw)
    return (x - (c * tx - sn * ty),
            y - (sn * tx + c * ty),
            base_yaw)


def refine_pose(occupied, resolution, origin_x, origin_y, points, x, y, yaw,
                span_m=.5, span_yaw_deg=15.0, step_m=.04, step_yaw_deg=1.0,
                valid_mask=None, base_tf=(0.0, 0.0, 0.0)):
    """Local refinement around a prior pose (e.g. the last known checkpoint).

    ``valid_mask`` 给了就**只在车能停进去的位置**上搜索（细化不能把位姿挪进墙里）。
    """
    distance = distance_field(occupied, resolution)
    best = (9.9, x, y, yaw)
    height, width = occupied.shape

    tx, ty, tyaw = base_tf

    def _ok(px, py, pyaw):
        """按该偏航角对应的**车体位置**判断能不能停。"""
        if valid_mask is None:
            return True
        byaw = pyaw - tyaw
        c2, s2 = math.cos(byaw), math.sin(byaw)
        bx = px - (c2 * tx - s2 * ty)
        by = py - (s2 * tx + c2 * ty)
        ix = int((bx - origin_x) / resolution)
        iy = int((by - origin_y) / resolution)
        return 0 <= ix < width and 0 <= iy < height and bool(valid_mask[iy, ix])

    yaw_span = math.radians(span_yaw_deg)
    for dx in np.arange(-span_m, span_m + 1e-9, step_m):
        for dy in np.arange(-span_m, span_m + 1e-9, step_m):
            for dth in np.arange(-yaw_span, yaw_span + 1e-9, math.radians(step_yaw_deg)):
                if not _ok(x + dx, y + dy, yaw + dth):
                    continue
                value = score_pose(distance, resolution, origin_x, origin_y,
                                   points, x + dx, y + dy, yaw + dth)
                if value < best[0]:
                    best = (value, x + dx, y + dy, yaw + dth)
    return best
