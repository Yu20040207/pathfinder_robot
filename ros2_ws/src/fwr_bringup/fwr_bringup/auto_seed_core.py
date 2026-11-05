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


def find_best_pose(occupied, resolution, origin_x, origin_y, bounds, points,
                   coarse_step=.3, coarse_yaw_deg=10., fine_step=.04,
                   fine_yaw_deg=1.0, max_points=80):
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
        k = int(np.argmin(d))
        if d[k] < best[0]:
            best = (float(d[k]), float(wx[k]), float(wy[k]), float(yaw))
    if best[0] > 9.0:
        return best
    # 粗搜索第一名附近做局部细化
    score, x, y, yaw = refine_pose(occupied, resolution, origin_x, origin_y, points,
                                   best[1], best[2], best[3],
                                   span_m=coarse_step, span_yaw_deg=coarse_yaw_deg,
                                   step_m=fine_step, step_yaw_deg=fine_yaw_deg)
    return (score, x, y, yaw)


def base_pose_from_laser(x, y, yaw, laser_offset_m=.21):
    """Convert a matched laser pose to the base_footprint pose."""
    base_yaw = yaw - math.pi
    return (x - laser_offset_m * math.cos(base_yaw),
            y - laser_offset_m * math.sin(base_yaw), base_yaw)


def refine_pose(occupied, resolution, origin_x, origin_y, points, x, y, yaw,
                span_m=.5, span_yaw_deg=15.0, step_m=.04, step_yaw_deg=1.0):
    """Local refinement around a prior pose (e.g. the last known checkpoint)."""
    distance = distance_field(occupied, resolution)
    best = (9.9, x, y, yaw)
    yaw_span = math.radians(span_yaw_deg)
    for dx in np.arange(-span_m, span_m + 1e-9, step_m):
        for dy in np.arange(-span_m, span_m + 1e-9, step_m):
            for dth in np.arange(-yaw_span, yaw_span + 1e-9, math.radians(step_yaw_deg)):
                value = score_pose(distance, resolution, origin_x, origin_y,
                                   points, x + dx, y + dy, yaw + dth)
                if value < best[0]:
                    best = (value, x + dx, y + dy, yaw + dth)
    return best
