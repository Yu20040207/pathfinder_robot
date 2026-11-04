"""视点规划核心（纯函数，不依赖 ROS，方便单元测试）。

设计要点（见 docs/plans/2026-11-04-auto-mapping-viewpoint-plan.md）：

1. 建图要的是"**看见**"，不是"走遍" —— 用射线投射判断可见性，成本远低于密扫。
2. **半径受限**：只在 R 米内选视点，天然实现"这间房没看完就不出门"
   （跨门的点必然在 R 外）。
3. **用栅格 BFS 的真实路径距离**排序，绝不用欧氏直线（否则会规划穿墙）。
4. 扩展阶段目标必须落在**已知自由区一侧**，不能打进未知区
   （静态层 track_unknown_space=false 会把未知当可通行 → 撞上看不见的墙）。
"""
from __future__ import annotations

import math
from collections import deque
from typing import Iterable, Sequence

import numpy as np

UNKNOWN_MAX = 0        # OccupancyGrid: -1 = unknown；其余 0..100 是代价占比


def free_mask(data: np.ndarray, width: int, height: int) -> np.ndarray:
    """已知自由格（OccupancyGrid 里 value == 0）。"""
    occ = np.asarray(data, dtype=np.int16).reshape(height, width)
    return occ == 0


def unknown_mask(data: np.ndarray, width: int, height: int) -> np.ndarray:
    occ = np.asarray(data, dtype=np.int16).reshape(height, width)
    return occ < 0


def blocked_mask(data: np.ndarray, width: int, height: int,
                 occupied_threshold: int = 65) -> np.ndarray:
    """障碍格（>= occupied_threshold 视为不可通行）。"""
    occ = np.asarray(data, dtype=np.int16).reshape(height, width)
    return occ >= occupied_threshold


def erode(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """方形结构元素腐蚀（不引入 scipy 依赖，用切片最大值实现）。"""
    if radius_cells <= 0:
        return mask.copy()
    out = mask.copy()
    h, w = mask.shape
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            shifted = np.zeros_like(mask)
            ys0, ys1 = max(0, dy), min(h, h + dy)
            xs0, xs1 = max(0, dx), min(w, w + dx)
            yd0, yd1 = max(0, -dy), min(h, h - dy)
            xd0, xd1 = max(0, -dx), min(w, w - dx)
            shifted[yd0:yd1, xd0:xd1] = mask[ys0:ys1, xs0:xs1]
            out &= shifted
    return out


def dilate(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """方形结构元素膨胀（与 erode 互补，仍不依赖 scipy）。"""
    if radius_cells <= 0:
        return mask.copy()
    out = mask.copy()
    h, w = mask.shape
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            shifted = np.zeros_like(mask)
            ys0, ys1 = max(0, dy), min(h, h + dy)
            xs0, xs1 = max(0, dx), min(w, w + dx)
            yd0, yd1 = max(0, -dy), min(h, h - dy)
            xd0, xd1 = max(0, -dx), min(w, w - dx)
            shifted[yd0:yd1, xd0:xd1] = mask[ys0:ys1, xs0:xs1]
            out |= shifted
    return out


def bfs_distance(passable: np.ndarray, start: tuple[int, int]) -> np.ndarray:
    """8 邻域 BFS 的最短路径距离（单位=格）。不可达 = -1。"""
    h, w = passable.shape
    dist = np.full((h, w), -1, dtype=np.int32)
    sy, sx = start
    if not (0 <= sy < h and 0 <= sx < w) or not passable[sy, sx]:
        return dist
    dist[sy, sx] = 0
    q = deque([(sy, sx)])
    nb = ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))
    while q:
        y, x = q.popleft()
        d = dist[y, x] + 1
        for dy, dx in nb:
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and passable[ny, nx] and dist[ny, nx] < 0:
                dist[ny, nx] = d
                q.append((ny, nx))
    return dist


def visible_cells(blocked: np.ndarray, unknown: np.ndarray, start: tuple[int, int],
                  sensor_cells: int, rays: int = 180,
                  fill_cells: int = 2,
                  stop_at_unknown: bool = False) -> np.ndarray:
    """站在 start 时雷达能"看见"的格子（扁平索引数组）。

    射线遇**障碍**停。`unknown` 表示"还没记录过的格子" —— 物理上并不挡光
    （雷达照样能看穿），默认**不**在未知处中断；只有需要"只统计已记录区域"时才
    传 `stop_at_unknown=True`。

    ⚠️ 必须做 `fill_cells` 膨胀：射线是离散的，6 m 处相邻射线间隔可达 0.5 m，
    只标记射线经过的格子会把扇面中间漏掉（实测覆盖率只剩 23%）。
    """
    h, w = blocked.shape
    y0, x0 = start
    seen: set[int] = set()
    for i in range(rays):
        ang = 2.0 * math.pi * i / rays
        dx, dy = math.cos(ang), math.sin(ang)
        for t in range(1, sensor_cells + 1):
            x = int(round(x0 + dx * t))
            y = int(round(y0 + dy * t))
            if not (0 <= x < w and 0 <= y < h):
                break
            if blocked[y, x]:
                break
            seen.add(y * w + x)
            if stop_at_unknown and unknown[y, x]:
                break
    idx = np.fromiter(seen, dtype=np.int64)
    if fill_cells > 0 and idx.size:
        mask = np.zeros(blocked.shape, dtype=bool)
        mask.ravel()[idx] = True
        mask = dilate(mask, fill_cells)
        idx = np.flatnonzero(mask.ravel())
    return idx


def candidate_viewpoints(drivable: np.ndarray, step_cells: int) -> list[tuple[int, int]]:
    """在可行驶区按网格撒候选视点。"""
    h, w = drivable.shape
    step = max(1, int(step_cells))
    return [(y, x) for y in range(step, h, step) for x in range(step, w, step)
            if drivable[y, x]]


def pick_next_viewpoint(candidates: Sequence[tuple[int, int]],
                        visibility: Sequence[np.ndarray],
                        covered: np.ndarray,
                        dist: np.ndarray,
                        radius_cells: int,
                        visited: Iterable[tuple[int, int]] = (),
                        min_gain_cells: int = 12,
                        home_dist: np.ndarray | None = None) -> int | None:
    """在半径内挑"性价比最高"（新增可见/路径长度）的视点，返回候选下标。

    - 半径限制用 home_dist（相对车本身）判断，dist 只用来算性价比；
      两者分开是为了支持"滚动续点"：选点要连贯（相对上一点），但不能越选越远。
    - 新增可见格少于 min_gain_cells 的不要（避免为 1 格跑很远）
    """
    visited_set = set(visited)
    best_i, best_score = None, 0.0
    for i, cell in enumerate(candidates):
        if cell in visited_set:
            continue
        d = int(dist[cell[0], cell[1]])
        if d < 0:
            continue
        anchor = home_dist if home_dist is not None else dist
        a = int(anchor[cell[0], cell[1]])
        if a < 0 or a > radius_cells:
            continue
        gain = int((~covered[visibility[i]]).sum())
        if gain < min_gain_cells:
            continue
        score = gain / (d + 1.0)          # 性价比：新增可见格 / 路径代价
        if score > best_score:
            best_score, best_i = score, i
    return best_i


def frontier_cells(free: np.ndarray, unknown: np.ndarray) -> list[tuple[int, int]]:
    """前沿：已知自由、且 8 邻域内有未知的格子。"""
    h, w = free.shape
    out = []
    for y in range(h):
        for x in range(w):
            if not free[y, x]:
                continue
            y0, y1 = max(0, y - 1), min(h, y + 2)
            x0, x1 = max(0, x - 1), min(w, x + 2)
            if unknown[y0:y1, x0:x1].any():
                out.append((y, x))
    return out


def expansion_target(frontiers: Sequence[tuple[int, int]],
                     drivable: np.ndarray,
                     dist: np.ndarray,
                     inset_cells: int = 8) -> tuple[int, int] | None:
    """选一个"离未知边界 inset_cells 格、且在可行驶区内、可达"的目标点。

    这样 Nav2 规划时不会把目标打进未知区（否则会穿过未知规划）。
    """
    h, w = drivable.shape
    best, best_d = None, None
    for (fy, fx) in frontiers:
        d_here = int(dist[fy, fx])
        # 从边界往里退：取可行驶区内、距离 >= inset 的最近点
        for r in range(inset_cells, inset_cells * 3 + 1):
            y0, y1 = max(0, fy - r), min(h, fy + r + 1)
            x0, x1 = max(0, fx - r), min(w, fx + r + 1)
            sub = drivable[y0:y1, x0:x1] & (dist[y0:y1, x0:x1] >= 0)
            if not sub.any():
                continue
            ys, xs = np.where(sub)
            i = int(np.argmin(np.abs(ys - (fy - y0)) + np.abs(xs - (fx - x0))))
            cand = (y0 + int(ys[i]), x0 + int(xs[i]))
            d = int(dist[cand[0], cand[1]])
            if best is None or d < best_d:
                best, best_d = cand, d
            break
        if d_here >= 0 and best is not None and best_d is not None and best_d <= d_here:
            break
    return best


def coverage_ratio(covered: np.ndarray, free: np.ndarray) -> float:
    total = int(free.sum())
    if total == 0:
        return 1.0
    return float(((covered & free).sum()) / total)


def perimeter_waypoints(free: np.ndarray, seed: tuple[int, int], radius_cells: int,
                        clearance_cells: int, step_cells: int) -> list[tuple[int, int]]:
    """房间"边缘一圈"的采样点（内缩 clearance 后的轮廓，按环绕顺序排列）。

    用户 2026-11-04 的思路：房间大致成型后，先沿边缘走一圈把墙确认清楚，再出门。
    中间乱挑"信息量最大"的点容易来回折返，沿轮廓走是一条闭合的环，不会折返。
    """
    h, w = free.shape
    yy, xx = np.ogrid[:h, :w]
    disk = (yy - seed[0]) ** 2 + (xx - seed[1]) ** 2 <= radius_cells ** 2
    room = free & disk
    if not room.any():
        return []
    inner = erode(room, max(1, clearance_cells))
    if not inner.any():
        inner = room
    ring = inner & ~erode(inner, 1)          # 轮廓（1 格厚）
    ys, xs = np.nonzero(ring)
    if ys.size == 0:
        return []
    # 按极角排序 → 得到环绕一圈的顺序（不会来回折返）
    cy, cx = float(ys.mean()), float(xs.mean())
    order = np.argsort(np.arctan2(ys - cy, xs - cx))
    pts = [(int(ys[i]), int(xs[i])) for i in order]
    # 等距抽样
    step = max(1, int(step_cells))
    out = []
    last = None
    for p_ in pts:
        if last is None or (p_[0] - last[0]) ** 2 + (p_[1] - last[1]) ** 2 >= step ** 2:
            out.append(p_)
            last = p_
    return out
