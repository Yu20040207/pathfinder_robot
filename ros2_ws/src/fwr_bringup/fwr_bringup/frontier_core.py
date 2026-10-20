"""Reachable frontier selection with full rectangular footprint clearance."""

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FrontierGoal:
    x: float
    y: float
    yaw: float
    grid_index: int
    score: float


def choose_frontier(grid, robot, blocked: set[int], current: FrontierGoal | None) -> FrontierGoal | None:
    info = grid.info
    w, h, res = info.width, info.height, info.resolution
    if w <= 0 or h <= 0 or res <= 0 or len(grid.data) != w * h:
        return None
    ox, oy = info.origin.position.x, info.origin.position.y
    rx, ry = robot.pose.position.x, robot.pose.position.y
    sx, sy = int((rx - ox) / res), int((ry - oy) / res)
    if not (0 <= sx < w and 0 <= sy < h):
        return None

    def xy(i):
        return i % w, i // w

    def neighbors(i, diagonal=False):
        x, y = xy(i)
        offsets = ((1, 0), (-1, 0), (0, 1), (0, -1))
        if diagonal:
            offsets += ((1, 1), (1, -1), (-1, 1), (-1, -1))
        for dx, dy in offsets:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h:
                yield ny * w + nx

    # Cell centers plus conservative half-cell margin.
    hx = math.ceil((.32 + res / 2) / res)
    hy = math.ceil((.235 + res / 2) / res)
    def clear(i):
        x, y = xy(i)
        if x - hx < 0 or x + hx >= w or y - hy < 0 or y + hy >= h:
            return False
        return all(grid.data[yy * w + xx] == 0
                   for yy in range(y - hy, y + hy + 1)
                   for xx in range(x - hx, x + hx + 1))

    start = sy * w + sx
    if not clear(start):
        return None
    distance = {start: 0}
    queue = deque([start])
    while queue:
        i = queue.popleft()
        for j in neighbors(i):
            if j not in distance and clear(j):
                distance[j] = distance[i] + 1
                queue.append(j)

    # Reachable known-free goal cells can sit behind the frontier by a footprint radius.
    candidates = []
    for i, steps in distance.items():
        if i in blocked:
            continue
        x, y = xy(i)
        nearby_unknown = 0
        for dy in range(-hy - 2, hy + 3):
            for dx in range(-hx - 2, hx + 3):
                xx, yy = x + dx, y + dy
                if 0 <= xx < w and 0 <= yy < h and grid.data[yy * w + xx] == -1:
                    nearby_unknown += 1
        if not nearby_unknown:
            continue
        gain = min(nearby_unknown, 30)
        score = steps * res - .02 * gain
        yaw = math.atan2(y * res + oy - ry, x * res + ox - rx)
        candidates.append(FrontierGoal(ox + (x + .5) * res, oy + (y + .5) * res,
                                       yaw, i, score))
    if not candidates:
        return None
    best = min(candidates, key=lambda g: (g.score, g.grid_index))
    if current and current.grid_index in distance and current.grid_index not in blocked and clear(current.grid_index):
        old = next((g for g in candidates if g.grid_index == current.grid_index), None)
        if old and best.score >= old.score - .2 * max(abs(old.score), .1):
            return old
    return best
