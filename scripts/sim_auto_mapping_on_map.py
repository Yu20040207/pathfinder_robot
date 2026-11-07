#!/usr/bin/env python3
"""Offline map-replay simulation for the auto-mapping viewpoint planner.

The selected occupancy map is treated as GROUND TRUTH.  The simulated robot
starts with only a small known patch, ray-casts a 6 m lidar at each scan pose,
reveals free/occupied cells, and uses fwr_bringup.viewpoint_core to choose the
next viewpoint.  It is a coverage/path-planning simulation, not Gazebo physics.

Example:
    .venv/bin/python scripts/sim_auto_mapping_on_map.py \
        --map ros2_ws/src/fwr_bringup/maps/real_map_manual_1101.yaml \
        --start 0 -2 --max-steps 80
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'fwr_bringup'))
from fwr_bringup import viewpoint_core as vc  # noqa: E402

NEIGHBORS = ((-1, 0), (1, 0), (0, -1), (0, 1),
             (-1, -1), (-1, 1), (1, -1), (1, 1))


def load_map(path: Path):
    meta = yaml.safe_load(path.read_text(encoding='utf-8'))
    image = Image.open(path.parent / meta['image']).convert('L')
    values = np.asarray(image, dtype=np.uint8)
    free = values == 254
    blocked = values == 0
    unknown = values == 205
    # For ground-truth ray casting, unknown/out-of-map must not be traversed.
    return meta, free, blocked, unknown


def cell_of(meta, x: float, y: float) -> tuple[int, int]:
    """World (x,y) -> array cell (row=y, col=x)."""
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    return int((y - oy) // res), int((x - ox) // res)


def world_of(meta, cell: tuple[int, int]) -> tuple[float, float]:
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    return ox + (cell[1] + 0.5) * res, oy + (cell[0] + 0.5) * res


def reachable_free(free: np.ndarray, start: tuple[int, int]) -> np.ndarray:
    h, w = free.shape
    seen = np.zeros_like(free, dtype=bool)
    sy, sx = start
    if not (0 <= sy < h and 0 <= sx < w and free[sy, sx]):
        return seen
    seen[sy, sx] = True
    q = deque([(sy, sx)])
    while q:
        y, x = q.popleft()
        for dy, dx in NEIGHBORS:
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and free[ny, nx] and not seen[ny, nx]:
                seen[ny, nx] = True
                q.append((ny, nx))
    return seen


def cast_ground_truth(blocked: np.ndarray, start: tuple[int, int],
                      sensor_cells: int, rays: int):
    """Return (seen free flat indices, seen blocked flat indices)."""
    h, w = blocked.shape
    sy, sx = start
    free_idx: set[int] = set()
    blocked_idx: set[int] = set()
    for i in range(rays):
        ang = 2.0 * math.pi * i / rays
        dx, dy = math.cos(ang), math.sin(ang)
        for t in range(1, sensor_cells + 1):
            x = int(round(sx + dx * t))
            y = int(round(sy + dy * t))
            if not (0 <= x < w and 0 <= y < h):
                break
            if blocked[y, x]:
                blocked_idx.add(y * w + x)
                break
            free_idx.add(y * w + x)
    return np.fromiter(free_idx, dtype=np.int64), np.fromiter(blocked_idx, dtype=np.int64)


def bfs_path(passable: np.ndarray, start: tuple[int, int],
             goal: tuple[int, int]) -> list[tuple[int, int]]:
    h, w = passable.shape
    if not (0 <= goal[0] < h and 0 <= goal[1] < w and passable[goal[0], goal[1]]):
        return []
    q = deque([start])
    parent: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
    while q:
        cur = q.popleft()
        if cur == goal:
            break
        y, x = cur
        for dy, dx in NEIGHBORS:
            nxt = (y + dy, x + dx)
            if (0 <= nxt[0] < h and 0 <= nxt[1] < w and
                    passable[nxt] and nxt not in parent):
                parent[nxt] = cur
                q.append(nxt)
    if goal not in parent:
        return []
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = parent[cur]
    path.reverse()
    return path


def frontier_clusters(frontiers, shape):
    """Group adjacent frontier cells; one target per cluster prevents per-cell ping-pong."""
    h, w = shape
    remaining = set(frontiers)
    clusters = []
    while remaining:
        seed = remaining.pop()
        q = deque([seed])
        cluster = [seed]
        while q:
            y, x = q.popleft()
            for dy, dx in NEIGHBORS:
                nxt = (y + dy, x + dx)
                if nxt in remaining:
                    remaining.remove(nxt)
                    cluster.append(nxt)
                    q.append(nxt)
        clusters.append(cluster)
    return clusters


def nearest_reachable_frontier(frontiers, drivable, dist, excluded=None, min_d=2):
    excluded = excluded or set()
    best = None
    best_d = 10 ** 9
    for cell in frontiers:
        if cell in excluded:
            continue
        d = int(dist[cell])
        # d=0 means the "frontier" is the robot's own cell: choosing it would
        # produce an empty path and an endless loop (seen in first run).
        if min_d <= d < best_d:
            best, best_d = cell, d
    return best


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--start', nargs=2, type=float, metavar=('X', 'Y'),
                    default=(0.0, -2.0))
    ap.add_argument('--max-steps', type=int, default=80)
    ap.add_argument('--sensor-range', type=float, default=6.0)
    ap.add_argument('--rays', type=int, default=180)
    ap.add_argument('--scan-stride', type=int, default=5)
    ap.add_argument('--candidate-step', type=float, default=1.0)
    ap.add_argument('--clearance', type=float, default=0.25)
    ap.add_argument('--radius', type=float, default=4.0)
    ap.add_argument('--radius-max', type=float, default=8.0)
    ap.add_argument('--min-gain', type=int, default=400)
    ap.add_argument('--min-goal', type=float, default=1.2)
    ap.add_argument('--turn-weight', type=float, default=0.4)
    ap.add_argument('--cover-target', type=float, default=0.95)
    ap.add_argument('--out', default='/tmp/fwr_auto_map_sim.json')
    ap.add_argument('--rooms', default='', help='rooms.yaml for per-room coverage')
    ap.add_argument('--room-cover-target', type=float, default=0.95)
    ap.add_argument('--room-radius', type=float, default=2.0)
    ap.add_argument('--stagnant-limit', type=int, default=30)
    args = ap.parse_args()

    map_path = Path(args.map).resolve()
    meta, free_gt, blocked_gt, unknown_gt = load_map(map_path)
    res = float(meta['resolution'])
    h, w = free_gt.shape
    start = cell_of(meta, *args.start)
    if not (0 <= start[0] < h and 0 <= start[1] < w and free_gt[start]):
        print(f'start {args.start} -> cell {start} is not known free', file=sys.stderr)
        return 2
    reachable = reachable_free(free_gt, start)
    reachable_count = int(reachable.sum())
    print(f'map {map_path.name}: {w}x{h} @ {res:.3f} m, free {int(free_gt.sum())}, '
          f'reachable-from-start {reachable_count}')
    if reachable_count == 0:
        return 2

    sensor_cells = int(round(args.sensor_range / res))
    step_cells = max(1, int(round(args.candidate_step / res)))
    clearance_cells = max(1, int(round(args.clearance / res)))
    radius_cells = max(1, int(round(args.radius / res)))
    radius_max_cells = max(radius_cells, int(round(args.radius_max / res)))
    min_goal_cells = int(round(args.min_goal / res))

    known_free = np.zeros_like(free_gt, dtype=bool)
    known_blocked = np.zeros_like(blocked_gt, dtype=bool)
    visited: set[tuple[int, int]] = set()
    bad_targets: set[tuple[int, int]] = set()
    visibility_cache: dict[tuple[int, int], np.ndarray] = {}

    def scan(cell):
        f_idx, b_idx = cast_ground_truth(blocked_gt, cell, sensor_cells, args.rays)
        f_idx = f_idx[(f_idx >= 0) & (f_idx < free_gt.size)]
        b_idx = b_idx[(b_idx >= 0) & (b_idx < blocked_gt.size)]
        # Unknown/occupied cells are not free even if a ray passed through them.
        f_idx = f_idx[free_gt.ravel()[f_idx]]
        b_idx = b_idx[blocked_gt.ravel()[b_idx]]
        known_free.ravel()[f_idx] = True
        known_blocked.ravel()[b_idx] = True

    def visibility(cell):
        if cell not in visibility_cache:
            f_idx, _ = cast_ground_truth(blocked_gt, cell, sensor_cells, args.rays)
            f_idx = f_idx[(f_idx >= 0) & (f_idx < free_gt.size)]
            f_idx = f_idx[free_gt.ravel()[f_idx]]
            mask = np.zeros(free_gt.size, dtype=bool)
            mask[f_idx] = True
            visibility_cache[cell] = mask
        return visibility_cache[cell]

    def coverage():
        return float((known_free & reachable).sum()) / reachable_count

    rooms = []
    rooms_path = Path(args.rooms) if args.rooms else map_path.parent / 'rooms.yaml'
    if rooms_path.is_file():
        rooms = yaml.safe_load(rooms_path.read_text(encoding='utf-8')).get('rooms', [])

    def room_stats():
        out = {}
        r_cells = int(round(args.room_radius / res))
        yy, xx = np.ogrid[:h, :w]
        for room in rooms:
            name = str(room.get('name', '?'))
            cy, cx = cell_of(meta, float(room['x']), float(room['y']))
            disk = (yy - cy) ** 2 + (xx - cx) ** 2 <= r_cells ** 2
            gt_room = free_gt & reachable & disk
            total = int(gt_room.sum())
            covered = int((known_free & gt_room).sum())
            out[name] = {'covered': covered, 'total': total,
                         'ratio': (covered / total) if total else 1.0}
        return out

    scan(start)
    robot = start
    path_total = 0
    scans = 1
    last_cov = coverage()
    stagnant_steps = 0
    revisit_events = 0
    recent_targets: list[tuple[int, int]] = []
    visited_rooms: set[str] = set()
    last_heading = None
    current_heading = None
    turn_flips = 0
    stop_reason = 'max-steps'
    decisions = 0
    t0 = time.time()

    for step in range(1, args.max_steps + 1):
        free = known_free
        blocked = known_blocked
        unknown = ~(free | blocked)
        drivable = vc.erode(free, clearance_cells)
        for relaxed in (max(1, clearance_cells // 2), 1, 0):
            if int(drivable.sum()) >= 200:
                break
            drivable = vc.erode(free, relaxed) if relaxed else free.copy()
        # The robot cell can be eroded out near a wall; planner has the same
        # snap rule.  Without it BFS returns all -1 and the mission looks stuck.
        if not drivable[robot]:
            ys, xs = np.nonzero(drivable)
            if ys.size == 0:
                print(f'step {step:03d}: no drivable cells')
                break
            i = int(np.argmin((ys - robot[0]) ** 2 + (xs - robot[1]) ** 2))
            robot = (int(ys[i]), int(xs[i]))
            scan(robot)
        dist = vc.bfs_distance(drivable, robot)
        # Candidate list from all currently drivable grid cells.
        candidates = vc.candidate_viewpoints(drivable, step_cells)
        if not candidates:
            candidates = vc.candidate_viewpoints(drivable, 1)
        target, label, target_gain = None, '', 0
        path_space = drivable
        if candidates:
            visibility_arrays = [visibility(c) for c in candidates]
            radius = radius_cells
            while radius <= radius_max_cells and target is None:
                best_i, best_score = None, 0.0
                covered_flat = known_free.ravel()
                for i, cell in enumerate(candidates):
                    if cell in visited:
                        continue
                    d = int(dist[cell])
                    if d < min_goal_cells:
                        continue
                    if d <= 0 or d > radius:
                        continue
                    gain = int((~covered_flat[visibility_arrays[i]]).sum())
                    if gain < args.min_gain:
                        continue
                    score = gain / (d + 1.0)
                    hdg = math.atan2(cell[0] - robot[0], cell[1] - robot[1])
                    if current_heading is not None:
                        delta = (hdg - current_heading + math.pi) % (2 * math.pi) - math.pi
                        # Prefer continuing the current travel direction.  This is
                        # the simulation-side analogue of the planner's rolling
                        # waypoint chain; it prevents left/right ping-pong.
                        score *= max(0.15, 1.0 - args.turn_weight * abs(delta) / math.pi)
                    if score > best_score:
                        best_i, best_score = i, score
                if best_i is not None:
                    target = candidates[best_i]
                    label = 'viewpoint'
                    target_gain = int((~covered_flat[visibility_arrays[best_i]]).sum())
                    break
                radius *= 2
        selected_cluster = None
        if target is None:
            frontiers = vc.frontier_cells(free, unknown)
            if frontiers:
                # Pick a whole frontier cluster, not a random adjacent cell.
                best, best_score = None, -1.0
                free_dist = None
                for cluster in frontier_clusters(frontiers, (h, w)):
                    if any(c in bad_targets for c in cluster):
                        continue
                    members = [c for c in cluster if int(dist[c]) >= 2]
                    use_dist = dist
                    if not members:
                        if free_dist is None:
                            free_dist = vc.bfs_distance(free, robot)
                        use_dist = free_dist
                        members = [c for c in cluster if int(use_dist[c]) >= 2]
                    if not members:
                        continue
                    target = min(members, key=lambda c: int(use_dist[c]))
                    d = int(use_dist[target])
                    score = len(cluster) / (d + 1.0)
                    if score > best_score:
                        best, best_score = target, score
                        selected_cluster = cluster
                        if use_dist is free_dist:
                            path_space = free.copy()
                        else:
                            path_space = drivable
                target = best
                if target is not None:
                    label = 'frontier-cluster'
        if target is None:
            # Strict 0.25 m erosion can remove every frontier cell at a narrow
            # doorway.  Fall back to the known-free mask (still BFS, never a
            # straight line) so the simulation measures coverage rather than
            # stopping at the first doorway.  Actual planner should adopt a
            # similarly bounded fallback only after review.
            # Use the known-free mask itself: erode(free,1) would remove
            # exactly the frontier cells (they are on the unknown boundary).
            relaxed = free.copy()
            dist_free = vc.bfs_distance(relaxed, robot)
            target = nearest_reachable_frontier(
                frontiers, relaxed, dist_free, excluded=bad_targets, min_d=2)
            if target is not None:
                path_space = relaxed
                dist = dist_free
                label = 'frontier-free'
        if target is None:
            freach = sum(1 for c in frontiers if int(dist[c]) >= 2)
            maxgain = 0
            if candidates:
                maxgain = max(int((~known_free.ravel()[visibility(c)]).sum()) for c in candidates)
            print(f'debug free={int(free.sum())} blocked={int(blocked.sum())} '
                  f'unknown={int(unknown.sum())} drivable={int(drivable.sum())} '
                  f'candidates={len(candidates)} frontiers={len(frontiers)} '
                  f'frontiers_reachable={freach} maxgain={maxgain} '
                  f'robot={robot} drivable_robot={bool(drivable[robot])} bad={len(bad_targets)}')
            print(f'step {step:03d}: no reachable viewpoint/frontier, done')
            break
        # Heading reversal monitor: catches A->B->A target ping-pong.
        heading = math.atan2(target[1] - robot[1], target[0] - robot[0])
        if last_heading is not None:
            delta_heading = (heading - last_heading + math.pi) % (2 * math.pi) - math.pi
            if abs(delta_heading) > 0.8:
                turn_flips += 1
        last_heading = heading
        path = bfs_path(path_space, robot, target)
        if len(path) <= 1:
            visited.add(target)
            bad_targets.add(target)
            print(f'step {step:03d}: target {target} already at robot/unreachable, skip')
            continue
        cov_before = coverage()
        decisions += 1
        # Move along the path, scanning periodically, as the real robot would.
        for i, cell in enumerate(path):
            if i % args.scan_stride == 0 or cell == target:
                scan(cell)
                scans += 1
        path_total += len(path) - 1
        if len(path) > 1:
            current_heading = math.atan2(path[-1][0] - path[-2][0],
                                          path[-1][1] - path[-2][1])
        # Record room visits along the simulated route (2 m arrival radius).
        for room in rooms:
            name = str(room.get('name', '?'))
            ry, rx = cell_of(meta, float(room['x']), float(room['y']))
            rr = int(round(1.5 / res))
            if any((c[0] - ry) ** 2 + (c[1] - rx) ** 2 <= rr * rr for c in path):
                visited_rooms.add(name)
        robot = target
        visited.add(target)
        cov = coverage()
        if label.startswith('frontier') and cov <= cov_before + 0.0001:
            # Arriving at this frontier revealed no new free space (wall corner /
            # already-seen strip). Blacklist its neighborhood, otherwise the
            # same frontier is selected forever.
            bad_targets.add(target)
            if selected_cluster:
                bad_targets.update(selected_cluster)
            r = int(round(0.8 / res))
            ty, tx = target
            for yy in range(max(0, ty - r), min(h, ty + r + 1)):
                for xx in range(max(0, tx - r), min(w, tx + r + 1)):
                    if (yy - ty) ** 2 + (xx - tx) ** 2 <= r * r:
                        bad_targets.add((yy, xx))
            print(f'step {step:03d}: {label:9s} target=({target[1]},{target[0]}) '
                  f'NO_GAIN -> blacklist, cov={cov*100:5.1f}%')
            continue
        print(f'step {step:03d}: {label:9s} target=({target[1]},{target[0]}) '
              f'path={len(path)-1:3d} gain={target_gain:5d} cov={cov*100:5.1f}%')
        delta = cov - last_cov
        last_cov = cov
        if delta < 0.0002:
            stagnant_steps += 1
        else:
            stagnant_steps = 0
        # Revisit/near-revisit monitor: count targets close to a recent target.
        for old in recent_targets[-8:]:
            if (old[0] - target[0]) ** 2 + (old[1] - target[1]) ** 2 <= int(round(0.8 / res)) ** 2:
                revisit_events += 1
                break
        recent_targets.append(target)
        if len(recent_targets) > 20:
            recent_targets.pop(0)
        rooms_now = room_stats()
        if rooms_now:
            print('  rooms ' + ' '.join(f'{k}={v["ratio"]*100:.0f}%' for k, v in rooms_now.items()))
        if rooms_now and all(v['ratio'] >= args.room_cover_target for v in rooms_now.values()):
            stop_reason = 'all_rooms'
            print(f'all rooms >= {args.room_cover_target*100:.0f}%')
            break
        if stagnant_steps >= args.stagnant_limit:
            stop_reason = 'stagnated'
            print(f'coverage stagnated for {stagnant_steps} steps, stop')
            break
        if cov >= args.cover_target:
            stop_reason = 'coverage_target'
            print(f'coverage target reached: {cov*100:.1f}%')
            break

    cov = coverage()
    result = {
        'map': str(map_path),
        'start_cell': [start[1], start[0]],
        'reachable_free_cells': reachable_count,
        'known_free_cells': int(known_free.sum()),
        'coverage': round(cov, 4),
        'steps': step,
        'decisions': decisions,
        'scans': scans,
        'path_cells': path_total,
        'path_m': round(path_total * res, 2),
        'elapsed_s': round(time.time() - t0, 2),
        'cover_target': args.cover_target,
        'room_cover_target': args.room_cover_target,
        'stop_reason': stop_reason,
        'stagnant_steps': stagnant_steps,
        'revisit_events': revisit_events,
        'turn_flips': turn_flips,
        'turn_weight': args.turn_weight,
        'rooms_visited': sorted(visited_rooms),
        'rooms': room_result if 'room_result' in locals() else {},
    }

    room_result = {}
    rooms_path = Path(args.rooms) if args.rooms else map_path.parent / 'rooms.yaml'
    if rooms_path.is_file():
        rooms = yaml.safe_load(rooms_path.read_text(encoding='utf-8')).get('rooms', [])
        for room in rooms:
            name = str(room.get('name', '?'))
            cy, cx = cell_of(meta, float(room['x']), float(room['y']))
            radius_cells = int(round(2.0 / res))
            yy, xx = np.ogrid[:h, :w]
            disk = (yy - cy) ** 2 + (xx - cx) ** 2 <= radius_cells ** 2
            gt_room = free_gt & reachable & disk
            total = int(gt_room.sum())
            covered = int((known_free & gt_room).sum())
            frac = (covered / total) if total else 1.0
            room_result[name] = {'covered': covered, 'total': total,
                                 'ratio': round(frac, 4)}
        result['rooms'] = room_result
        print('room coverage:', json.dumps(room_result, ensure_ascii=False))

    Path(args.out).write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if cov >= args.cover_target else 1


if __name__ == '__main__':
    raise SystemExit(main())
