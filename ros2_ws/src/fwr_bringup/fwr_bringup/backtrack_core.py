"""Conservative, one-shot reverse path tracker for supervised trials."""
import math
from geometry_msgs.msg import Twist


def _clear(grid, x, y, occupied_limit=0):
    return _clear_problem(grid, x, y, occupied_limit=occupied_limit) is None


def _clear_problem(grid, x, y, with_location=False, occupied_limit=0):
    # Circumscribed disk is conservative at every chassis heading, including
    # the swept corners between stored poses.
    info = grid.info
    cx = int((x - info.origin.position.x) / info.resolution)
    cy = int((y - info.origin.position.y) / info.resolution)
    radius = math.hypot(.32, .235) + info.resolution
    hx = hy = math.ceil(radius / info.resolution)
    if cx - hx < 0 or cx + hx >= info.width or cy - hy < 0 or cy + hy >= info.height:
        return 'outside map'
    for yy in range(cy - hy, cy + hy + 1):
        for xx in range(cx - hx, cx + hx + 1):
            if math.hypot((xx + .5) * info.resolution + info.origin.position.x - x,
                          (yy + .5) * info.resolution + info.origin.position.y - y) > radius:
                continue
            value = grid.data[yy * info.width + xx]
            if value < 0:
                return (f'unknown at ({(xx + .5) * info.resolution + info.origin.position.x:.2f},'
                        f'{(yy + .5) * info.resolution + info.origin.position.y:.2f})'
                        if with_location else 'unknown')
            if value > occupied_limit:
                return (f'occupied at ({(xx + .5) * info.resolution + info.origin.position.x:.2f},'
                        f'{(yy + .5) * info.resolution + info.origin.position.y:.2f})'
                        if with_location else 'occupied')
    return None


class BacktrackController:
    segment_limit_m = .10
    total_limit_m = .30
    max_duration_s = 10.

    def __init__(self):
        self._attempted = False
        self._active = False
        self._path = []
        self._started = None
        self._last_progress_at = None
        self._last_xy = None
        self._distance = 0.
        self._segment_distance = 0.
        self._index = 0
        self.last_refusal = ''

    def begin(self, track, grid, now_s):
        self.last_refusal = ''
        if self._attempted:
            self.last_refusal = 'already attempted'
            return False
        self._attempted = True
        points = list(track.points)
        if (len(points) < 2 or track.frame_id != grid.header.frame_id or
                now_s - points[-1].at > .3):
            self.last_refusal = 'insufficient or stale track'
            return False
        path = [points[-1]]
        distance = 0.
        problem = _clear_problem(grid, path[0].x, path[0].y, with_location=True)
        if problem is not None:
            self.last_refusal = f'recorded path footprint not known free ({problem})'
            return False
        for point in reversed(points[:-1]):
            step = math.dist((path[-1].x, path[-1].y), (point.x, point.y))
            if step > .10:
                self.last_refusal = (f'track discontinuity {step:.3f} m '
                                     f'from ({path[-1].x:.2f},{path[-1].y:.2f}) '
                                     f'to ({point.x:.2f},{point.y:.2f})')
                return False
            if distance + step > self.total_limit_m:
                break
            problem = _clear_problem(grid, point.x, point.y, with_location=True)
            if problem is not None:
                # A previously driven pose can still be unknown in the live
                # map. Never include it; a shorter known-free suffix may be
                # sufficient for one supervised recovery segment.
                break
            path.append(point)
            distance += step
            if distance >= self.total_limit_m - .02:
                break
        if distance < .05:
            self.last_refusal = ('less than 0.05 m known-free recorded'
                                 + (f' ({problem})' if problem is not None else ''))
            return False
        self._path = path
        self._started = now_s
        self._last_progress_at = now_s
        self._last_xy = (path[0].x, path[0].y)
        self._active = True
        self._index = 1
        self._distance = 0.
        self._segment_distance = 0.
        return True

    def next_segment(self, now_s):
        self._segment_distance = 0.
        self._last_progress_at = now_s

    def cancel(self):
        self._active = False

    def update(self, odom, grid, sensors_fresh, fault, now_s,
               local_grid=None, map_to_local=None):
        zero = Twist()
        if not self._active:
            return zero, 'aborted'
        p = odom.pose.pose
        xy = (p.position.x, p.position.y)
        local_clear = True
        if local_grid is not None:
            if map_to_local is None:
                local_clear = False
            else:
                # Nav2's published OccupancyGrid maps inflation costs to
                # 1..98, inscribed cells to 99, and lethal cells to 100.
                # The swept disk already includes the chassis radius; keep
                # unknown/inscribed/lethal forbidden while allowing inflation.
                local_clear = all(_clear(local_grid, *map_to_local(q.x, q.y), occupied_limit=98)
                                  for q in self._path[self._index - 1:])
                local_clear = local_clear and _clear(local_grid, *map_to_local(*xy), occupied_limit=98)
        reasons = []
        if not local_clear:
            reasons.append('local costmap not clear')
        if not sensors_fresh:
            reasons.append('stale sensor or map')
        if fault:
            reasons.append(f'base fault {fault}')
        if odom.header.frame_id != grid.header.frame_id:
            reasons.append('frame mismatch')
        if now_s - self._started > self.max_duration_s:
            reasons.append('duration limit')
        if math.dist(xy, self._last_xy) > .25:
            reasons.append('pose discontinuity')
        problem = _clear_problem(grid, *xy, with_location=True)
        if problem is not None:
            reasons.append(f'current map footprint {problem}')
        if any(not _clear(grid, q.x, q.y) for q in self._path[self._index:]):
            reasons.append('remaining map path not clear')
        if reasons:
            self.last_refusal = ', '.join(reasons)
            self.cancel()
            return zero, 'aborted'
        moved = math.dist(xy, self._last_xy)
        if moved >= .008:
            self._distance += moved
            self._segment_distance += moved
            self._last_xy = xy
            self._last_progress_at = now_s
        if now_s - self._last_progress_at > 1.:
            self.last_refusal = 'no reverse odometry progress within 1.0 s'
            self.cancel()
            return zero, 'aborted'
        if self._distance >= self.total_limit_m:
            self._active = False
            return zero, 'done'
        if self._segment_distance >= .095:
            return zero, 'segment_done'
        while self._index < len(self._path) and math.dist(xy, (self._path[self._index].x, self._path[self._index].y)) < .025:
            self._index += 1
        if self._index >= len(self._path):
            self._active = False
            return zero, 'done'
        target = self._path[self._index]
        # Never skip more than one 0.10 m segment.
        if math.dist(xy, (target.x, target.y)) > self.segment_limit_m + .025:
            self.cancel()
            return zero, 'aborted'
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        desired = math.atan2(target.y - xy[1], target.x - xy[0]) + math.pi
        error = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))
        if abs(error) > .35:
            self.cancel()
            return zero, 'aborted'
        cmd = Twist()
        cmd.linear.x = -.06
        cmd.angular.z = max(-.15, min(.15, .8 * error))
        return cmd, 'running'
