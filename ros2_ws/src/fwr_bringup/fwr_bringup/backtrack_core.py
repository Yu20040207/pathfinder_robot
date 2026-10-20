"""Conservative, one-shot reverse path tracker for supervised trials."""
import math
from geometry_msgs.msg import Twist


def _clear(grid, x, y):
    info = grid.info
    cx = int((x - info.origin.position.x) / info.resolution)
    cy = int((y - info.origin.position.y) / info.resolution)
    hx = math.ceil((.32 + info.resolution / 2) / info.resolution)
    hy = math.ceil((.235 + info.resolution / 2) / info.resolution)
    if cx - hx < 0 or cx + hx >= info.width or cy - hy < 0 or cy + hy >= info.height:
        return False
    return all(grid.data[yy * info.width + xx] == 0
               for yy in range(cy - hy, cy + hy + 1)
               for xx in range(cx - hx, cx + hx + 1))


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
        self._index = 0

    def begin(self, track, grid, now_s):
        if self._attempted:
            return False
        self._attempted = True
        points = list(track.points)
        if (len(points) < 2 or track.frame_id != grid.header.frame_id or
                now_s - points[-1].at > .3):
            return False
        path = [points[-1]]
        distance = 0.
        for point in reversed(points[:-1]):
            step = math.dist((path[-1].x, path[-1].y), (point.x, point.y))
            if step > .10:
                return False
            if distance + step > self.total_limit_m:
                break
            path.append(point)
            distance += step
            if distance >= self.total_limit_m - .02:
                break
        if distance < .05 or not all(_clear(grid, p.x, p.y) for p in path):
            return False
        self._path = path
        self._started = now_s
        self._last_progress_at = now_s
        self._last_xy = (path[0].x, path[0].y)
        self._active = True
        self._index = 1
        return True

    def cancel(self):
        self._active = False

    def update(self, odom, grid, sensors_fresh, fault, now_s):
        zero = Twist()
        if not self._active:
            return zero, 'aborted'
        p = odom.pose.pose
        xy = (p.position.x, p.position.y)
        if (not sensors_fresh or fault or odom.header.frame_id != grid.header.frame_id or
                now_s - self._started > self.max_duration_s or
                math.dist(xy, self._last_xy) > .25 or
                not _clear(grid, *xy) or
                any(not _clear(grid, q.x, q.y) for q in self._path[self._index:])):
            self.cancel()
            return zero, 'aborted'
        moved = math.dist(xy, self._last_xy)
        if moved >= .008:
            self._distance += moved
            self._last_xy = xy
            self._last_progress_at = now_s
        if now_s - self._last_progress_at > 1.:
            self.cancel()
            return zero, 'aborted'
        if self._distance >= self.total_limit_m:
            self._active = False
            return zero, 'done'
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
