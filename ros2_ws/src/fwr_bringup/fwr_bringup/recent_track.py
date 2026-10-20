"""Short, timestamped path of actual poses; never treats it as live sensing."""
from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TrackPoint:
    x: float
    y: float
    yaw: float
    at: float


class RecentTrack:
    def __init__(self, max_age_s=5., max_length_m=1.):
        self.points = deque()
        self.max_age_s = max_age_s
        self.max_length_m = max_length_m
        self.frame_id = None

    def observe(self, odom, now_s):
        if self.frame_id is None:
            self.frame_id = odom.header.frame_id
        if odom.header.frame_id != self.frame_id:
            self.points.clear()
            self.frame_id = odom.header.frame_id
        p = odom.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        point = TrackPoint(p.position.x, p.position.y, yaw, now_s)
        if self.points and (now_s <= self.points[-1].at or
                math.dist((point.x, point.y), (self.points[-1].x, self.points[-1].y)) > .25):
            self.points.clear()
        if not self.points or math.dist((point.x, point.y), (self.points[-1].x, self.points[-1].y)) >= .01:
            self.points.append(point)
        while len(self.points) > 1 and now_s - self.points[0].at > self.max_age_s:
            self.points.popleft()
        length = 0.
        for a, b in zip(reversed(self.points), list(reversed(self.points))[1:]):
            length += math.dist((a.x, a.y), (b.x, b.y))
        while length > self.max_length_m and len(self.points) > 1:
            a, b = self.points[0], self.points[1]
            length -= math.dist((a.x, a.y), (b.x, b.y))
            self.points.popleft()
