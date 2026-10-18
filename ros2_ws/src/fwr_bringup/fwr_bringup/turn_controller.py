"""Bounded relative yaw goal translated to wheel-speed requests."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TurnFinished:
    success: bool
    reason: str
    actual_angle_deg: float
    command: tuple[int, int] = (0, 0)


class TurnController:
    PIVOT = 1
    ARC = 2

    def __init__(self, angle_deg: float, mode: int, started_at_s: float) -> None:
        if not math.isfinite(angle_deg) or not 2 <= abs(angle_deg) <= 90:
            raise ValueError('relative angle must be 2..90 degrees')
        if mode not in (self.PIVOT, self.ARC):
            raise ValueError('invalid turn mode')
        self.target_deg = angle_deg
        self.mode = mode
        self.started_at_s = started_at_s
        self._finished = None
        self.remaining_angle_deg = angle_deg
        self._last_progress = [0.0, 0.0]
        self._last_moved_at = [started_at_s, started_at_s]

    def _finish(self, success: bool, reason: str, actual: float) -> TurnFinished:
        self._finished = TurnFinished(success, reason, actual)
        return self._finished

    def cancel(self) -> TurnFinished:
        return self._finish(False, 'cancelled', self.target_deg - self.remaining_angle_deg)

    def update(self, relative_yaw_rad: float, yaw_rate_rad_s: float, now_s: float,
               left_progress_m: float, right_progress_m: float,
               imu_age_s: float = 0.0, fault: int = 0) -> tuple[int, int] | TurnFinished:
        if self._finished is not None:
            return self._finished
        if not all(map(math.isfinite, (relative_yaw_rad, yaw_rate_rad_s,
                                       now_s, left_progress_m, right_progress_m,
                                       imu_age_s))):
            return self._finish(False, 'invalid_sensor', 0)
        actual = math.degrees(math.atan2(math.sin(relative_yaw_rad),
                                         math.cos(relative_yaw_rad)))
        self.remaining_angle_deg = self.target_deg - actual
        if fault:
            return self._finish(False, 'fault', actual)
        if imu_age_s > .15 or imu_age_s < 0:
            return self._finish(False, 'imu_stale', actual)
        if now_s - self.started_at_s > 3.0:
            return self._finish(False, 'timeout', actual)
        if self.target_deg * actual < -2 or abs(actual) > abs(self.target_deg) + 4:
            return self._finish(False, 'overshoot', actual)
        if abs(self.remaining_angle_deg) <= 2:
            return self._finish(True, 'complete', actual)
        for side, progress in enumerate((left_progress_m, right_progress_m)):
            if abs(progress - self._last_progress[side]) >= .0005:
                self._last_moved_at[side] = now_s
                self._last_progress[side] = progress
        if (now_s - self.started_at_s >= .4 and
                any(now_s - last > .35 for last in self._last_moved_at)):
            return self._finish(False, 'encoder_stall', actual)
        sign = 1 if self.target_deg > 0 else -1
        slow = abs(self.remaining_angle_deg) <= 3
        if self.mode == self.PIVOT:
            speed = 50 if slow else 80
            return (-sign * speed, sign * speed)
        inner, outer = (30, 50) if slow else (50, 80)
        return (inner, outer) if sign > 0 else (outer, inner)
