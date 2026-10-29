"""Bounded manual yaw-rate correction using the robot-mounted IMU gyro."""

import math


class ImuYawRateController:
    def __init__(self, max_rate_rad_s: float = .75):
        self.max_rate_rad_s = max_rate_rad_s
        self.reset()

    def reset(self):
        self._direction = 0
        self._last_at = None
        self._integral = 0.0
        self._stalled_s = 0.0

    def update(self, target_rad_s: float, measured_rad_s: float, now_s: float) -> float:
        if not all(map(math.isfinite, (target_rad_s, measured_rad_s, now_s))):
            self.reset()
            raise ValueError('nonfinite IMU turn rate')
        if abs(target_rad_s) > self.max_rate_rad_s:
            self.reset()
            raise ValueError('turn rate exceeds manual limit')
        if abs(target_rad_s) < 1e-6:
            self.reset()
            return 0.0
        direction = 1 if target_rad_s > 0 else -1
        if (direction != self._direction or self._last_at is None or
                now_s < self._last_at or now_s - self._last_at > .25):
            self.reset()
            self._direction = direction
            dt = 0.0
        else:
            dt = min(now_s - self._last_at, .1)
        self._last_at = now_s
        error = abs(target_rad_s) - direction * measured_rad_s
        if direction * measured_rad_s < .08 and abs(target_rad_s) >= .2:
            self._stalled_s += dt
        else:
            self._stalled_s = 0.0
        trial = max(-.3, min(1.0, self._integral + error * dt))
        unbounded = abs(target_rad_s) + .25 * error + .30 * trial
        if (0.0 <= unbounded <= self.max_rate_rad_s or
                (unbounded > self.max_rate_rad_s and error < 0) or
                (unbounded < 0.0 and error > 0)):
            self._integral = trial
        magnitude = abs(target_rad_s) + .25 * error + .30 * self._integral
        if self._stalled_s >= .15:
            magnitude = self.max_rate_rad_s
        return direction * max(0.0, min(self.max_rate_rad_s, magnitude))
