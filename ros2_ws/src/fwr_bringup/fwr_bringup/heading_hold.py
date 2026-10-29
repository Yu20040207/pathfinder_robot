"""Straight-line assist: hold the IMU heading while driving without steering.

差速底盘左右轮等效半径不完全相同，只给前进指令时车会缓慢画弧（实测约 1°/米）。
这里在“只前进、不转向”时用 IMU 航向做一个小闭环，把车拉回直线。
只在手动（键盘/建图）路径使用，导航控制器仍完全掌控角速度。
"""
import math


class HeadingHold:
    def __init__(self, gain: float = 1.5, limit_rad_s: float = .12,
                 deadband_rad_s: float = .02, min_speed_mps: float = .02,
                 stale_s: float = .3):
        self.gain = gain
        self.limit_rad_s = limit_rad_s
        self.deadband_rad_s = deadband_rad_s
        self.min_speed_mps = min_speed_mps
        self.stale_s = stale_s
        self.reset()

    def reset(self) -> None:
        self._reference = None
        self._last_at = None

    def update(self, linear_mps: float, requested_rad_s: float, yaw_rad: float,
               now_s: float) -> float:
        """Return the angular command to send instead of `requested_rad_s`."""
        if not all(map(math.isfinite, (linear_mps, requested_rad_s, yaw_rad, now_s))):
            self.reset()
            return requested_rad_s
        # 只有“直行、不转向”才介入；一旦有转向指令或车停下就交还原指令。
        if (abs(requested_rad_s) > self.deadband_rad_s or
                abs(linear_mps) < self.min_speed_mps):
            self.reset()
            return requested_rad_s
        if self._last_at is not None and now_s - self._last_at > self.stale_s:
            self._reference = None
        self._last_at = now_s
        if self._reference is None:
            self._reference = yaw_rad
            return requested_rad_s
        error = math.atan2(math.sin(yaw_rad - self._reference),
                           math.cos(yaw_rad - self._reference))
        correction = max(-self.limit_rad_s,
                         min(self.limit_rad_s, -self.gain * error))
        return requested_rad_s + correction
