"""Differential-drive odometry from ESP32-S3 quadrature counts."""

from dataclasses import dataclass
import math
import re


_ENCODER_LINE = re.compile(
    r'\bI \((\d+)\) fwr_motor_pulse: encoder '
    r'A=(-?\d+) skipped=(\d+) B=(-?\d+) skipped=(\d+)\b'
)


def parse_encoder_line(line: str):
    """Return (boot_ms, A, A_skipped, B, B_skipped), or None."""
    match = _ENCODER_LINE.search(line)
    return tuple(map(int, match.groups())) if match else None


@dataclass(frozen=True)
class OdomSample:
    x: float
    y: float
    yaw: float
    linear_mps: float
    angular_rps: float


class WheelOdometry:
    """Track pose using A=-left-forward and B=right-forward encoder counts."""

    def __init__(self, circumference_m: float, track_m: float,
                 counts_per_rev: int, max_wheel_mps: float = 2.0):
        if circumference_m <= 0 or track_m <= 0 or counts_per_rev <= 0 or max_wheel_mps <= 0:
            raise ValueError('odometry calibration values must be positive')
        self.meters_per_count = circumference_m / counts_per_rev
        self.track_m = track_m
        self.max_wheel_mps = max_wheel_mps
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._previous = None

    def reset_reference(self) -> None:
        """Keep accumulated pose while treating the next counts as a new baseline."""
        self._previous = None

    def update(self, left_count: int, right_count: int,
               stamp_s: float, boot_ms: int):
        current = (left_count, right_count, stamp_s, boot_ms)
        previous = self._previous
        if previous is None:
            self._previous = current
            return None
        old_left, old_right, old_stamp, old_boot = previous
        dt = stamp_s - old_stamp
        if boot_ms < old_boot or dt <= 0:
            self._previous = current
            return None
        # Firmware counters are signed 32-bit values and may wrap while moving.
        left_delta = ((left_count - old_left + 2**31) % 2**32) - 2**31
        right_delta = ((right_count - old_right + 2**31) % 2**32) - 2**31
        left_m = -left_delta * self.meters_per_count
        right_m = right_delta * self.meters_per_count
        self._previous = current
        if max(abs(left_m), abs(right_m)) > self.max_wheel_mps * dt:
            return None
        distance = (left_m + right_m) / 2.0
        turn = (right_m - left_m) / self.track_m
        self.x += distance * math.cos(self.yaw + turn / 2.0)
        self.y += distance * math.sin(self.yaw + turn / 2.0)
        self.yaw = math.atan2(math.sin(self.yaw + turn), math.cos(self.yaw + turn))
        return OdomSample(self.x, self.y, self.yaw, distance / dt, turn / dt)
