"""Serial-independent command gate and odometry for the physical base."""

from dataclasses import dataclass
import math

from .base_protocol import BaseState, encode_wheel_speed, parse_handshake
from .wheel_odometry import OdomSample, WheelOdometry


class ScanWatchdog:
    """Require a recent LaserScan containing at least one usable range."""

    def __init__(self, timeout_s: float = .25) -> None:
        self.timeout_s = timeout_s
        self._last_valid_at = None

    def observe(self, ranges, now_s: float) -> bool:
        valid = any(math.isfinite(r) and r > 0 for r in ranges)
        if valid:
            self._last_valid_at = now_s
        return valid

    def ready(self, now_s: float) -> bool:
        return (self._last_valid_at is not None and
                0 <= now_s - self._last_valid_at <= self.timeout_s)


class SensorStampGuard:
    """Check both acquisition time and monotonic arrival freshness."""

    def __init__(self, timeout_s: float = .25, max_future_s: float = .05) -> None:
        self.timeout_s = timeout_s
        self.max_future_s = max_future_s
        self._last_stamp_s = None
        self._last_valid_at = None
        self._age_at_arrival_s = 0.0

    def accept(self, stamp_s: float, ros_now_s: float, mono_now_s: float) -> bool:
        if not all(map(math.isfinite, (stamp_s, ros_now_s, mono_now_s))):
            return False
        age = ros_now_s - stamp_s
        if (stamp_s <= 0 or age < -self.max_future_s or age > self.timeout_s or
                (self._last_stamp_s is not None and stamp_s <= self._last_stamp_s)):
            return False
        self._last_stamp_s = stamp_s
        self._last_valid_at = mono_now_s
        self._age_at_arrival_s = max(0.0, age)
        return True

    def age_at(self, mono_now_s: float) -> float:
        if self._last_valid_at is None or mono_now_s < self._last_valid_at:
            return math.inf
        return self._age_at_arrival_s + mono_now_s - self._last_valid_at

    def ready(self, mono_now_s: float) -> bool:
        return self.age_at(mono_now_s) <= self.timeout_s


def forward_only_velocity(v_mps: float, omega_rps: float, track_m: float) -> bool:
    """For autonomous /cmd_vel; diagnostic turn action has its own gate."""
    if not all(map(math.isfinite, (v_mps, omega_rps))):
        return False
    if v_mps < 0 or (v_mps == 0 and omega_rps != 0):
        return False
    return v_mps - abs(omega_rps) * track_m / 2 >= -1e-9


@dataclass(frozen=True)
class BridgeOutput:
    command: bytes
    odom: OdomSample | None
    wheel_positions_rad: tuple[float, float]
    ready: bool
    fault: int


class BaseBridgeCore:
    def __init__(self, track_m: float = .263, circumference_m: float = .210,
                 counts_per_rev: int = 1468) -> None:
        self.track_m = track_m
        self.counts_per_rev = counts_per_rev
        self.odom = WheelOdometry(circumference_m, track_m, counts_per_rev)
        self.connected = False
        self.verified = False
        self.fault = 0
        self._last_state_at = None
        self._last_cmd_at = None
        self._target = None
        self._last_counts = None
        self._last_esp_us = None
        self._wheel_positions = [0.0, 0.0]
        self._turn_owner = False

    def connect(self, now_s: float) -> None:
        self.connected = True
        self.verified = False
        self._last_state_at = None
        self._last_cmd_at = None
        self._target = None
        self._last_counts = None
        self._last_esp_us = None
        self.odom.reset_reference()
        self._turn_owner = False

    def disconnect(self) -> None:
        self.connected = False
        self.verified = False
        self._target = None
        self._last_cmd_at = None
        self._last_state_at = None
        self._last_counts = None
        self.odom.reset_reference()
        self._turn_owner = False

    def handshake(self, line: str) -> bool:
        self.verified = parse_handshake(line)
        return self.verified

    def accept_cmd(self, v_mps: float, omega_rps: float, now_s: float) -> tuple[float, float]:
        if self._turn_owner:
            raise RuntimeError('relative turn owns base')
        self._target = None
        self._last_cmd_at = None
        if not math.isfinite(v_mps) or not math.isfinite(omega_rps):
            raise ValueError('nonfinite body velocity')
        left = 1000.0 * (v_mps - omega_rps * self.track_m / 2.0)
        right = 1000.0 * (v_mps + omega_rps * self.track_m / 2.0)
        encode_wheel_speed(left, right)
        if self.ready(now_s):
            self._target = (left, right)
            self._last_cmd_at = now_s
        return left, right

    def begin_turn(self, now_s: float) -> bool:
        if (self._turn_owner or not self.ready(now_s) or
                (self._target is not None and self._last_cmd_at is not None and
                 0 <= now_s - self._last_cmd_at <= .20)):
            return False
        self._turn_owner = True
        self._target = None
        self._last_cmd_at = None
        return True

    def accept_turn_cmd(self, left_mm_s: float, right_mm_s: float, now_s: float) -> None:
        if not self._turn_owner or not self.ready(now_s):
            raise RuntimeError('relative turn unavailable')
        encode_wheel_speed(left_mm_s, right_mm_s)
        self._target = (left_mm_s, right_mm_s)
        self._last_cmd_at = now_s

    def end_turn(self) -> None:
        self.halt()

    def halt(self) -> None:
        self._turn_owner = False
        self._target = None
        self._last_cmd_at = None

    def ready(self, now_s: float) -> bool:
        return (self.connected and self.verified and self.fault == 0 and
                self._last_state_at is not None and
                0 <= now_s - self._last_state_at <= .25)

    def step(self, state: BaseState | None, now_s: float) -> BridgeOutput:
        sample = None
        if state is not None and self.connected and self.verified:
            self._last_state_at = now_s
            if state.fault:
                self.fault = state.fault
            if self._last_esp_us is not None and state.esp_us < self._last_esp_us:
                self.connect(now_s)
                return BridgeOutput(b'x', None, tuple(self._wheel_positions), False,
                                    self.fault)
            sample = self.odom.update(state.left_count, state.right_count,
                                      now_s, state.esp_us // 1000)
            if sample is not None and self._last_counts is not None:
                old_left, old_right = self._last_counts
                left_delta = ((state.left_count - old_left + 2**31) % 2**32) - 2**31
                right_delta = ((state.right_count - old_right + 2**31) % 2**32) - 2**31
                self._wheel_positions[0] -= left_delta * 2 * math.pi / self.counts_per_rev
                self._wheel_positions[1] += right_delta * 2 * math.pi / self.counts_per_rev
            self._last_counts = (state.left_count, state.right_count)
            self._last_esp_us = state.esp_us
        ready = self.ready(now_s)
        if not self.connected:
            command = b''
        elif (ready and self._target is not None and self._last_cmd_at is not None
              and 0 <= now_s - self._last_cmd_at <= .20):
            command = encode_wheel_speed(*self._target)
        else:
            command = b'x'
        return BridgeOutput(command, sample, tuple(self._wheel_positions), ready,
                            self.fault)
