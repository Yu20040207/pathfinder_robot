"""Serial-independent command gate and odometry for the physical base."""

from dataclasses import dataclass
import math

from .base_protocol import BaseState, encode_wheel_speed, parse_handshake
from .wheel_odometry import OdomSample, WheelOdometry


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

    def connect(self, now_s: float) -> None:
        self.connected = True
        self.verified = False
        self._last_state_at = None
        self._last_cmd_at = None
        self._target = None
        self._last_counts = None
        self._last_esp_us = None
        self.odom.reset_reference()

    def disconnect(self) -> None:
        self.connected = False
        self.verified = False
        self._target = None
        self._last_cmd_at = None
        self._last_state_at = None
        self._last_counts = None
        self.odom.reset_reference()

    def handshake(self, line: str) -> bool:
        self.verified = parse_handshake(line)
        return self.verified

    def accept_cmd(self, v_mps: float, omega_rps: float, now_s: float) -> tuple[float, float]:
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
                self._last_counts = None
                self.odom.reset_reference()
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
