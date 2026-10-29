"""Bounded, versioned ESP32 base serial records."""

from dataclasses import dataclass
import math


MAX_LINE = 160
MAX_FAULT = 6


@dataclass(frozen=True)
class BaseState:
    esp_us: int
    left_count: int
    right_count: int
    left_skipped: int
    right_skipped: int
    left_measured: float
    right_measured: float
    left_setpoint: float
    right_setpoint: float
    left_pwm: float
    right_pwm: float
    active: bool
    fault: int


def parse_handshake(line: str) -> bool:
    return line == 'BASE_CONTROL_V2'


def parse_state_line(line: str) -> BaseState | None:
    if len(line) > MAX_LINE:
        return None
    fields = line.split()
    if len(fields) != 14 or fields[0] != 'S2':
        return None
    try:
        esp_us, left, right, left_skipped, right_skipped = map(int, fields[1:6])
        speeds = tuple(map(float, fields[6:12]))
        active, fault = map(int, fields[12:14])
    except ValueError:
        return None
    if (esp_us < 0 or left_skipped < 0 or right_skipped < 0 or
            active not in (0, 1) or not 0 <= fault <= MAX_FAULT or
            not all(math.isfinite(value) for value in speeds) or
            any(abs(value) > 1000 for value in speeds[:4]) or
            any(abs(value) > 1 for value in speeds[4:])):
        return None
    return BaseState(esp_us, left, right, left_skipped, right_skipped,
                     *speeds, bool(active), fault)


def encode_wheel_speed(left_mm_s: float, right_mm_s: float) -> bytes:
    if not all(math.isfinite(value) and abs(value) <= 300
               for value in (left_mm_s, right_mm_s)):
        raise ValueError('wheel speed must be finite and within ±300 mm/s')
    return f'v {left_mm_s:.3f} {right_mm_s:.3f}\n'.encode('ascii')
