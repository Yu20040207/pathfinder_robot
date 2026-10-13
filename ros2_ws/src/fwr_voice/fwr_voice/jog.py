"""Simulation-only chassis motion commands."""

import math

JOG_COMMANDS = {
    "forward": (0.12, 0.0),
    "backward": (-0.12, 0.0),
    "left": (0.0, 0.35),
    "right": (0.0, -0.35),
}


def run_jog(direction, publish, cancelled, sleep, interval_s=0.1):
    """Publish a one-second fixed jog, returning True if canceled."""
    if direction not in JOG_COMMANDS:
        raise ValueError("unsupported jog direction")
    linear, angular = JOG_COMMANDS[direction]
    try:
        for _ in range(10):
            if cancelled():
                return True
            publish(linear, angular)
            sleep(interval_s)
        return False
    finally:
        publish(0.0, 0.0)


def parse_motion_target(target):
    direction, separator, raw = target.partition(":")
    if direction not in JOG_COMMANDS:
        raise ValueError("unsupported motion direction")
    if not separator:
        return direction, None
    try:
        distance = float(raw)
    except ValueError as exc:
        raise ValueError("invalid distance") from exc
    if not math.isfinite(distance) or not 0 < distance <= 10:
        raise ValueError("invalid distance")
    return direction, distance


def run_motion(target, publish, cancelled, sleep, distance_read=None, interval_s=0.1):
    """Refresh demo velocity until canceled or target distance is reached."""
    direction, limit = parse_motion_target(target)
    linear, angular = JOG_COMMANDS[direction]
    progress = 0.0
    initial = distance_read() if limit is not None and distance_read else None
    try:
        while True:
            if cancelled():
                return True
            if limit is not None and progress >= limit:
                return False
            publish(linear, angular)
            sleep(interval_s)
            if limit is not None:
                progress = ((distance_read() - initial) if initial is not None
                            else progress + (abs(linear) or abs(angular)) * interval_s)
    finally:
        publish(0.0, 0.0)
