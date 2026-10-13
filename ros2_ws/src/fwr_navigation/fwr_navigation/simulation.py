"""Differential motion model and a demo-only waypoint follower."""

from dataclasses import dataclass, field
import math
import time


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class DifferentialPose:
    x: float
    y: float
    yaw: float

    def step(self, left_mps: float, right_mps: float, dt: float,
             wheel_separation_m: float) -> "DifferentialPose":
        values = (left_mps, right_mps, dt, wheel_separation_m)
        if not all(math.isfinite(value) for value in values) or dt < 0 or wheel_separation_m <= 0:
            raise ValueError("Invalid differential motion parameters")
        speed = (left_mps + right_mps) / 2
        turn_rate = (right_mps - left_mps) / wheel_separation_m
        new_yaw = self.yaw + turn_rate * dt
        if abs(turn_rate) < 1e-9:
            dx, dy = speed * dt * math.cos(self.yaw), speed * dt * math.sin(self.yaw)
        else:
            radius = speed / turn_rate
            dx = radius * (math.sin(new_yaw) - math.sin(self.yaw))
            dy = -radius * (math.cos(new_yaw) - math.cos(self.yaw))
        return DifferentialPose(self.x + dx, self.y + dy, _wrap(new_yaw))


@dataclass
class ChassisState:
    """Kinematic RViz preview of the two rear drives and two front casters."""

    pose: DifferentialPose = field(default_factory=lambda: DifferentialPose(0.0, 0.0, 0.0))
    left_wheel_rad: float = 0.0
    right_wheel_rad: float = 0.0
    front_left_steer_rad: float = 0.0
    front_right_steer_rad: float = 0.0
    front_left_roll_rad: float = 0.0
    front_right_roll_rad: float = 0.0
    wheel_separation_m: float = 0.29
    drive_radius_m: float = 0.05
    caster_radius_m: float = 0.04

    def step(self, linear: float, angular: float, dt: float) -> None:
        left = linear - angular * self.wheel_separation_m / 2
        right = linear + angular * self.wheel_separation_m / 2
        self.pose = self.pose.step(left, right, dt, self.wheel_separation_m)
        self.left_wheel_rad += left * dt / self.drive_radius_m
        self.right_wheel_rad += right * dt / self.drive_radius_m
        for side in ("left", "right"):
            caster_y = 0.115 if side == "left" else -0.115
            vx = linear - angular * caster_y
            vy = angular * 0.19
            if math.hypot(vx, vy) < 1e-6:
                continue
            angle = math.atan2(vy, vx)
            setattr(self, f"front_{side}_steer_rad", angle)
            speed = math.hypot(vx, vy) * (1 if vx >= 0 else -1)
            setattr(self, f"front_{side}_roll_rad",
                    getattr(self, f"front_{side}_roll_rad") + speed * dt / self.caster_radius_m)


class DemoVelocityLatch:
    """Clamp demo commands and stop if their publisher disappears."""

    def __init__(self, timeout_s: float = 0.3):
        self.timeout_s = timeout_s
        self.linear = 0.0
        self.angular = 0.0
        self.updated_at = float("-inf")

    def accept(self, linear: float, angular: float, now: float | None = None) -> None:
        if not math.isfinite(linear) or not math.isfinite(angular):
            raise ValueError("invalid demo velocity")
        self.linear = max(-0.2, min(0.2, linear))
        self.angular = max(-0.6, min(0.6, angular))
        self.updated_at = time.monotonic() if now is None else now

    def current(self, now: float | None = None) -> tuple[float, float]:
        now = time.monotonic() if now is None else now
        return ((self.linear, self.angular) if now - self.updated_at < self.timeout_s
                else (0.0, 0.0))


@dataclass(frozen=True)
class DemoCommand:
    linear: float = 0.0
    angular: float = 0.0


class WaypointFollower:
    """Turn-then-drive demo policy; never use as a real-robot controller."""

    def __init__(self, max_linear: float = 0.35, max_angular: float = 0.8,
                 position_tolerance: float = 0.12):
        if not all(math.isfinite(v) and v > 0 for v in
                   (max_linear, max_angular, position_tolerance)):
            raise ValueError("Invalid demo follower limits")
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.tolerance = position_tolerance
        self.waypoints: list[tuple[float, float]] = []
        self.index = 0
        self.reached = False

    def set_path(self, waypoints: list[tuple[float, float]]) -> None:
        if any(not all(math.isfinite(v) for v in point) for point in waypoints):
            raise ValueError("Invalid waypoint")
        self.waypoints = list(waypoints)
        self.index = 1 if len(waypoints) > 1 else 0
        self.reached = False

    def command(self, pose: DifferentialPose) -> DemoCommand:
        while self.index < len(self.waypoints):
            tx, ty = self.waypoints[self.index]
            distance = math.hypot(tx - pose.x, ty - pose.y)
            if distance > self.tolerance:
                heading = math.atan2(ty - pose.y, tx - pose.x)
                error = _wrap(heading - pose.yaw)
                turn = max(-self.max_angular, min(self.max_angular, 2.0 * error))
                if abs(error) > 0.2:
                    return DemoCommand(0.0, turn)
                return DemoCommand(min(self.max_linear, distance), turn)
            self.index += 1
        self.reached = bool(self.waypoints)
        return DemoCommand()
