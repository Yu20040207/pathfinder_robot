"""Validate and convert the DM-IMU-L1's wxyz attitude to ROS xyzw."""

import math


def quaternion_xyzw(wxyz: tuple[float, ...]) -> tuple[float, float, float, float] | None:
    if len(wxyz) != 4 or not all(math.isfinite(value) for value in wxyz):
        return None
    magnitude = math.sqrt(sum(value * value for value in wxyz))
    if magnitude < 0.5 or magnitude > 1.5:
        return None
    w, x, y, z = (value / magnitude for value in wxyz)
    return x, y, z, w


def relative_quaternion_xyzw(reference: tuple[float, float, float, float],
                             current: tuple[float, float, float, float]
                             ) -> tuple[float, float, float, float]:
    """Return the current body attitude relative to its preview start pose."""
    ax, ay, az, aw = current
    bx, by, bz, bw = (-reference[0], -reference[1], -reference[2], reference[3])
    result = (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )
    size = math.sqrt(sum(value * value for value in result))
    return tuple(value / size for value in result)


def _euler_from_xyzw(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def _xyzw_from_euler(roll: float, pitch: float, yaw: float
                     ) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def scale_yaw_xyzw(quaternion: tuple[float, float, float, float], scale: float,
                   reference_yaw: float) -> tuple[float, float, float, float]:
    """Scale yaw about a reference angle, keeping roll/pitch untouched.

    2026-10-30 实测：DM-IMU-L1 的六轴航向积分少算约 1.25%
    （转一圈 IMU 报 355.04°，雷达互相关真值 359.55°）。
    这里把「相对参考朝向的偏航角」乘上修正系数，得到真实角度。
    """
    if not math.isfinite(scale) or scale <= 0.0:
        return quaternion
    roll, pitch, yaw = _euler_from_xyzw(quaternion)
    delta = math.atan2(math.sin(yaw - reference_yaw), math.cos(yaw - reference_yaw))
    corrected = reference_yaw + scale * delta
    return _xyzw_from_euler(roll, pitch, corrected)
