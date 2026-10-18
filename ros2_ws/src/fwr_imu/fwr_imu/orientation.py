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
