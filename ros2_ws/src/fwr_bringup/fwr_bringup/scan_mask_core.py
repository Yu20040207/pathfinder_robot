"""Pure angular mask for the C1's permanent rear occlusion."""

import math
from typing import Sequence


def mask_rear_ranges(ranges: Sequence[float], angle_min: float,
                     angle_increment: float, rear_half_angle: float) -> list[float]:
    """Return a copy with rear-facing samples set to NaN.

    The C1's scan zero points toward the robot rear.  This masks observations;
    it never creates free-space measurements in the blind sector.
    """
    if (not all(math.isfinite(v) for v in (angle_min, angle_increment, rear_half_angle))
            or angle_increment == 0 or not 0 < rear_half_angle < math.pi):
        raise ValueError('invalid scan angle geometry or rear mask width')
    result = list(ranges)
    for index in range(len(result)):
        angle = angle_min + index * angle_increment
        if not math.isfinite(angle):
            raise ValueError('scan angle overflow')
        wrapped = math.atan2(math.sin(angle), math.cos(angle))
        # LaserScan angular fields are float32; include intended boundary rays.
        if abs(wrapped) <= rear_half_angle + 1e-6:
            result[index] = math.nan
    return result
