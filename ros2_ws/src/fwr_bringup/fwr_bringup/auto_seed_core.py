"""Global scan-to-map match used to seed AMCL after every stack start."""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt


def occupancy_from_grid(data, width, height):
    """Return a boolean occupied mask from nav_msgs OccupancyGrid data."""
    grid = np.array(data, dtype=np.int16).reshape(height, width)
    return grid >= 65


def distance_field(occupied, resolution):
    return distance_transform_edt(~occupied) * resolution


def score_pose(distance, resolution, origin_x, origin_y, points, x, y, yaw):
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    height, width = distance.shape
    xf = cos_yaw * points[:, 0] - sin_yaw * points[:, 1] + x
    yf = sin_yaw * points[:, 0] + cos_yaw * points[:, 1] + y
    ix = ((xf - origin_x) / resolution).astype(int)
    iy = (height - 1 - (yf - origin_y) / resolution).astype(int)
    inside = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    if inside.sum() < len(points) * 0.5:
        return 9.9
    return float(distance[iy[inside], ix[inside]].mean())


def find_best_pose(occupied, resolution, origin_x, origin_y, bounds, points,
                   coarse_step=.3, coarse_yaw_deg=10., fine_step=.04,
                   fine_yaw_deg=1.0):
    """Coarse-to-fine global search. Returns (score, x, y, yaw_rad).

    ``bounds`` is (min_x, max_x, min_y, max_y) in map coordinates.
    """
    distance = distance_field(occupied, resolution)
    min_x, max_x, min_y, max_y = bounds
    best = (9.9, 0.0, 0.0, 0.0)
    yaw_range = np.radians(np.arange(-180.0, 180.0, coarse_yaw_deg))
    for x in np.arange(min_x, max_x, coarse_step):
        for y in np.arange(min_y, max_y, coarse_step):
            for yaw in yaw_range:
                value = score_pose(distance, resolution, origin_x, origin_y,
                                   points, x, y, yaw)
                if value < best[0]:
                    best = (value, x, y, yaw)
    if best[0] > 9.0:
        return best
    centre = best
    yaw_span = np.radians(np.arange(-coarse_yaw_deg, coarse_yaw_deg + 1e-9, fine_yaw_deg))
    for x in np.arange(centre[1] - coarse_step, centre[1] + coarse_step, fine_step):
        for y in np.arange(centre[2] - coarse_step, centre[2] + coarse_step, fine_step):
            for yaw in centre[3] + yaw_span:
                value = score_pose(distance, resolution, origin_x, origin_y,
                                   points, x, y, yaw)
                if value < best[0]:
                    best = (value, x, y, yaw)
    return best


def base_pose_from_laser(x, y, yaw, laser_offset_m=.21):
    """Convert a matched laser pose to the base_footprint pose."""
    base_yaw = yaw - math.pi
    return (x - laser_offset_m * math.cos(base_yaw),
            y - laser_offset_m * math.sin(base_yaw), base_yaw)


def refine_pose(occupied, resolution, origin_x, origin_y, points, x, y, yaw,
                span_m=.5, span_yaw_deg=15.0, step_m=.04, step_yaw_deg=1.0):
    """Local refinement around a prior pose (e.g. the last known checkpoint)."""
    distance = distance_field(occupied, resolution)
    best = (9.9, x, y, yaw)
    yaw_span = math.radians(span_yaw_deg)
    for dx in np.arange(-span_m, span_m + 1e-9, step_m):
        for dy in np.arange(-span_m, span_m + 1e-9, step_m):
            for dth in np.arange(-yaw_span, yaw_span + 1e-9, math.radians(step_yaw_deg)):
                value = score_pose(distance, resolution, origin_x, origin_y,
                                   points, x + dx, y + dy, yaw + dth)
                if value < best[0]:
                    best = (value, x + dx, y + dy, yaw + dth)
    return best
