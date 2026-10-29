"""Conservative saved-map checks for a supervised single navigation goal."""

import math


def accept_map_message(static_map, valid_stamp, stamp_age_s):
    return bool(static_map or (valid_stamp and -.1 <= stamp_age_s <= 1.))


def map_available(static_map, received, received_at_s, now_s, timeout_s=1.):
    return bool(received and received_at_s is not None and
                (static_map or 0 <= now_s - received_at_s < timeout_s))


def validate_fixed_goal(grid, robot_x, robot_y, goal_x, goal_y, goal_yaw,
                        max_distance_m=.5):
    values = (robot_x, robot_y, goal_x, goal_y, goal_yaw, max_distance_m)
    if not all(math.isfinite(v) for v in values) or max_distance_m < 0:
        return False, 'non-finite coordinate or invalid distance limit'
    if max_distance_m > 0 and math.hypot(goal_x - robot_x, goal_y - robot_y) > max_distance_m + 1e-9:
        return False, 'goal exceeds trial distance limit'
    info = grid.info
    width, height, resolution = info.width, info.height, info.resolution
    if width <= 0 or height <= 0 or resolution <= 0 or len(grid.data) != width * height:
        return False, 'invalid map'
    q = info.origin.orientation
    origin_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                            1 - 2 * (q.y * q.y + q.z * q.z))
    dx, dy = goal_x - info.origin.position.x, goal_y - info.origin.position.y
    gx = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
    gy = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy
    cx, cy = int(math.floor(gx / resolution)), int(math.floor(gy / resolution))
    # 操作者要求：目标点离墙/未知区域 >= 0.30 m 即可（机身外接半径约 0.315 m，
    # 略低于它，实际避障由 Nav2 代价地图与碰撞监控兜底）。未知格仍一律拒绝。
    radius = .30
    cells = math.ceil(radius / resolution)
    if cx - cells < 0 or cy - cells < 0 or cx + cells >= width or cy + cells >= height:
        return False, 'goal footprint outside map'
    for y in range(cy - cells, cy + cells + 1):
        for x in range(cx - cells, cx + cells + 1):
            if math.hypot((x - cx) * resolution, (y - cy) * resolution) <= radius:
                if grid.data[y * width + x] != 0:
                    return False, 'goal footprint not known free'
    return True, 'accepted'


def validate_two_point_goals(grid, robot_x, robot_y, start, end, max_leg_distance_m=.5):
    # Nav2 may report the first goal reached within its 0.20 m XY tolerance.
    # Reserve that possible gap so the second dispatch still fits the trial cap.
    second_limit = max_leg_distance_m - .2 if max_leg_distance_m > 0 else 0.
    if max_leg_distance_m > 0 and second_limit <= 0:
        return False, 'trial limit smaller than first-goal tolerance'
    for name, origin, target in (
        ('first', (robot_x, robot_y), start),
        ('second', start[:2], end),
    ):
        ok, reason = validate_fixed_goal(grid, origin[0], origin[1],
                                         target[0], target[1], target[2],
                                         max_leg_distance_m if name == 'first' else second_limit)
        if not ok:
            return False, f'{name} leg: {reason}'
    return True, 'accepted'
