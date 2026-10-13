"""Planar map-to-odom transform from Gazebo world pose and wheel odometry."""

import math


def align_map_to_odom(map_base, odom_base):
    """Return (x, y, yaw) for T_map_odom = T_map_base * inverse(T_odom_base)."""
    gx, gy, gyaw = map_base
    ox, oy, oyaw = odom_base
    yaw = math.atan2(math.sin(gyaw - oyaw), math.cos(gyaw - oyaw))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return gx - cosine * ox + sine * oy, gy - sine * ox - cosine * oy, yaw


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
