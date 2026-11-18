"""雷达位姿 → base 位姿的换算。

2026-11-18 的 bug：写死 `yaw - pi` / `-0.21 m` 只对倒装的前雷达成立，
换成 /scan_merged（base_link）后位姿被多转 180°、多偏 21 cm，
而残差在换算前算的，日志照样"很漂亮"。
"""
import math

from fwr_bringup.auto_seed_core import base_pose_from_laser, pose_laser_to_base


def test_identity_keeps_pose():
    """扫描坐标系 == base 时位置/朝向都不应变化（/scan_merged 的情形）。"""
    x, y, yaw = 1.5, -2.5, math.radians(37)
    bx, by, byaw = pose_laser_to_base(x, y, yaw, 0.0, 0.0, 0.0)
    assert abs(bx - x) < 1e-9 and abs(by - y) < 1e-9
    assert abs(byaw - yaw) < 1e-9


def test_front_lidar_matches_legacy():
    """前雷达倒装：外参 (0.21, 0, pi)。必须与旧的写死补偿完全一致。"""
    la_x, la_y, la_yaw = 4.0, 1.0, math.radians(120)
    off = 0.21
    old = base_pose_from_laser(la_x, la_y, la_yaw, off)
    new = pose_laser_to_base(la_x, la_y, la_yaw, off, 0.0, math.pi)
    for a, b in zip(old, new):
        assert abs(a - b) < 1e-9, f'旧={old} 新={new}'


def test_merged_scan_not_rotated_by_pi():
    """回归：换用 base_link 的合并扫描后，绝不能再减 180°。"""
    yaw = math.radians(90)
    bx, _, byaw = pose_laser_to_base(2.0, 0.0, yaw, 0.0, 0.0, 0.0)
    assert abs(byaw - yaw) < 1e-9
    assert abs(math.degrees(byaw) - 90.0) < 1e-6


def test_rear_lidar_transform():
    """后雷达：base←laser 为 (-0.229, 0, 0)。位姿应平移而非旋转。"""
    bx, by, byaw = pose_laser_to_base(0.0, 0.0, 0.0, -0.229, 0.0, 0.0)
    assert abs(bx - 0.229) < 1e-9     # base 在雷达前方 0.229 m
    assert abs(by) < 1e-9
    assert abs(byaw) < 1e-9


def test_rotation_and_translation_combined():
    """雷达在 base 前方 1 m 且相对 base 转 +90°（外参 1,0,pi/2）。

    雷达位姿（在 map 中）是 (3, 0, 0)。
      base_yaw = 0 - pi/2 = -90°
      雷达原点在 map = base 位置 + R(-90°)·(1,0) = base + (0,-1)
      → base 位置 = (3,0) - (0,-1) = (3, 1)
    """
    bx, by, byaw = pose_laser_to_base(3.0, 0.0, 0.0, 1.0, 0.0, math.pi / 2)
    assert abs(byaw - (-math.pi / 2)) < 1e-9
    assert abs(bx - 3.0) < 1e-9
    assert abs(by - 1.0) < 1e-9
