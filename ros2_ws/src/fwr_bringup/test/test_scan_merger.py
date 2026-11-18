"""双雷达合并的核心逻辑：角度归位、同格取最近、空格 +inf、外参旋转。"""
import math

from fwr_bringup.scan_merger_core import BINS, filled_ratio, merge_into_bins

# 一路只有正前方 4 个点的扫描，range 都是 2.0，从 -2° 到 +2°
FWD = ([2.0, 2.0, 2.0, 2.0],
       math.radians(-2.0), math.radians(1.0), 0.0, 0.0, 0.0)


def test_empty_input_is_all_inf():
    merged = merge_into_bins([], BINS)
    assert len(merged) == BINS
    assert all(math.isinf(v) for v in merged)
    assert filled_ratio(merged) == 0.0


def test_forward_points_land_near_zero():
    merged = merge_into_bins([FWD], BINS)
    mid = BINS // 2          # 索引 0 = -pi，中间 = 0 附近
    assert math.isfinite(merged[mid]) or math.isfinite(merged[mid - 1])
    assert filled_ratio(merged) < 0.05


def test_rear_scan_rotated_180_lands_opposite():
    """后雷达装成 yaw=pi 时，它测到的"前方"应落到合并图的另一侧。"""
    rear = (FWD[0], FWD[1], FWD[2], math.pi, 0.0, 0.0)
    merged = merge_into_bins([rear], BINS)
    mid = BINS // 2
    assert all(math.isinf(v) for v in merged[mid - 2:mid + 3])
    assert any(math.isfinite(v) for v in merged[:5]) or any(
        math.isfinite(v) for v in merged[-5:])


def test_nearest_wins_in_same_bin():
    """同一格有远有近时取近的（保守，不会把障碍当空地）。"""
    INC = math.radians(0.5)
    near = ([1.0], 0.0, INC, 0.0, 0.0, 0.0)
    far = ([5.0], 0.0, INC, 0.0, 0.0, 0.0)
    merged = merge_into_bins([near, far], BINS)
    mid = BINS // 2
    assert abs(merged[mid] - 1.0) < 1e-9


def test_translation_applied():
    """目标系里平移 1 m 后，距离应变成 1.0 + 原距离。"""
    p = ([2.0], 0.0, math.radians(0.5), 0.0, -1.0, 0.0)   # 雷达在 x=-1，朝 +x
    merged = merge_into_bins([p], BINS)
    mid = BINS // 2
    assert abs(merged[mid] - 1.0) < 0.02


def test_out_of_range_dropped():
    p = ([30.0], 0.0, math.radians(0.5), 0.0, 0.0, 0.0)
    merged = merge_into_bins([p], BINS, range_max=12.0)
    assert all(math.isinf(v) for v in merged)


def test_nan_and_inf_inputs_ignored():
    p = ([float('nan'), float('inf'), 3.0], 0.0, math.radians(1.0), 0.0, 0.0, 0.0)
    merged = merge_into_bins([p], BINS)
    assert filled_ratio(merged) <= 2.0 / BINS


def test_two_supplementary_scans_cover_more_than_each():
    """两路互补（0° 与 180° 方向）合并后有效格应更多。"""
    a = ([2.0] * 5, math.radians(-2.0), math.radians(1.0), 0.0, 0.0, 0.0)
    b = ([2.0] * 5, math.radians(-2.0), math.radians(1.0), math.pi, 0.0, 0.0)
    one = filled_ratio(merge_into_bins([a], BINS))
    two = filled_ratio(merge_into_bins([a, b], BINS))
    assert two > one
