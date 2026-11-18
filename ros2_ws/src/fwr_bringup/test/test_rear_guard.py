"""车尾超声守卫：滞回与确认逻辑的单测（不需要硬件）。"""
import pytest

from fwr_bringup.rear_guard import update_hits

STOP, RESUME, CONFIRM = 0.30, 0.45, 3


def test_needs_consecutive_confirm_frames():
    hits = 0
    for _ in range(CONFIRM - 1):
        hits, hit = update_hits(0.20, STOP, RESUME, hits, CONFIRM)
        assert not hit
    hits, hit = update_hits(0.20, STOP, RESUME, hits, CONFIRM)
    assert hit


def test_gap_between_stop_and_resume_holds_state():
    """0.30~0.45 之间既不累加也不清零（滞回带）。"""
    hits = 0
    for _ in range(CONFIRM):
        hits, _ = update_hits(0.20, STOP, RESUME, hits, CONFIRM)
    assert hits == CONFIRM
    hits, hit = update_hits(0.35, STOP, RESUME, hits, CONFIRM)   # 滞回带内
    assert hits == CONFIRM and hit


def test_release_requires_going_past_resume():
    hits = 0
    for _ in range(CONFIRM):
        hits, _ = update_hits(0.20, STOP, RESUME, hits, CONFIRM)
    hits, hit = update_hits(0.50, STOP, RESUME, hits, CONFIRM)   # 超过 resume
    assert hits == 0 and not hit


def test_single_noise_frame_does_not_block():
    """单帧低于阈值不足以触发（需要连续 confirm 帧）。"""
    hits = 0
    hits, hit = update_hits(0.10, STOP, RESUME, hits, CONFIRM)
    assert not hit
    hits, hit = update_hits(1.30, STOP, RESUME, hits, CONFIRM)   # 噪声后回到背景
    assert hits == 0 and not hit


# ---------------- 2026-11-18：车尾改用后雷达 ----------------
import math

from fwr_bringup.rear_guard_core import sector_min_range


def _scan(points_deg_range, range_m=1.0, step=1.0, center=0.0):
    """构造一圈扫描：在 points_deg_range 范围内的角度上给 range_m。"""
    lo, hi = points_deg_range
    n = int(360 / step)
    ranges = [float('inf')] * n
    for i in range(n):
        a = -180.0 + i * step
        if lo <= a <= hi:
            ranges[i] = range_m
    return ranges, math.radians(-180.0), math.radians(step)


def test_sector_min_picks_nearest_in_rear_sector():
    """正后方扇区内取最近值。"""
    ranges, amin, inc = _scan((-180, -150), 0.8, 1.0)
    ranges2, _, _ = _scan((150, 180), 0.4, 1.0)
    merged = [min(a, b) for a, b in zip(ranges, ranges2)]
    d = sector_min_range(merged, amin, inc, center_deg=180.0, half_deg=35.0)
    assert abs(d - 0.4) < 1e-6


def test_sector_ignores_front_returns():
    """正前方（0° 附近）的回波不能被算成车尾距离。"""
    ranges, amin, inc = _scan((-5, 5), 0.2, 1.0)
    d = sector_min_range(ranges, amin, inc, center_deg=180.0, half_deg=35.0)
    assert math.isinf(d)


def test_sector_ignores_inf_and_out_of_range():
    """遮罩留下的 +inf 和超量程值必须跳过。"""
    ranges = [float('inf')] * 360
    ranges[180] = 50.0          # 超量程
    d = sector_min_range(ranges, math.radians(-180), math.radians(1.0))
    assert math.isinf(d)


def test_sector_handles_wraparound():
    """扇区跨 ±180 边界时，两侧都要算进去。"""
    ranges, amin, inc = _scan((-180, -175), 0.55, 1.0)
    d = sector_min_range(ranges, amin, inc, center_deg=180.0, half_deg=35.0)
    assert abs(d - 0.55) < 1e-6


def test_sector_center_180_equals_backward():
    """后雷达 yaw=0：扫描角 180° 就是车尾正后方。"""
    ranges, amin, inc = _scan((178, 182), 0.33, 1.0)
    d = sector_min_range(ranges, amin, inc, center_deg=180.0, half_deg=35.0)
    assert abs(d - 0.33) < 1e-6
