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
