from datetime import datetime, timedelta, timezone
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "ros2_ws/src/fwr_data"))
sys.path.insert(0, str(root / "ros2_ws/src/fwr_vision"))
sys.path.insert(0, str(root / "ros2_ws/src/fwr_attendance"))

from fwr_vision.engine import FaceResult
from fwr_attendance.decision import AttendanceDecision


START = datetime(2026, 10, 4, tzinfo=timezone.utc)
KNOWN = FaceResult("P1", "A", True, 0.8, (1, 2, 40, 50))
UNKNOWN = FaceResult("", "", False, 0.1, (1, 2, 40, 50))


def test_three_distinct_frames_confirm_once_and_cool_down():
    decision = AttendanceDecision()
    assert decision.observe(START, [KNOWN, KNOWN], START) == []
    assert decision.observe(START, [KNOWN], START) == []
    assert decision.observe(START + timedelta(milliseconds=500), [KNOWN], START + timedelta(milliseconds=500)) == []
    confirmed = decision.observe(START + timedelta(seconds=1), [KNOWN], START + timedelta(seconds=1))
    assert [face.person_id for face in confirmed] == ["P1"]
    assert decision.observe(START + timedelta(seconds=2), [KNOWN], START + timedelta(seconds=2)) == []
    for n in (61, 62):
        assert decision.observe(START + timedelta(seconds=n), [KNOWN], START + timedelta(seconds=n)) == []
    assert len(decision.observe(START + timedelta(seconds=63), [KNOWN], START + timedelta(seconds=63))) == 1


def test_unknown_stale_and_future_frames_never_confirm():
    decision = AttendanceDecision()
    for n in range(4):
        now = START + timedelta(seconds=n)
        assert decision.observe(START, [KNOWN], now + timedelta(seconds=4)) == []
        assert decision.observe(now, [UNKNOWN], now) == []
        assert decision.observe(now + timedelta(seconds=2), [KNOWN], now) == []
