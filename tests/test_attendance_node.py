from datetime import datetime, timedelta, timezone
from pathlib import Path
import sqlite3
import sys

import pytest

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "ros2_ws/src/fwr_data"))
sys.path.insert(0, str(root / "ros2_ws/src/fwr_vision"))
sys.path.insert(0, str(root / "ros2_ws/src/fwr_attendance"))

from fwr_data.store import AttendanceStore
from fwr_attendance.decision import AttendanceDecision
from fwr_attendance.node import process_observations
from fwr_vision.engine import FaceResult
from fwr_vision.node import to_message
from std_msgs.msg import Header


def test_event_is_published_only_after_database_write(tmp_path):
    store = AttendanceStore(tmp_path / "db.sqlite3")
    decision = AttendanceDecision()
    start = datetime(2026, 10, 4, tzinfo=timezone.utc)
    face = FaceResult("P1", "A", True, 0.8, (1, 2, 40, 50))
    events = []
    for n in range(3):
        header = Header()
        header.stamp.sec = int(start.timestamp()) + n
        message = to_message(header, [face])
        events.extend(process_observations(store, decision, message, start + timedelta(seconds=n), "chest"))
    assert len(events) == 1
    assert events[0].person_id == "P1"
    assert len(store.events()) == 1


def test_multi_person_write_failure_rolls_back_every_confirmation(tmp_path):
    path = tmp_path / "db.sqlite3"
    store = AttendanceStore(path)
    decision = AttendanceDecision()
    start = datetime(2026, 10, 4, tzinfo=timezone.utc)
    faces = [FaceResult(pid, pid, True, 0.8, (1, 2, 40, 50)) for pid in ("P1", "P2", "P3")]
    with sqlite3.connect(path) as db:
        db.execute("CREATE TRIGGER reject_p2 BEFORE INSERT ON events WHEN NEW.person_id='P2' BEGIN SELECT RAISE(ABORT, 'injected failure'); END")
    for n in range(2):
        header = Header()
        header.stamp.sec = int(start.timestamp()) + n
        assert process_observations(store, decision, to_message(header, faces), start + timedelta(seconds=n), "chest") == []
    header = Header()
    header.stamp.sec = int(start.timestamp()) + 2
    with pytest.raises(sqlite3.DatabaseError, match="injected failure"):
        process_observations(store, decision, to_message(header, faces), start + timedelta(seconds=2), "chest")
    assert store.events() == []
    assert decision.last_confirmation == {}
