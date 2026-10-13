from datetime import datetime, timezone
from pathlib import Path
import stat
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/fwr_data"))
from fwr_data.store import AttendanceStore, EnrollmentSample


def sample(value=0.25):
    return EnrollmentSample(b"photo", (value, 0.5, 0.75))


def test_enrollment_is_persistent_and_duplicate_id_rejected(tmp_path):
    path = tmp_path / "attendance.sqlite3"
    AttendanceStore(path).enroll("P_01", "张三", [sample(), sample(0.3)])
    store = AttendanceStore(path)
    assert len(store.templates()) == 2
    assert store.templates()[0].person_id == "P_01"
    with pytest.raises(ValueError, match="already exists"):
        store.enroll("P_01", "李四", [sample()])
    assert len(store.templates()) == 2


def test_invalid_sample_rolls_back_person(tmp_path):
    store = AttendanceStore(tmp_path / "db.sqlite3")
    with pytest.raises(ValueError):
        store.enroll("P02", "A", [sample(), EnrollmentSample(b"", (1.0,))])
    assert store.templates() == []


def test_delete_clears_templates_but_preserves_event(tmp_path):
    path = tmp_path / "db.sqlite3"
    store = AttendanceStore(path)
    store.enroll("P03", "A", [sample()])
    event_id = store.record_event("P03", "A", datetime(2026, 10, 4, tzinfo=timezone.utc), 0.8, "chest")
    assert event_id > 0
    assert store.delete_person("P03") is True
    assert AttendanceStore(path).templates() == []
    assert AttendanceStore(path).events()[0].person_id == "P03"


def test_biometric_database_is_private(tmp_path):
    path = tmp_path / "people.sqlite3"
    store = AttendanceStore(path)
    assert stat.S_IMODE(path.stat().st_mode) == 0o600
    with store._connect() as db:
        db.execute("INSERT INTO people VALUES (?,?)", ("P1", "A"))
        wal = Path(str(path) + "-wal")
        assert wal.exists()
        assert stat.S_IMODE(wal.stat().st_mode) == 0o600
