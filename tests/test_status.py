from pathlib import Path
import stat
import sys

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "ros2_ws/src/fwr_attendance"))
from fwr_attendance.status import StatusFile


def test_error_survives_process_restart_and_file_is_private(tmp_path):
    path = tmp_path / "attendance-status.json"
    StatusFile(path).set_error("database unavailable")
    assert StatusFile(path).get_error() == "database unavailable"
    assert stat.S_IMODE(path.stat().st_mode) == 0o600
    StatusFile(path).clear_error()
    assert StatusFile(path).get_error() is None
