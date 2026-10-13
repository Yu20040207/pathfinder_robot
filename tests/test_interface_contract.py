import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def test_ros_messages_are_generated_and_importable():
    install = ROOT / "ros2_ws" / "install"
    command = (
        "source /opt/ros/humble/setup.bash; "
        f"source {install}/setup.bash; "
        "python3 -c 'from fwr_interfaces.msg import "
        "FaceObservation, FaceObservationArray, AttendanceEvent; "
        "assert hasattr(FaceObservation(), \"person_id\"); "
        "assert hasattr(FaceObservationArray(), \"faces\"); "
        "assert hasattr(AttendanceEvent(), \"device_id\")'"
    )
    result = subprocess.run(["bash", "-lc", command], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_runtime_data_is_ignored():
    ignore = (ROOT / ".gitignore").read_text()
    for item in (".venv/", "ros2_ws/build/", "ros2_ws/install/", "data/", "models/"):
        assert item in ignore
