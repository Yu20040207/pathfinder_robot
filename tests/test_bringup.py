from datetime import datetime, timedelta, timezone
import importlib.util
from pathlib import Path
import sys

import numpy as np

root = Path(__file__).resolve().parents[1]
for package in ("fwr_data", "fwr_vision", "fwr_attendance"):
    sys.path.insert(0, str(root / f"ros2_ws/src/{package}"))

from fwr_data.store import AttendanceStore, EnrollmentSample
from fwr_vision.engine import FaceEngine
from fwr_vision.node import to_message
from fwr_attendance.decision import AttendanceDecision
from fwr_attendance.node import process_observations
from std_msgs.msg import Header


class Detector:
    def setInputSize(self, size):
        pass

    def detect(self, image):
        return True, np.array([[10, 10, 40, 40] + [0] * 11], np.float32)


class Recognizer:
    def alignCrop(self, image, face):
        return image

    def feature(self, image):
        return np.array([[1.0, 0.0]], np.float32)


def test_offline_image_to_attendance_event(tmp_path):
    store = AttendanceStore(tmp_path / "db.sqlite3")
    store.enroll("P1", "A", [EnrollmentSample(b"image", (1.0, 0.0))])
    engine = FaceEngine(Detector(), Recognizer())
    decision = AttendanceDecision()
    start = datetime(2026, 10, 4, tzinfo=timezone.utc)
    events = []
    pattern = (np.indices((80, 80)).sum(axis=0) % 2 * 255).astype(np.uint8)
    image = np.repeat(pattern[:, :, None], 3, axis=2)
    for n in range(3):
        stamp = start + timedelta(milliseconds=500 * n)
        header = Header()
        header.stamp.sec = int(stamp.timestamp())
        header.stamp.nanosec = (n % 2) * 500_000_000
        faces = engine.analyze(image, store.templates())
        events.extend(process_observations(store, decision, to_message(header, faces), stamp, "chest"))
    assert len(events) == 1
    assert len(store.events()) == 1


def test_hp60c_topic_adapter_maps_camera_streams():
    path = root / "ros2_ws/src/fwr_bringup/launch/vision_attendance.launch.py"
    spec = importlib.util.spec_from_file_location("fwr_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    remaps = dict(module.camera_remappings())
    assert remaps["~/rgb0/image"] == "/robot/camera/color/image_raw"
    assert remaps["~/depth0/image_raw"] == "/robot/camera/depth/image_raw"
    remaps_second = dict(module.camera_remappings(1))
    assert remaps_second["~/rgb1/image"] == "/robot/camera/color/image_raw"


def test_camera_input_topics_can_be_overridden_for_external_driver():
    path = root / "ros2_ws/src/fwr_bringup/launch/vision_attendance.launch.py"
    spec = importlib.util.spec_from_file_location("fwr_launch_topics", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    arguments = {
        entity.name for entity in module.generate_launch_description().entities
        if type(entity).__name__ == "DeclareLaunchArgument"
    }
    assert {"color_topic", "depth_topic"} <= arguments
