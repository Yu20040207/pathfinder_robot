from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "ros2_ws/src/fwr_data"))
sys.path.insert(0, str(root / "ros2_ws/src/fwr_vision"))

from std_msgs.msg import Header
from fwr_vision.engine import FaceResult
from fwr_vision.node import to_message, attach_optional_depth, apply_distance_filter


def test_observation_message_keeps_capture_stamp_and_unknown_state():
    header = Header()
    header.stamp.sec = 42
    header.frame_id = "camera_rgb"
    msg = to_message(header, [FaceResult("", "", False, 0.2, (1, 2, 30, 40))])
    assert msg.header.stamp.sec == 42
    assert msg.faces[0].recognized is False
    assert msg.faces[0].has_distance is False
    assert msg.faces[0].width == 30


def test_bad_depth_decoding_keeps_rgb_face_result():
    class BrokenBridge:
        def imgmsg_to_cv2(self, *_args, **_kwargs):
            raise ValueError("unsupported depth encoding")

    class DepthMessage:
        header = Header()
        encoding = "unrecognized"

    header = Header()
    header.stamp.sec = 42
    depth = DepthMessage()
    depth.header.stamp.sec = 42
    face = FaceResult("P1", "A", True, 0.8, (1, 2, 30, 40))
    assert attach_optional_depth([face], header.stamp, depth, BrokenBridge(), True, 0.05) == [face]


def test_valid_out_of_range_depth_makes_identity_unknown():
    face = FaceResult("P1", "A", True, 0.8, (1, 2, 30, 40), 0.2)
    filtered = apply_distance_filter([face], 0.4, 3.0)[0]
    assert filtered.recognized is False
    assert filtered.person_id == ""
    assert apply_distance_filter([FaceResult("P1", "A", True, 0.8, (1, 2, 30, 40))], 0.4, 3.0)[0].recognized
