"""ROS 2 adapter for the camera-independent face engine."""

from dataclasses import replace
import math
import os
from pathlib import Path
import time

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy
from sensor_msgs.msg import Image

from fwr_data.store import AttendanceStore
from fwr_interfaces.msg import FaceObservation, FaceObservationArray
from fwr_vision.depth import sample_distance
from fwr_vision.engine import FaceEngine, FaceResult
from fwr_vision.models import load_models


def to_message(header, results: list[FaceResult]) -> FaceObservationArray:
    message = FaceObservationArray()
    message.header = header
    for result in results:
        face = FaceObservation()
        face.person_id = result.person_id
        face.display_name = result.display_name
        face.recognized = result.recognized
        face.score = float(result.score)
        face.x, face.y, face.width, face.height = result.box
        face.has_distance = result.distance_m is not None
        face.distance_m = float(result.distance_m or 0.0)
        message.faces.append(face)
    return message


def _stamp_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec / 1_000_000_000


def attach_optional_depth(results, color_stamp, depth_message, bridge, aligned: bool,
                          max_delta_sec: float, on_error=None):
    if not aligned or depth_message is None:
        return results
    if abs(_stamp_seconds(color_stamp) - _stamp_seconds(depth_message.header.stamp)) > max_delta_sec:
        return results
    try:
        depth_array = bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
        return [replace(face, distance_m=sample_distance(
            depth_array, face.box, depth_message.encoding, True)) for face in results]
    except Exception as error:
        if on_error is not None:
            on_error(f"Depth unavailable for this frame: {error}")
        return results


def apply_distance_filter(results, minimum_m: float, maximum_m: float):
    if minimum_m < 0 or maximum_m <= minimum_m:
        raise ValueError("Invalid face distance range")
    return [
        replace(face, person_id="", display_name="", recognized=False)
        if face.distance_m is not None and
        (not math.isfinite(face.distance_m) or not minimum_m <= face.distance_m <= maximum_m)
        else face
        for face in results
    ]


class FaceNode(Node):
    def __init__(self):
        super().__init__("face_node")
        base = Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local/share")) / "four_wheel_robot"
        model_dir = Path(os.environ.get("FWR_MODELS_DIR", base / "models"))
        self.declare_parameter("db_path", str(base / "attendance.sqlite3"))
        self.declare_parameter("detector_path", str(model_dir / "face_detection_yunet_2023mar.onnx"))
        self.declare_parameter("recognizer_path", str(model_dir / "face_recognition_sface_2021dec.onnx"))
        self.declare_parameter("color_topic", "/robot/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/robot/camera/depth/image_raw")
        self.declare_parameter("depth_aligned", False)
        self.declare_parameter("max_depth_delta_sec", 0.05)
        self.declare_parameter("max_fps", 5.0)
        self.declare_parameter("score_threshold", 0.363)
        self.declare_parameter("identity_margin", 0.05)
        self.declare_parameter("blur_variance_min", 25.0)
        self.declare_parameter("min_face_distance_m", 0.4)
        self.declare_parameter("max_face_distance_m", 3.0)
        detector, recognizer = load_models(
            Path(self.get_parameter("detector_path").value),
            Path(self.get_parameter("recognizer_path").value),
        )
        self.engine = FaceEngine(detector, recognizer,
                                 self.get_parameter("score_threshold").value,
                                 self.get_parameter("identity_margin").value,
                                 self.get_parameter("blur_variance_min").value)
        self.store = AttendanceStore(Path(self.get_parameter("db_path").value))
        self.bridge = CvBridge()
        self.depth_image = None
        self.last_processed = 0.0
        self.publisher = self.create_publisher(FaceObservationArray, "/robot/vision/faces", 10)
        self.create_subscription(Image, self.get_parameter("color_topic").value,
                                 self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter("depth_topic").value,
                                 self.on_depth, qos_profile_sensor_data)

    def on_depth(self, message: Image):
        self.depth_image = message

    def on_color(self, message: Image):
        now = time.monotonic()
        if now - self.last_processed < 1.0 / max(0.1, self.get_parameter("max_fps").value):
            return
        self.last_processed = now
        try:
            bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            results = self.engine.analyze(bgr, self.store.templates())
            results = attach_optional_depth(
                results, message.header.stamp, self.depth_image, self.bridge,
                self.get_parameter("depth_aligned").value,
                self.get_parameter("max_depth_delta_sec").value,
                self.get_logger().warning,
            )
            if self.get_parameter("depth_aligned").value:
                results = apply_distance_filter(
                    results,
                    self.get_parameter("min_face_distance_m").value,
                    self.get_parameter("max_face_distance_m").value,
                )
            self.publisher.publish(to_message(message.header, results))
        except Exception as error:
            self.get_logger().error(f"Could not process camera frame: {error}")


def main():
    rclpy.init()
    node = FaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
