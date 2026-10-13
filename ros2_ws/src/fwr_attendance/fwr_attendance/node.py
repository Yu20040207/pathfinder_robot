"""ROS 2 adapter that commits attendance before publishing events."""

from datetime import datetime, timezone
import os
from pathlib import Path

import rclpy
from rclpy.node import Node

from fwr_data.store import AttendanceStore
from fwr_interfaces.msg import AttendanceEvent, FaceObservationArray
from fwr_vision.engine import FaceResult
from fwr_attendance.decision import AttendanceDecision
from fwr_attendance.status import StatusFile


def process_observations(store: AttendanceStore, decision: AttendanceDecision,
                         message: FaceObservationArray, now: datetime,
                         device_id: str) -> list[AttendanceEvent]:
    stamp = message.header.stamp
    frame_time = datetime.fromtimestamp(stamp.sec + stamp.nanosec / 1_000_000_000, timezone.utc)
    faces = [FaceResult(f.person_id, f.display_name, f.recognized, f.score,
                        (f.x, f.y, f.width, f.height), f.distance_m if f.has_distance else None)
             for f in message.faces]
    confirmed = decision.observe(frame_time, faces, now)
    if not confirmed:
        return []
    try:
        store.record_events_batch([
            (face.person_id, face.display_name, frame_time, face.score, device_id)
            for face in confirmed
        ])
    except Exception:
        for face in confirmed:
            decision.rollback_confirmation(face.person_id)
        raise
    events = []
    for face in confirmed:
        event = AttendanceEvent()
        event.header = message.header
        event.person_id = face.person_id
        event.display_name = face.display_name
        event.score = face.score
        event.device_id = device_id
        events.append(event)
    return events


class AttendanceNode(Node):
    def __init__(self):
        super().__init__("attendance_node")
        base = Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local/share")) / "four_wheel_robot"
        self.declare_parameter("db_path", str(base / "attendance.sqlite3"))
        self.declare_parameter("device_id", "chest_hp60c")
        self.declare_parameter("window_seconds", 2.0)
        self.declare_parameter("hits_required", 3)
        self.declare_parameter("cooldown_seconds", 60.0)
        self.store = AttendanceStore(Path(self.get_parameter("db_path").value))
        self.status = StatusFile(self.store.path.parent / "attendance-status.json")
        self.decision = AttendanceDecision(
            self.get_parameter("window_seconds").value,
            self.get_parameter("hits_required").value,
            self.get_parameter("cooldown_seconds").value,
        )
        self.device_id = self.get_parameter("device_id").value
        self.publisher = self.create_publisher(AttendanceEvent, "/robot/attendance/events", 10)
        self.create_subscription(FaceObservationArray, "/robot/vision/faces", self.on_faces, 10)

    def on_faces(self, message: FaceObservationArray):
        try:
            events = process_observations(self.store, self.decision, message,
                                          datetime.now(timezone.utc), self.device_id)
            for event in events:
                self.publisher.publish(event)
            if events:
                self.status.clear_error()
        except Exception as error:
            self.get_logger().error(f"Attendance database write failed: {error}")
            try:
                self.status.set_error(f"Attendance database write failed: {error}")
            except OSError as status_error:
                self.get_logger().error(f"Could not persist attendance error status: {status_error}")


def main():
    rclpy.init()
    node = AttendanceNode()
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
