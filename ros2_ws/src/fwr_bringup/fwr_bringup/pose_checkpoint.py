"""Persist the last known map pose so the next start can initialise from it."""
import math
import os

import rclpy
from rclpy.node import Node
import tf2_ros
from rclpy.duration import Duration


def pose_yaml(x: float, y: float, yaw: float) -> str:
    return f'x: {x:.4f}\ny: {y:.4f}\nyaw: {yaw:.4f}\n'


class PoseCheckpoint(Node):
    def __init__(self):
        super().__init__('pose_checkpoint')
        self.declare_parameter('output_file', os.path.expanduser('~/.config/fwr/last_pose.yaml'))
        self._output = str(self.get_parameter('output_file').value)
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, self)
        self.create_timer(2.0, self._save)

    def _save(self):
        try:
            transform = self._buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except Exception:  # noqa: BLE001 - TF gaps are normal before localisation is up
            return
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        try:
            os.makedirs(os.path.dirname(self._output), exist_ok=True)
            with open(self._output, 'w', encoding='utf-8') as handle:
                handle.write(pose_yaml(t.x, t.y, yaw))
        except OSError as exc:
            self.get_logger().warn(f'cannot save pose checkpoint: {exc}',
                                   throttle_duration_sec=10.0)


def main():
    rclpy.init()
    node = PoseCheckpoint()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
