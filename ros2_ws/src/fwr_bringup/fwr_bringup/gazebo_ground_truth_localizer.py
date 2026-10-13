"""Simulation-only map-to-odom alignment after Gazebo model repositioning.

The known corridor map uses Gazebo's world coordinates. Gazebo DiffDrive odometry
is wheel-relative and does not follow an externally dragged model, so this node
aligns the two frames from Gazebo world pose. Never use it on physical hardware.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

from .pose_alignment import align_map_to_odom, yaw_from_quaternion


class GazeboGroundTruthLocalizer(Node):
    def __init__(self):
        super().__init__('gazebo_ground_truth_localizer')
        self._world_base = None
        self._odom_base = None
        self._world_received = None
        self._odom_received = None
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            TFMessage, '/gazebo_world_poses', self._on_world_poses, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_timer(0.05, self._publish_alignment)

    def _on_world_poses(self, message):
        for transform in message.transforms:
            if transform.child_frame_id == 'fwr_chassis':
                p = transform.transform.translation
                q = transform.transform.rotation
                self._world_base = (p.x, p.y, yaw_from_quaternion(q))
                self._world_received = self.get_clock().now()
                break

    def _on_odom(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        self._odom_base = (p.x, p.y, yaw_from_quaternion(q))
        self._odom_received = self.get_clock().now()

    def _publish_alignment(self):
        if self._world_base is None or self._odom_base is None:
            return
        now = self.get_clock().now()
        # Avoid broadcasting an alignment computed from stale or paused inputs.
        if (now - self._world_received).nanoseconds > 500_000_000 or \
                (now - self._odom_received).nanoseconds > 500_000_000:
            return
        x, y, yaw = align_map_to_odom(self._world_base, self._odom_base)
        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self._broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = GazeboGroundTruthLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
