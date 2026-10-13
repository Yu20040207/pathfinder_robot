#!/usr/bin/env python3
"""Publish illustrative front-caster steering and rolling for the RViz preview."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CasterPreview(Node):
    def __init__(self):
        super().__init__("caster_preview")
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.start = self.get_clock().now()
        self.create_timer(0.05, self.publish_state)

    def publish_state(self):
        now = self.get_clock().now()
        seconds = (now - self.start).nanoseconds / 1e9
        steering = 0.75 * math.sin(seconds * 0.8)
        state = JointState()
        state.header.stamp = now.to_msg()
        state.name = [
            "front_left_caster_steer_joint",
            "front_left_caster_roll_joint",
            "front_right_caster_steer_joint",
            "front_right_caster_roll_joint",
        ]
        state.position = [steering, seconds * 2, steering, seconds * 2]
        self.publisher.publish(state)


def main():
    rclpy.init()
    node = CasterPreview()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
