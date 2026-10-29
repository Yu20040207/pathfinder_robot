"""Publish a stationary pose for the URDF's movable preview joints."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = (
    'left_drive_wheel_preview_joint',
    'right_drive_wheel_preview_joint',
    'front_left_caster_steer_joint',
    'front_left_caster_roll_joint',
    'front_right_caster_steer_joint',
    'front_right_caster_roll_joint',
)


class CadPreviewJointStates(Node):
    def __init__(self):
        super().__init__('cad_preview_joint_states')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.2, self.publish_pose)

    def publish_pose(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = [0.0] * len(JOINT_NAMES)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CadPreviewJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
