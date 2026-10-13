"""RViz-only moving chassis driven by the voice demo velocity topic."""

import math
import time

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from .simulation import ChassisState, DemoVelocityLatch


class VoiceChassisDemo(Node):
    def __init__(self):
        super().__init__("voice_chassis_demo")
        self.chassis = ChassisState()
        self.velocity = DemoVelocityLatch()
        self.last_tick = time.monotonic()
        self.last_path_sample = self.last_tick
        self.path = Path()
        self.path.header.frame_id = "odom"
        self.odom_pub = self.create_publisher(Odometry, "/demo/odom", 10)
        self.path_pub = self.create_publisher(Path, "/demo/chassis_path", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(Twist, "/demo/cmd_vel", self._on_velocity, 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info("RViz kinematic chassis ready on /demo/cmd_vel")

    def _on_velocity(self, msg):
        try:
            self.velocity.accept(msg.linear.x, msg.angular.z)
        except ValueError:
            self.get_logger().warning("Ignored invalid demo velocity")

    def _tick(self):
        now_mono = time.monotonic()
        dt = min(max(now_mono - self.last_tick, 0.0), 0.1)
        self.last_tick = now_mono
        linear, angular = self.velocity.current(now_mono)
        self.chassis.step(linear, angular, dt)
        stamp = self.get_clock().now().to_msg()
        pose = self.chassis.pose
        z = math.sin(pose.yaw / 2)
        w = math.cos(pose.yaw / 2)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = pose.x
        transform.transform.translation.y = pose.y
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w
        self.tf_pub.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.z = z
        odom.pose.pose.orientation.w = w
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        self.odom_pub.publish(odom)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ["left_drive_wheel_preview_joint", "right_drive_wheel_preview_joint",
                       "front_left_caster_steer_joint", "front_left_caster_roll_joint",
                       "front_right_caster_steer_joint", "front_right_caster_roll_joint"]
        joints.position = [self.chassis.left_wheel_rad, self.chassis.right_wheel_rad,
                           self.chassis.front_left_steer_rad, self.chassis.front_left_roll_rad,
                           self.chassis.front_right_steer_rad, self.chassis.front_right_roll_rad]
        self.joint_pub.publish(joints)

        if now_mono - self.last_path_sample >= 0.1:
            point = PoseStamped()
            point.header.stamp = stamp
            point.header.frame_id = "odom"
            point.pose = odom.pose.pose
            self.path.poses.append(point)
            self.path.poses = self.path.poses[-1000:]
            self.path.header.stamp = stamp
            self.path_pub.publish(self.path)
            self.last_path_sample = now_mono


def main():
    rclpy.init()
    node = VoiceChassisDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
