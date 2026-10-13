"""Isolated ROS 2 demo: synthetic map and simulated differential robot only."""

import math
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from fwr_navigation.simulation import DifferentialPose, WaypointFollower


class NavigationDemoNode(Node):
    def __init__(self):
        super().__init__("navigation_demo_node")
        self.board = OccupancyGrid()
        self.board.header.frame_id = "demo_map"
        self.board.info.resolution = 0.25
        self.board.info.width = 12
        self.board.info.height = 12
        self.board.info.origin.orientation.w = 1.0
        self.board.data = [100 if x in (5, 6) and y < 8 else 0
                           for y in range(12) for x in range(12)]
        self.goal = PoseStamped()
        self.goal.header.frame_id = "demo_map"
        self.goal.pose.position.x = 2.375
        self.goal.pose.position.y = 0.625
        self.goal.pose.orientation.w = 1.0
        self.pose = DifferentialPose(0.625, 0.625, 0.0)
        self.follower = WaypointFollower(max_linear=0.35, max_angular=0.9,
                                         position_tolerance=0.07)
        self.wheel_separation_m = 0.4  # demo-only; not a hardware measurement
        self.path_received = False
        self.done_logged = False
        self.last_tick = time.monotonic()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_publisher = self.create_publisher(OccupancyGrid, "/demo/map", latched)
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, "/demo/pose", 10)
        self.goal_publisher = self.create_publisher(PoseStamped, "/demo/goal", 10)
        self.reached_publisher = self.create_publisher(Bool, "/demo/reached", latched)
        self.create_subscription(Path, "/demo/path", self._on_path, latched)
        self.create_timer(0.05, self._tick)
        self.create_timer(1.0, self._announce)
        self.get_logger().info("Demo only: no serial device, motor command or real robot control")

    def _on_path(self, message):
        if self.path_received or message.header.frame_id != "demo_map" or not message.poses:
            return
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]
        self.follower.set_path(points)
        self.path_received = True
        self.get_logger().info(f"Demo route received: {len(points)} points")

    def _announce(self):
        now = self.get_clock().now().to_msg()
        self.board.header.stamp = now
        self.goal.header.stamp = now
        self.map_publisher.publish(self.board)
        if not self.path_received:
            self.goal_publisher.publish(self.goal)

    def _tick(self):
        now_mono = time.monotonic()
        dt = min(max(now_mono - self.last_tick, 0.0), 0.1)
        self.last_tick = now_mono
        if self.path_received and not self.follower.reached:
            command = self.follower.command(self.pose)
            left = command.linear - command.angular * self.wheel_separation_m / 2
            right = command.linear + command.angular * self.wheel_separation_m / 2
            self.pose = self.pose.step(left, right, dt, self.wheel_separation_m)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "demo_map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = self.pose.x
        pose.pose.pose.position.y = self.pose.y
        pose.pose.pose.orientation.z = math.sin(self.pose.yaw / 2)
        pose.pose.pose.orientation.w = math.cos(self.pose.yaw / 2)
        self.pose_publisher.publish(pose)
        reached = bool(self.path_received and self.follower.reached)
        self.reached_publisher.publish(Bool(data=reached))
        if reached and not self.done_logged:
            self.done_logged = True
            self.get_logger().info(f"Demo goal reached at ({self.pose.x:.2f}, {self.pose.y:.2f}) m")


def main():
    rclpy.init()
    node = None
    try:
        node = NavigationDemoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
