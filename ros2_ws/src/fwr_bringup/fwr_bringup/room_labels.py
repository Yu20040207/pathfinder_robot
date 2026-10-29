"""Publish room names as RViz text markers and navigate to them by name."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from .room_labels_core import RoomError, find_room, load_rooms


class RoomLabels(Node):
    def __init__(self):
        super().__init__('room_labels')
        self.declare_parameter('rooms_file', '')
        self.declare_parameter('frame_id', 'map')
        path = str(self.get_parameter('rooms_file').value)
        self._rooms = []
        if path:
            try:
                with open(path, 'r', encoding='utf-8') as handle:
                    self._rooms = load_rooms(handle.read())
                self.get_logger().info(f'loaded {len(self._rooms)} rooms from {path}')
            except (OSError, RoomError) as exc:
                self.get_logger().error(f'cannot load rooms: {exc}')
        else:
            self.get_logger().warn('rooms_file is empty; no labels published')
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(MarkerArray, '/room_labels', latch)
        self._navigator = None
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(String, '/room_goal', self._on_goal, 10)
        self.create_timer(2.0, self._publish)

    def _publish(self):
        if not self._rooms:
            return
        frame = str(self.get_parameter('frame_id').value)
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for index, room in enumerate(self._rooms):
            text = Marker()
            text.header.frame_id = frame
            text.ns = 'room_name'
            text.id = index
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = room['x']
            text.pose.position.y = room['y']
            text.pose.position.z = 0.6
            text.scale.z = 0.35
            text.color.r = 1.0
            text.color.g = 0.85
            text.color.b = 0.1
            text.color.a = 1.0
            text.text = room['name']
            array.markers.append(text)
        self._publisher.publish(array)

    def _on_goal(self, msg):
        room = find_room(self._rooms, msg.data.strip())
        if room is None:
            self.get_logger().warn(f'unknown room: {msg.data!r}')
            return
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server is not available')
            return
        pose = PoseStamped()
        pose.header.frame_id = str(self.get_parameter('frame_id').value)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = room['x']
        pose.pose.position.y = room['y']
        pose.pose.orientation.z = math.sin(room['yaw'] / 2.0)
        pose.pose.orientation.w = math.cos(room['yaw'] / 2.0)
        self.get_logger().info(
            f'navigating to room {room["name"]} '
            '(motion gate and arm must already be enabled by the panel)')
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._client.send_goal_async(goal)


def main():
    rclpy.init()
    node = RoomLabels()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
