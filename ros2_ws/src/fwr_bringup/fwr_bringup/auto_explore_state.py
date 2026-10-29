"""Publish /explore/state = 'navigating' so the safety chain stays open while
explore_lite drives the robot to map frontiers."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AutoExploreState(Node):
    def __init__(self):
        super().__init__('auto_explore_state')
        self.declare_parameter('state', 'navigating')
        self.declare_parameter('rate_hz', 10.0)
        self._state = str(self.get_parameter('state').value)
        self._pub = self.create_publisher(String, '/explore/state', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)

    def _tick(self):
        msg = String()
        msg.data = self._state
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = AutoExploreState()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
