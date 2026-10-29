"""Periodically persist the slam_toolbox map (image + posegraph) while driving."""
import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap, SerializePoseGraph


class MapAutosave(Node):
    def __init__(self):
        super().__init__('map_autosave')
        self.declare_parameter('map_name', '')
        self.declare_parameter('period_s', 120.0)
        self._name = str(self.get_parameter('map_name').value)
        self._save = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self._serialize = self.create_client(SerializePoseGraph,
                                             '/slam_toolbox/serialize_map')
        period = max(30.0, float(self.get_parameter('period_s').value))
        if not self._name:
            self.get_logger().warn('map_name is empty; autosave disabled')
            return
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'autosaving map to {self._name} every {period:.0f} s (slam_toolbox)')

    def _tick(self):
        if not self._save.service_is_ready() or not self._serialize.service_is_ready():
            self.get_logger().warn('slam_toolbox save services not ready yet',
                                   throttle_duration_sec=60.0)
            return
        request = SerializePoseGraph.Request()
        request.filename = self._name
        self._serialize.call_async(request)
        save = SaveMap.Request()
        save.name = self._name
        self._save.call_async(save)
        self.get_logger().info('map autosaved (image + posegraph)')


def main():
    rclpy.init()
    node = MapAutosave()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
