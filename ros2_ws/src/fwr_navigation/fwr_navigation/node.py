"""ROS 2 route preview. This node never publishes a motor command."""

import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from fwr_navigation.planner import GridMap, PlanningError, plan_path


def _yaw(quaternion) -> float:
    return math.atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1 - 2 * (quaternion.y ** 2 + quaternion.z ** 2))


def grid_from_message(message: OccupancyGrid) -> GridMap:
    origin = message.info.origin
    return GridMap(message.info.width, message.info.height, message.info.resolution,
                   origin.position.x, origin.position.y, _yaw(origin.orientation),
                   tuple(message.data))


def path_message(board: GridMap, cells: list[tuple[int, int]], header,
                 start_xy: tuple[float, float], goal_xy: tuple[float, float]) -> Path:
    result = Path()
    result.header = header
    points = ([start_xy] + [board.cell_center(*cell) for cell in cells[1:-1]] + [goal_xy]
              if len(cells) > 1 or start_xy != goal_xy else [start_xy])
    for index, (x, y) in enumerate(points):
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = x
        pose.pose.position.y = y
        other = points[index + 1] if index + 1 < len(points) else points[index - 1] if index else (x, y)
        angle = math.atan2(other[1] - y, other[0] - x)
        pose.pose.orientation.z = math.sin(angle / 2)
        pose.pose.orientation.w = math.cos(angle / 2)
        result.poses.append(pose)
    return result


class PathPreviewNode(Node):
    def __init__(self):
        super().__init__("path_preview_node")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("path_topic", "/robot/navigation/planned_path")
        self.declare_parameter("clearance_m", 0.25)
        self.declare_parameter("occupancy_threshold", 50)
        self.clearance = float(self.get_parameter("clearance_m").value)
        self.threshold = int(self.get_parameter("occupancy_threshold").value)
        self.map_message = None
        self.pose_message = None
        self.goal_message = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Path, self.get_parameter("path_topic").value, latched)
        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value,
                                 self._on_map, latched)
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter("pose_topic").value,
                                 self._on_pose, 10)
        self.create_subscription(PoseStamped, self.get_parameter("goal_topic").value,
                                 self._on_goal, 10)

    def _on_map(self, message):
        self.map_message = message
        self._replan()

    def _on_pose(self, message):
        self.pose_message = message
        self._replan()

    def _on_goal(self, message):
        self.goal_message = message
        self._replan()

    def _replan(self):
        if not all((self.map_message, self.pose_message, self.goal_message)):
            return
        header = self.map_message.header
        result_header = type(header)()
        result_header.frame_id = header.frame_id
        result_header.stamp = self.get_clock().now().to_msg()
        try:
            if not header.frame_id or any(frame != header.frame_id for frame in
                                          (self.pose_message.header.frame_id,
                                           self.goal_message.header.frame_id)):
                raise PlanningError("map, pose and goal frame IDs must match")
            board = grid_from_message(self.map_message)
            start = self.pose_message.pose.pose.position
            goal = self.goal_message.pose.position
            start_xy, goal_xy = (start.x, start.y), (goal.x, goal.y)
            cells = plan_path(board, start_xy, goal_xy, clearance_m=self.clearance,
                              occupancy_threshold=self.threshold)
            path = path_message(board, cells, result_header, start_xy, goal_xy)
            self.publisher.publish(path)
        except (ValueError, OverflowError) as error:
            self.publisher.publish(Path(header=result_header))
            self.get_logger().warn(f"Path preview unavailable: {error}")


def main():
    rclpy.init()
    node = None
    try:
        node = PathPreviewNode()
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
