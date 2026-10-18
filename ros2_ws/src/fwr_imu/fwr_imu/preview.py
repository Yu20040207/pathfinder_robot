"""Preview-only relative IMU attitude TF; never use alongside wheel odometry."""

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from fwr_imu.orientation import quaternion_xyzw, relative_quaternion_xyzw


class ImuPreview(Node):
    def __init__(self) -> None:
        super().__init__('imu_heading_preview')
        self._reference = None
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(Imu, '/imu/data', self._attitude, 10)

    def _attitude(self, msg: Imu) -> None:
        q = quaternion_xyzw((msg.orientation.w, msg.orientation.x,
                             msg.orientation.y, msg.orientation.z))
        if q is None:
            return
        if self._reference is None:
            self._reference = q
            self.get_logger().info('IMU preview relative heading set to zero')
        x, y, z, w = relative_quaternion_xyzw(self._reference, q)
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'imu_preview'
        transform.child_frame_id = 'base_footprint'
        transform.transform.rotation.x = x
        transform.transform.rotation.y = y
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w
        self._broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
