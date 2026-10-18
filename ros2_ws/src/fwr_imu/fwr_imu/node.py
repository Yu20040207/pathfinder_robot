"""Publish DM-IMU-L1 USB measurements as ROS 2 Imu messages."""

import errno
import os
import termios
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

from fwr_imu.orientation import quaternion_xyzw
from fwr_imu.protocol import ImuFrameParser


DEFAULT_DEVICE = '/dev/serial/by-id/usb-DM-Tech_DM-IMU-L1_DMIMU20250212-if00'


def open_serial(device: str) -> int:
    fd = os.open(device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = (attrs[2] & ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)) | termios.CS8 | termios.CLOCAL | termios.CREAD
        if hasattr(termios, 'CRTSCTS'):
            attrs[2] &= ~termios.CRTSCTS
        attrs[3] = 0
        attrs[4] = termios.B921600
        attrs[5] = termios.B921600
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return fd
    except BaseException:
        os.close(fd)
        raise


class DmImuNode(Node):
    def __init__(self) -> None:
        super().__init__('dm_imu_l1')
        self.declare_parameter('device', DEFAULT_DEVICE)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 100.0)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if not 1.0 <= rate <= 500.0:
            raise ValueError('publish_rate_hz must be between 1 and 500')
        self._device = str(self.get_parameter('device').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._publisher = self.create_publisher(Imu, '/imu/data_raw', 10)
        self._attitude_publisher = self.create_publisher(Imu, '/imu/data', 10)
        self._pose_publisher = self.create_publisher(PoseStamped, '/imu/pose', 10)
        self._parser = ImuFrameParser()
        self._fd = None
        self._last_open_attempt = 0.0
        self._accel = None
        self._gyro = None
        self._quaternion = None
        self._fresh = set()
        self._timer = self.create_timer(1.0 / rate, self._tick)

    def _open_if_needed(self) -> None:
        if self._fd is not None or time.monotonic() - self._last_open_attempt < 1.0:
            return
        self._last_open_attempt = time.monotonic()
        try:
            self._fd = open_serial(self._device)
            self._parser = ImuFrameParser()
            self._accel = self._gyro = None
            self._quaternion = None
            self._fresh.clear()
            self.get_logger().info(f'Connected to {self._device} at 921600 8N1')
        except OSError as exc:
            self.get_logger().warn(f'Cannot open {self._device}: {exc}', throttle_duration_sec=5.0)

    def _disconnect(self, reason: str) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
        self._fresh.clear()
        self.get_logger().warn(f'IMU disconnected: {reason}')

    def _tick(self) -> None:
        self._open_if_needed()
        if self._fd is None:
            return
        try:
            while True:
                chunk = os.read(self._fd, 8192)
                if not chunk:
                    # A nonblocking CDC-ACM read returns b'' when no frame is ready.
                    if not os.path.exists(self._device):
                        self._disconnect('device path disappeared')
                        return
                    break
                for frame in self._parser.feed(chunk):
                    if frame.register == 1:
                        self._accel = frame.values
                        self._fresh.add(1)
                    elif frame.register == 2:
                        self._gyro = frame.values
                        self._fresh.add(2)
                    elif frame.register == 4:
                        self._quaternion = quaternion_xyzw(frame.values)
                        if self._quaternion is not None:
                            self._fresh.add(4)
        except BlockingIOError:
            pass
        except OSError as exc:
            if exc.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                self._disconnect(str(exc))
                return
        if not {1, 2}.issubset(self._fresh):
            return
        have_attitude = 4 in self._fresh and self._quaternion is not None
        self._fresh.clear()
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        # Axis mounting and world-frame convention are not calibrated yet.
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = self._gyro
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = self._accel
        # Zero covariance means unknown. Set measured covariances before EKF fusion.
        self._publisher.publish(msg)
        if have_attitude:
            attitude = Imu()
            attitude.header = msg.header
            attitude.orientation.x, attitude.orientation.y, attitude.orientation.z, attitude.orientation.w = self._quaternion
            attitude.angular_velocity = msg.angular_velocity
            attitude.linear_acceleration = msg.linear_acceleration
            # Covariances remain unknown. The six-axis yaw is relative and drifts.
            self._attitude_publisher.publish(attitude)
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            # Put the front arrow at the geometric center of the chassis top.
            # The preview TF rotates the entire robot instead of this arrow.
            pose.header.frame_id = 'base_link'
            pose.pose.position.z = 0.175
            pose.pose.orientation.w = 1.0
            self._pose_publisher.publish(pose)

    def destroy_node(self) -> bool:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DmImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
