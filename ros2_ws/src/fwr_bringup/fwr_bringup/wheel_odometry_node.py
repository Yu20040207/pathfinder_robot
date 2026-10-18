"""Read ESP32-S3 encoder diagnostics and publish inspection wheel odometry."""

import errno
import fcntl
import math
import os
import termios
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .wheel_odometry import WheelOdometry, parse_encoder_line


DEFAULT_DEVICE = (
    '/dev/serial/by-id/'
    'usb-Espressif_USB_JTAG_serial_debug_unit_90:70:69:F7:F0:B0-if00'
)


def open_serial(device: str) -> int:
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = (attrs[2] & ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)) | termios.CS8 | termios.CLOCAL | termios.CREAD
        if hasattr(termios, 'CRTSCTS'):
            attrs[2] &= ~termios.CRTSCTS
        attrs[3] = 0
        attrs[4] = termios.B115200
        attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return fd
    except BaseException:
        os.close(fd)
        raise


class WheelOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__('wheel_odometry')
        self.declare_parameter('device', DEFAULT_DEVICE)
        self.declare_parameter('wheel_circumference_m', 0.210)
        self.declare_parameter('wheel_track_m', 0.263)
        self.declare_parameter('counts_per_revolution', 1468)
        self.declare_parameter('poll_rate_hz', 10.0)
        self.declare_parameter('odom_topic', '/wheel/odom')
        circumference = float(self.get_parameter('wheel_circumference_m').value)
        track = float(self.get_parameter('wheel_track_m').value)
        counts = int(self.get_parameter('counts_per_revolution').value)
        rate = float(self.get_parameter('poll_rate_hz').value)
        if not 1.0 <= rate <= 20.0:
            raise ValueError('poll_rate_hz must be between 1 and 20')
        self._odometry = WheelOdometry(circumference, track, counts)
        self._device = str(self.get_parameter('device').value)
        self._publisher = self.create_publisher(
            Odometry, str(self.get_parameter('odom_topic').value), 10)
        self._fd = None
        self._pending = bytearray()
        self._last_open_attempt = 0.0
        self._timer = self.create_timer(1.0 / rate, self._tick)

    def _open_if_needed(self) -> None:
        if self._fd is not None or time.monotonic() - self._last_open_attempt < 1.0:
            return
        self._last_open_attempt = time.monotonic()
        try:
            self._fd = open_serial(self._device)
            self._pending.clear()
            self._odometry.reset_reference()
            self.get_logger().info(f'Connected to ESP32-S3 at {self._device}')
        except OSError as exc:
            self.get_logger().warn(f'Cannot open {self._device}: {exc}', throttle_duration_sec=5.0)

    def _disconnect(self, reason: str) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
        self._pending.clear()
        self._odometry.reset_reference()
        self.get_logger().warn(f'Encoder serial disconnected: {reason}')

    def _consume(self, chunk: bytes) -> None:
        self._pending.extend(chunk)
        if len(self._pending) > 8192:
            self._pending.clear()
            self.get_logger().warn('Encoder serial line exceeded 8192 bytes')
            return
        while b'\n' in self._pending:
            line, _, rest = self._pending.partition(b'\n')
            self._pending = bytearray(rest)
            parsed = parse_encoder_line(line.decode('ascii', errors='replace'))
            if parsed is None:
                continue
            boot_ms, left, left_skipped, right, right_skipped = parsed
            sample = self._odometry.update(left, right, time.monotonic(), boot_ms)
            if left_skipped or right_skipped:
                self.get_logger().warn(
                    f'Encoder invalid transitions: left={left_skipped}, right={right_skipped}',
                    throttle_duration_sec=5.0)
            if sample is not None:
                self._publish(sample)

    def _publish(self, sample) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = sample.x
        msg.pose.pose.position.y = sample.y
        msg.pose.pose.orientation.z = math.sin(sample.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(sample.yaw / 2.0)
        msg.twist.twist.linear.x = sample.linear_mps
        msg.twist.twist.angular.z = sample.angular_rps
        # Ground slip and effective track have not yet been calibrated.
        for index in (0, 7, 35):
            msg.pose.covariance[index] = 1e3
            msg.twist.covariance[index] = 1e3
        for index in (14, 21, 28):
            msg.pose.covariance[index] = 1e6
            msg.twist.covariance[index] = 1e6
        self._publisher.publish(msg)

    def _tick(self) -> None:
        self._open_if_needed()
        if self._fd is None:
            return
        try:
            while True:
                chunk = os.read(self._fd, 4096)
                if not chunk:
                    if not os.path.exists(self._device):
                        self._disconnect('device path disappeared')
                        return
                    break
                self._consume(chunk)
            os.write(self._fd, b'r')
        except BlockingIOError:
            pass
        except OSError as exc:
            if exc.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                self._disconnect(str(exc))

    def destroy_node(self) -> bool:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = WheelOdometryNode()
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
