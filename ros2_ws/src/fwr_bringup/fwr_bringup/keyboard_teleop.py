"""Fixed low-speed, key-repeat based teleop for supervised mapping."""

import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy


class KeyboardCommand:
    _KEYS = {
        'w': (.10, 0.0), 's': (-.10, 0.0),
        'a': (0.0, .75), 'd': (0.0, -.75),
    }

    def __init__(self):
        self._value = (0.0, 0.0)
        self._last_at = None

    def press(self, key: str, now: float) -> tuple[float, float]:
        self._value = self._KEYS.get(key.lower(), (0.0, 0.0))
        self._last_at = now
        return self._value

    def current(self, now: float) -> tuple[float, float]:
        if self._last_at is None or now - self._last_at > .18:
            return (0.0, 0.0)
        return self._value


def main(args=None):
    if not sys.stdin.isatty():
        raise SystemExit('keyboard_teleop requires a focused terminal')
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = rclpy.create_node('fwr_keyboard_teleop')
    publisher = node.create_publisher(Twist, '/cmd_vel_manual', 10)
    command = KeyboardCommand()
    last_sent = None
    try:
        tty.setcbreak(sys.stdin.fileno())
        print('W/S 前进后退，A/D 原地左右转，空格停机，Q 退出。按键需持续重复；松键约 0.18 秒停机。', flush=True)
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], .05)
            if ready:
                key = sys.stdin.read(1)
                if key.lower() == 'q' or key == '\x03':
                    break
                command.press(key, time.monotonic())
            value = command.current(time.monotonic())
            if value != last_sent or value != (0.0, 0.0):
                msg = Twist()
                msg.linear.x, msg.angular.z = value
                publisher.publish(msg)
                last_sent = value
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        msg = Twist()
        for _ in range(3):
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=.01)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
