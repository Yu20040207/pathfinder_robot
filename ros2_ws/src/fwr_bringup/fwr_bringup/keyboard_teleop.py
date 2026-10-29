"""Fixed low-speed, key-repeat based teleop for supervised mapping."""

import select
import os
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from std_msgs.msg import Int32


class KeyboardCommand:
    def __init__(self, turn_rate_rad_s: float = .75,
                 forward_speed_mps: float = .10):
        if not 0 < turn_rate_rad_s <= .75:
            raise ValueError('turn rate must be within (0, 0.75] rad/s')
        if not .02 <= forward_speed_mps <= .2:
            raise ValueError('forward speed must be within [0.02, 0.2] m/s')
        self._turn_rate = turn_rate_rad_s
        self._keys = {
            'w': (forward_speed_mps, 0.0), 's': (-forward_speed_mps, 0.0),
        }
        self._value = (0.0, 0.0)
        self._last_at = None

    def press(self, key: str, now: float) -> tuple[float, float]:
        self._value = ({'a': (0.0, self._turn_rate),
                        'd': (0.0, -self._turn_rate)}.get(
                            key.lower(), self._keys.get(key.lower(), (0.0, 0.0))))
        self._last_at = now
        return self._value

    def current(self, now: float) -> tuple[float, float]:
        if self._last_at is None or now - self._last_at > .18:
            return (0.0, 0.0)
        return self._value


class KeyboardFeedback:
    _FAULTS = {
        1: '失速；请检查障碍物或车轮受阻',
        2: '轮子转向与指令相反',
        3: '轮速过高',
        4: '编码器异常',
        5: '控制时序异常',
        6: '左右轮速跟踪异常',
    }

    def __init__(self):
        self._last = (0.0, 0.0)
        self.fault_code = 0

    def command(self, value: tuple[float, float], source: str) -> str | None:
        if value == self._last:
            return None
        self._last = value
        if value == (0.0, 0.0):
            return '空格：停机' if source == ' ' else '松键：停机'
        if value[0] > 0:
            return f'W 前进 {value[0]:.2f} m/s'
        if value[0] < 0:
            return f'S 后退 {abs(value[0]):.2f} m/s'
        if value[1] > 0:
            return f'A 左转 {value[1]:.2f} rad/s'
        return f'D 右转 {abs(value[1]):.2f} rad/s'

    def fault(self, code: int) -> str | None:
        if code <= 0 or self.fault_code:
            return None
        self.fault_code = code
        return (f'底盘故障 {code}：{self._FAULTS.get(code, "未知故障")}；'
                '已停机并退出键盘控制')


def read_pending_key(fd: int) -> str:
    """Consume queued terminal repeats so a stale key cannot restart motion."""
    keys = os.read(fd, 256).decode('ascii', errors='ignore').lower()
    if 'q' in keys or '\x03' in keys:
        return 'q'
    return keys[-1:] if keys else ''


def main(args=None):
    if not sys.stdin.isatty():
        raise SystemExit('keyboard_teleop requires a focused terminal')
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = rclpy.create_node('fwr_keyboard_teleop')
    node.declare_parameter('turn_rate_rad_s', .75)
    node.declare_parameter('forward_speed_mps', .10)
    publisher = node.create_publisher(Twist, '/cmd_vel_manual', 10)
    command = KeyboardCommand(
        float(node.get_parameter('turn_rate_rad_s').value),
        float(node.get_parameter('forward_speed_mps').value))
    feedback = KeyboardFeedback()

    def on_fault(msg: Int32) -> None:
        notice = feedback.fault(msg.data)
        if notice:
            print(notice, file=sys.stderr, flush=True)

    node.create_subscription(Int32, '/base/fault', on_fault, 10)
    last_sent = None
    try:
        tty.setcbreak(sys.stdin.fileno())
        print('W/S 前进后退，A/D 原地左右转，空格停机，Q 退出。按键需持续重复；松键约 0.18 秒停机。', flush=True)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            if feedback.fault_code:
                break
            ready, _, _ = select.select([sys.stdin], [], [], .05)
            if ready:
                key = read_pending_key(sys.stdin.fileno())
                if key.lower() == 'q' or key == '\x03':
                    print('退出：停机', flush=True)
                    break
                value = command.press(key, time.monotonic())
                notice = feedback.command(value, key)
                if notice:
                    print(notice, flush=True)
            value = command.current(time.monotonic())
            notice = feedback.command(value, 'timeout')
            if notice:
                print(notice, flush=True)
            if value != last_sent or value != (0.0, 0.0):
                msg = Twist()
                msg.linear.x, msg.angular.z = value
                publisher.publish(msg)
                last_sent = value
    finally:
        msg = Twist()
        for _ in range(3):
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=.01)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
    if feedback.fault_code:
        raise SystemExit(2)
