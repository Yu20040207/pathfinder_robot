"""Re-arm the base automatically when a transient sensor glitch disarms it.

base_bridge 里任何一次传感器瞬时异常都会把 auto_armed 永久置 False，直到有人再
调用 /base/arm_auto。自动建图场景没有人点按钮，这里检测到解除授权且底盘就绪就
自动重新授权。
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class AutoArmWatchdog(Node):
    def __init__(self):
        super().__init__('auto_arm_watchdog')
        self.declare_parameter('retry_delay_s', 2.0)
        self._armed = False
        self._armed_at = 0.0
        self._last_try = 0.0
        self._client = self.create_client(Trigger, '/base/arm_auto')
        self.create_subscription(Bool, '/base/auto_armed', self._on_armed, 10)
        self.create_timer(1.0, self._tick)

    def _on_armed(self, msg):
        if msg.data and not self._armed:
            self.get_logger().info('base armed')
        self._armed = bool(msg.data)
        if self._armed:
            self._armed_at = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        if self._armed or now - self._armed_at < 0.0:
            self._armed_at = now if not self._armed else self._armed_at
        if self._armed:
            return
        if now - self._last_try < float(self.get_parameter('retry_delay_s').value):
            return
        if not self._client.service_is_ready():
            return
        self._last_try = now
        self.get_logger().warn('base disarmed; re-arming')
        request = Trigger.Request()
        future = self._client.call_async(request)
        future.add_done_callback(self._done)

    def _done(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('re-armed')
            else:
                self.get_logger().warn(f're-arm failed: {result.message}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f're-arm error: {exc}')


def main():
    rclpy.init()
    node = AutoArmWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
