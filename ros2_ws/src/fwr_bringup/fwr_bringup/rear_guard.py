#!/usr/bin/env python3
"""后退近障停车守卫：车尾四路超声低于阈值时，禁止后退。

只做"不许后退"，**不产生任何前进/自动倒车指令**（车尾盲区是结构性限制，
自动倒车许可需要单独验收，见 docs/hardware/rear-a22-ultrasonic-handoff.md）。

算法（依据 2026-10-31 实车标定）：
  · 每路取最近 median_window 帧的中位数（单帧噪声 ±3 cm，right_out 还会偶发误报）
  · 任一路中位数 < stop_m **且连续 confirm_frames 帧成立** → blocked
  · 该路读数过期（超过 stale_timeout）→ 视为不可信，不参与判断但计入 degraded
  · 全部探头都过期时，按 fail_safe_block 决定是否保守禁止后退

发布：
  /base/rear_blocked  (std_msgs/Bool)  true = 禁止后退
  /base/rear_status   (std_msgs/String) 便于现场诊断
"""
import statistics

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String

NAMES = ('left_out', 'left_in', 'right_in', 'right_out')


def update_hits(median_m, stop_m, resume_m, hits, confirm):
    """滞回计数：低于 stop 累加，高于 resume 清零，中间保持。

    返回 (新的 hits, 是否命中)。
    """
    if median_m < stop_m:
        hits = min(hits + 1, 10000)
    elif median_m > resume_m:
        hits = 0
    return hits, hits >= confirm


class RearGuard(Node):
    def __init__(self) -> None:
        super().__init__('rear_guard')
        self.declare_parameter('stop_m', 0.30)
        self.declare_parameter('resume_m', 0.45)      # 滞后：离开到这么远才解除
        self.declare_parameter('median_window', 5)
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('stale_timeout_s', 1.0)
        self.declare_parameter('fail_safe_block', True)
        self.declare_parameter('blocked_topic', '/base/rear_blocked')
        self.declare_parameter('status_topic', '/base/rear_status')

        self._stop = float(self.get_parameter('stop_m').value)
        self._resume = float(self.get_parameter('resume_m').value)
        self._window = max(1, int(self.get_parameter('median_window').value))
        self._confirm = max(1, int(self.get_parameter('confirm_frames').value))
        self._stale = float(self.get_parameter('stale_timeout_s').value)

        self._samples = {n: [] for n in NAMES}
        self._last = {n: 0.0 for n in NAMES}
        self._hits = {n: 0 for n in NAMES}
        self._blocked = False
        self._active = {n: False for n in NAMES}

        for name in NAMES:
            self.create_subscription(
                Range, f'/ultrasonic_rear/{name}',
                lambda msg, n=name: self._on_range(n, msg), 10)
        self._pub = self.create_publisher(
            Bool, str(self.get_parameter('blocked_topic').value), 10)
        self._status = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self.create_timer(0.05, self._tick)          # 20 Hz，跟底盘指令同频
        self.get_logger().info(
            f'后退守卫启动：stop {self._stop:.2f} m / resume {self._resume:.2f} m '
            f'/ 中位 {self._window} 帧 / 连续 {self._confirm} 帧确认')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_range(self, name: str, msg: Range) -> None:
        self._last[name] = self._now()
        buf = self._samples[name]
        buf.append(float(msg.range))
        if len(buf) > self._window:
            del buf[0]

    def _tick(self) -> None:
        now = self._now()
        blocked = False
        degraded = False
        parts = []
        for name in NAMES:
            fresh = (self._last[name] > 0.0 and now - self._last[name] <= self._stale)
            self._active[name] = fresh
            if not fresh:
                degraded = True
                parts.append(f'{name}=stale')
                continue
            med = statistics.median(self._samples[name])
            self._hits[name], hit = update_hits(
                med, self._stop, self._resume, self._hits[name], self._confirm)
            blocked = blocked or hit
            parts.append(f'{name}={med:.2f}{"!" if hit else ""}')

        if degraded and not any(self._active.values()):
            # 全部探头都过期：数据不可信
            blocked = blocked or bool(self.get_parameter('fail_safe_block').value)

        if blocked != self._blocked:
            self._blocked = blocked
            self.get_logger().warn(
                '后退被禁止（车尾近障）' if blocked else '后退已解除')
        msg = Bool()
        msg.data = self._blocked
        self._pub.publish(msg)
        status = String()
        status.data = ('blocked=' if self._blocked else 'free=') + ' '.join(parts)
        self._status.publish(status)


def main():
    rclpy.init()
    node = RearGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
