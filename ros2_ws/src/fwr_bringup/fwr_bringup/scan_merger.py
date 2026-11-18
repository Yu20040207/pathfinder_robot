"""两路 LaserScan → 一路 360° 合并扫描（供 slam_toolbox 使用）。

订阅 ``scan_a`` / ``scan_b``（默认两路已遮罩的 220° 扫描），用 tf2 取各自在
``target_frame``（默认 base_link）下的平面外参，合并成 720 格（0.5°/格）的一圈，
发布到 ``output_topic``（默认 /scan_merged）。

设计要点见 ``scan_merger_core``：空格填 +inf、同格取最近、用较新一路的时间戳。
两路都超过 ``stale_timeout_s`` 没数据时**停止发布**（宁可不发，也不发一圈 inf，
否则 slam_toolbox 会以为前方全空）。
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .scan_merger_core import BINS, merge_into_bins


class ScanMerger(Node):
    def __init__(self) -> None:
        super().__init__('scan_merger')
        self.declare_parameter('scan_a', '/scan_mapping')
        self.declare_parameter('scan_b', '/scan_rear_mapping')
        self.declare_parameter('output_topic', '/scan_merged')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('bins', BINS)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('stale_timeout_s', 0.6)
        self.declare_parameter('publish_rate_hz', 10.0)
        # 2026-11-18：默认等**两路都就绪**再开始发布。
        # 原因：slam_toolbox 的 minimum_travel_distance=0.5，车不动时只处理第一帧；
        # 若第一帧只有前雷达（后雷达还在起转），整个地图就永远缺后半圈。
        self.declare_parameter('require_both', True)

        self._target = str(self.get_parameter('target_frame').value)
        self._bins = int(self.get_parameter('bins').value)
        self._rmin = float(self.get_parameter('range_min').value)
        self._rmax = float(self.get_parameter('range_max').value)
        self._stale = float(self.get_parameter('stale_timeout_s').value)

        self._latest: dict[str, LaserScan] = {}
        self._at: dict[str, float] = {}

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        outs = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        for slot, param in (('a', 'scan_a'), ('b', 'scan_b')):
            topic = str(self.get_parameter(param).value)
            self.create_subscription(
                LaserScan, topic, lambda m, s=slot: self._on_scan(s, m), qos)
        out_topic = str(self.get_parameter('output_topic').value)
        self._pub = self.create_publisher(LaserScan, out_topic, outs)

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'scan_merger: {self.get_parameter("scan_a").value} + '
            f'{self.get_parameter("scan_b").value} → {out_topic} '
            f'({self._bins} 格, 目标系 {self._target})')

    # ---------- 回调 ----------
    def _on_scan(self, slot: str, msg: LaserScan) -> None:
        self._latest[slot] = msg
        self._at[slot] = time.monotonic()

    # ---------- 外参 ----------
    def _planar(self, frame: str) -> tuple[float, float, float] | None:
        """取 frame→target 的平面外参 (yaw, tx, ty)；失败返回 None。"""
        if frame == self._target:
            return 0.0, 0.0, 0.0
        try:
            tr = self._tf.lookup_transform(self._target, frame, Time())
        except TransformException:
            return None
        t = tr.transform.translation
        q = tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return yaw, t.x, t.y

    # ---------- 发布 ----------
    def _tick(self) -> None:
        now = time.monotonic()
        fresh = [s for s in ('a', 'b')
                 if s in self._latest and now - self._at.get(s, 0.0) <= self._stale]
        if not fresh:
            return                       # 两路都没数据：不发（避免"全空"误导）
        if bool(self.get_parameter('require_both').value) and len(fresh) < 2:
            # 只有一路就绪时先不发布：slam_toolbox 可能只处理第一帧，
            # 那第一帧必须已经是完整 360°，否则地图会永久缺半圈。
            if not getattr(self, '_warned_wait', False):
                self._warned_wait = True
                self.get_logger().info(
                    f'等待第二路雷达就绪（已有 {fresh}），暂不发布合并扫描')
            return
        pieces = []
        stamps = []
        for slot in fresh:
            msg = self._latest[slot]
            ext = self._planar(msg.header.frame_id)
            if ext is None:
                continue                 # 外参还没到（tf 未就绪）
            yaw, tx, ty = ext
            pieces.append((msg.ranges, msg.angle_min, msg.angle_increment,
                           yaw, tx, ty))
            stamps.append(Time.from_msg(msg.header.stamp))
        if not pieces:
            return

        merged = merge_into_bins(pieces, self._bins, self._rmin, self._rmax)
        out = LaserScan()
        out.header.stamp = max(stamps).to_msg()      # 用较新一路的时间戳
        out.header.frame_id = self._target
        out.angle_min = -math.pi
        out.angle_max = math.pi - 2.0 * math.pi / self._bins
        out.angle_increment = 2.0 * math.pi / self._bins
        out.range_min = self._rmin
        out.range_max = self._rmax
        out.ranges = merged
        out.intensities = []
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
