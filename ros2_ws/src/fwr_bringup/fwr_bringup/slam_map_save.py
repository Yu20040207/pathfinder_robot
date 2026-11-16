#!/usr/bin/env python3
"""Save a SLAM Toolbox map from one RViz button click.

The service does BOTH files in one call:
  * <stem>.posegraph / .data  via /slam_toolbox/serialize_map
  * <stem>.pgm / .yaml        直接由订阅到的 OccupancyGrid 写出

Saving is intentionally serialized here (one button = one snapshot), because
high-frequency serialize_map calls can crash slam_toolbox.
"""
import os
import shutil
import subprocess
import time
from pathlib import Path

import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_srvs.srv import Trigger


class SlamMapSave(Node):
    def __init__(self) -> None:
        super().__init__('slam_map_save')
        default_dir = os.environ.get(
            'FWR_MAP_DIR', os.path.expanduser('~/robot_maps'))
        self.declare_parameter('map_dir', default_dir)
        self.declare_parameter('prefix', 'real_map_manual')
        self.declare_parameter('map_topic', '/map')
        self._dir = Path(str(self.get_parameter('map_dir').value)).expanduser()
        self._prefix = str(self.get_parameter('prefix').value)
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._latest_map = None
        self._map_topic = str(self.get_parameter('map_topic').value)
        self.create_subscription(OccupancyGrid, self._map_topic,
                                 self._on_map, qos)
        self.create_service(Trigger, '/mapping/save_slam', self._save)
        self.get_logger().info(
            f'save-map service ready: /mapping/save_slam -> {self._dir}')

    def _on_map(self, msg) -> None:
        self._latest_map = msg

    def _write_map_files(self, stem: Path) -> None:
        """把最新 OccupancyGrid 写成 PGM + YAML（与 map_saver 约定一致）。"""
        msg = self._latest_map
        if msg is None:
            raise RuntimeError(f'还没有收到 {self._map_topic} 地图')
        w, h = int(msg.info.width), int(msg.info.height)
        table = bytearray(256)
        table[255] = 205          # -1 未知 → 灰
        for i in range(0, 101):
            # 0..100：0=空闲(白)，100=占用(黑)
            table[i] = int(round(254 * (1 - i / 100.0)))
        gray = bytes(v & 0xFF for v in msg.data)
        rows = [gray[i * w:(i + 1) * w] for i in range(h)]
        top_down = b''.join(reversed(rows)).translate(bytes(table))
        pgm = stem.with_suffix('.pgm')
        with open(pgm, 'wb') as fh:
            fh.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
            fh.write(top_down)
        o = msg.info.origin.position
        yaml_path = stem.with_suffix('.yaml')
        with open(yaml_path, 'w', encoding='utf-8') as fh:
            fh.write(f'image: {pgm.name}\n')
            fh.write('resolution: %.6f\n' % float(msg.info.resolution))
            fh.write('origin: [%.6f, %.6f, 0.0]\n' % (o.x, o.y))
            fh.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')

    def _save(self, request, response):
        del request
        ros2 = shutil.which('ros2')
        if ros2 is None:
            response.success = False
            response.message = '找不到 ros2 命令'
            return response

        try:
            self._dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            response.success = False
            response.message = f'无法创建地图目录 {self._dir}: {exc}'
            return response

        # 前缀优先级：/tmp/fwr_map_save_name.txt（手机端下发）> ROS 参数 > 默认
        prefix = ''
        try:
            named = Path('/tmp/fwr_map_save_name.txt')
            if named.is_file() and time.time() - named.stat().st_mtime < 30:
                prefix = named.read_text(encoding='utf-8').strip()
                named.unlink(missing_ok=True)
        except OSError:
            pass
        if not prefix:
            try:
                prefix = str(self.get_parameter('prefix').value) or 'real_map_manual'
            except Exception:                    # noqa: BLE001
                prefix = self._prefix
        # 地图名只保留安全字符（空格会破坏 launch 参数和路径）
        import re as _re
        prefix = _re.sub(r'[^0-9A-Za-z_\-]+', '_', prefix).strip('_') or 'map'
        stem = self._dir / f'{prefix}_{time.strftime("%Y%m%d_%H%M%S")}'
        n = 1
        while stem.with_suffix('.posegraph').exists() or stem.with_suffix('.yaml').exists():
            stem = self._dir / (
                f'{prefix}_{time.strftime("%Y%m%d_%H%M%S")}_{n}')
            n += 1

        # Fail fast instead of making the RViz button wait for the CLI timeout.
        try:
            services = subprocess.run(
                [ros2, 'service', 'list'], capture_output=True, text=True,
                timeout=8).stdout
        except (OSError, subprocess.TimeoutExpired) as exc:
            response.success = False
            response.message = f'无法查询 ROS 服务: {exc}'
            return response
        if '/slam_toolbox/serialize_map' not in services:
            response.success = False
            response.message = 'slam_toolbox 未运行，无法保存位姿图'
            return response

        request_text = f"{{filename: '{stem}'}}"
        try:
            pose = subprocess.run(
                [ros2, 'service', 'call', '/slam_toolbox/serialize_map',
                 'slam_toolbox/srv/SerializePoseGraph', request_text],
                capture_output=True, text=True, timeout=35)
            if not (stem.with_suffix('.posegraph').is_file() and
                    stem.with_suffix('.data').is_file()):
                detail = (pose.stderr or pose.stdout or '').strip()[-300:]
                response.success = False
                response.message = f'位姿图保存失败: {detail}'
                return response

            self._write_map_files(stem)
            if not (stem.with_suffix('.yaml').is_file() and
                    stem.with_suffix('.pgm').is_file()):
                response.success = False
                response.message = '栅格图保存失败: 未生成文件'
                return response
        except subprocess.TimeoutExpired:
            response.success = False
            response.message = '保存超时（slam_toolbox 或 map_saver 未响应）'
            return response
        except OSError as exc:
            response.success = False
            response.message = f'保存命令执行失败: {exc}'
            return response

        self.get_logger().info(f'map saved: {stem}.posegraph + .pgm/.yaml')
        response.success = True
        response.message = f'{stem}.posegraph / .pgm / .yaml'
        return response


def main():
    rclpy.init()
    node = SlamMapSave()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
