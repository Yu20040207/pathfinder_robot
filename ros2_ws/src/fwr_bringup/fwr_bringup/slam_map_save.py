#!/usr/bin/env python3
"""Save a SLAM Toolbox map from one RViz button click.

The service does BOTH files in one call:
  * <stem>.posegraph / .data  via /slam_toolbox/serialize_map
  * <stem>.pgm / .yaml        via nav2_map_server/map_saver_cli

Saving is intentionally serialized here (one button = one snapshot), because
high-frequency serialize_map calls can crash slam_toolbox.
"""
import os
import shutil
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class SlamMapSave(Node):
    def __init__(self) -> None:
        super().__init__('slam_map_save')
        default_dir = os.environ.get(
            'FWR_MAP_DIR', os.path.expanduser('~/robot_maps'))
        self.declare_parameter('map_dir', default_dir)
        self.declare_parameter('prefix', 'real_map_manual')
        self._dir = Path(str(self.get_parameter('map_dir').value)).expanduser()
        self._prefix = str(self.get_parameter('prefix').value)
        self.create_service(Trigger, '/mapping/save_slam', self._save)
        self.get_logger().info(
            f'save-map service ready: /mapping/save_slam -> {self._dir}')

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

        stem = self._dir / f'{self._prefix}_{time.strftime("%Y%m%d_%H%M%S")}'
        n = 1
        while stem.with_suffix('.posegraph').exists() or stem.with_suffix('.yaml').exists():
            stem = self._dir / (
                f'{self._prefix}_{time.strftime("%Y%m%d_%H%M%S")}_{n}')
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

            grid = subprocess.run(
                [ros2, 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', str(stem)],
                capture_output=True, text=True, timeout=35)
            if not (stem.with_suffix('.yaml').is_file() and
                    stem.with_suffix('.pgm').is_file()):
                detail = (grid.stderr or grid.stdout or '').strip()[-300:]
                response.success = False
                response.message = f'栅格图保存失败: {detail}'
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
