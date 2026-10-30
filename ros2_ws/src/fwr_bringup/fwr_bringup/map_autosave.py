"""Periodically persist the map while driving.

支持两种建图引擎（2026-10-30 增加 Cartographer）：
  engine=slam  → slam_toolbox 的 /serialize_map + /save_map
  engine=carto → Cartographer 的 /write_state（写 pbstream），完成时再存栅格图
"""
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from cartographer_ros_msgs.srv import WriteState


class MapAutosave(Node):
    def __init__(self):
        super().__init__('map_autosave')
        self.declare_parameter('map_name', '')
        self.declare_parameter('period_s', 120.0)
        self.declare_parameter('engine', 'slam')          # slam | carto
        self._engine = str(self.get_parameter('engine').value)
        self._name = str(self.get_parameter('map_name').value)
        if self._engine == 'carto':
            self._init_carto()
            period = max(60.0, float(self.get_parameter('period_s').value))
            if not self._name:
                self.get_logger().warn('map_name is empty; autosave disabled')
                return
            self.create_timer(period, self._tick_carto)
            self.get_logger().info(
                f'autosaving Cartographer state to {self._name}.pbstream every '
                f'{period:.0f} s')
            return
        self._save = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self._serialize = self.create_client(SerializePoseGraph,
                                             '/slam_toolbox/serialize_map')
        period = max(30.0, float(self.get_parameter('period_s').value))
        if not self._name:
            self.get_logger().warn('map_name is empty; autosave disabled')
            return
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'autosaving map to {self._name} every {period:.0f} s (slam_toolbox)')

    def _init_carto(self):
        self._write = self.create_client(WriteState, '/write_state')

    def _tick_carto(self):
        if not self._write.service_is_ready():
            self.get_logger().warn('/write_state 不可用（Cartographer 没起来？）',
                                   throttle_duration_sec=60.0)
            return
        path = os.path.expanduser(f'~/.carto_maps/{self._name}.pbstream')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        request = WriteState.Request()
        request.filename = path
        self._write.call_async(request)
        self.get_logger().info(f'Cartographer state 已写入 {path}')
        # 顺带导出一张栅格图（异步，失败不影响建图）
        stem = os.path.expanduser(f'~/.carto_maps/{self._name}')
        try:
            subprocess.Popen(
                ['ros2', 'run', 'cartographer_ros', 'cartographer_pbstream_to_ros_map',
                 '-pbstream_filename', path, '-map_filestem', stem, '-resolution', '0.05'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as exc:
            self.get_logger().warn(f'导出栅格图失败: {exc}')

    def _tick(self):
        if not self._save.service_is_ready() or not self._serialize.service_is_ready():
            self.get_logger().warn('slam_toolbox save services not ready yet',
                                   throttle_duration_sec=60.0)
            return
        request = SerializePoseGraph.Request()
        request.filename = self._name
        self._serialize.call_async(request)
        save = SaveMap.Request()
        save.name = self._name
        self._save.call_async(save)
        self.get_logger().info('map autosaved (image + posegraph)')


def main():
    rclpy.init()
    node = MapAutosave()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
