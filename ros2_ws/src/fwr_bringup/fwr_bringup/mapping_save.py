#!/usr/bin/env python3
"""「停止建图」：停车 + 结束轨迹 + 保存 pbstream 和栅格地图。

现场需求（2026-10-31）：RViz 上按下停止按钮后，当前这轮建图的成果必须
立刻存盘，不能只把车停下、地图还悬在内存里。

流程：
  1. 发布零速度，关闭底盘运动门（车立刻停）
  2. /finish_trajectory（结束当前轨迹）
  3. /write_state  → ~/.carto_maps/<name>.pbstream（可续建）
  4. cartographer_pbstream_to_ros_map → <name>.pgm/.yaml（导航用）
  5. 回一个 Trigger 响应，消息里带保存路径

暴露服务：/mapping/save     （std_srvs/Trigger）
参数：map_dir（默认 ~/.carto_maps）、name（为空则用 日期_时间）
"""
import os
import subprocess
import time

import rclpy
from cartographer_ros_msgs.srv import FinishTrajectory, WriteState
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter as RclParameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_srvs.srv import Trigger


class MappingSave(Node):
    def __init__(self) -> None:
        super().__init__('mapping_save')
        self.declare_parameter('map_dir', os.path.expanduser('~/.carto_maps'))
        self.declare_parameter('name', '')
        self.declare_parameter('convert_resolution', 0.05)
        self._dir = str(self.get_parameter('map_dir').value)
        os.makedirs(self._dir, exist_ok=True)

        self._vel = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self._param = self.create_client(SetParameters, '/base_bridge/set_parameters')
        self._finish = self.create_client(FinishTrajectory, '/finish_trajectory')
        self._write = self.create_client(WriteState, '/write_state')
        self._state = 0
        self._stem = ''
        self._finish_future = None
        self._write_future = None
        self._convert_at = 0.0
        self.create_timer(0.1, self._advance)
        self.create_service(Trigger, '/mapping/save', self._on_save)
        self.get_logger().info(f'停止建图服务已就绪：/mapping/save（目录 {self._dir}）')

    # ---------- 工具 ----------
    def _stop_robot(self) -> None:
        self._vel.publish(Twist())
        if not self._param.service_is_ready():
            return
        request = SetParameters.Request()
        for name in ('motion_enabled', 'manual_enabled'):
            p = RclParameter()
            p.name = name
            p.value.type = ParameterType.PARAMETER_BOOL
            p.value.bool_value = False
            request.parameters.append(p)
        self._param.call_async(request)

    # ---------- 保存状态机（不能在服务回调里阻塞等 future） ----------
    def _advance(self) -> None:
        state = self._state
        if state == 0:
            return
        if state == 1:
            if not self._finish.service_is_ready():
                return
            self._finish_future = self._finish.call_async(FinishTrajectory.Request())
            self._state = 2
            return
        if state == 2:
            if self._finish_future is None or not self._finish_future.done():
                return
            if not self._write.service_is_ready():
                return
            write = WriteState.Request()
            write.filename = self._stem + '.pbstream'
            self._write_future = self._write.call_async(write)
            self._state = 3
            return
        if state == 3:
            if self._write_future is None or not self._write_future.done():
                return
            ok = self._write_future.result() is not None
            self.get_logger().info(
                f'pbstream {"已保存" if ok else "保存失败"}: {self._stem}.pbstream')
            if not ok:
                self._state = 0
                return
            self._state = 4
            subprocess.Popen(
                ['ros2', 'run', 'cartographer_ros', 'cartographer_pbstream_to_ros_map',
                 '-pbstream_filename', self._stem + '.pbstream',
                 '-map_filestem', self._stem,
                 '-resolution', str(self.get_parameter('convert_resolution').value)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self._convert_at = time.time()
            return
        if state == 4:
            # 等栅格图转出来（最多 90 s）
            if os.path.exists(self._stem + '.yaml'):
                self.get_logger().info(f'✅ 建图已保存：{self._stem}.pbstream / .pgm / .yaml')
                self._state = 0
            elif time.time() - self._convert_at > 90:
                self.get_logger().warn('栅格图转换超时（pbstream 仍在）')
                self._state = 0
            return

    def _on_save(self, request, response):
        name = str(self.get_parameter('name').value).strip()
        if not name:
            name = time.strftime('map_%m%d_%H%M%S')
        self._stem = os.path.join(self._dir, name)
        self.get_logger().info(f'收到停止建图请求 → 保存 {self._stem}')
        self._stop_robot()
        self._state = 1
        self._finish_future = None
        self._write_future = None
        response.success = True
        response.message = f'已停止并开始保存：{self._stem}'
        return response


def main():
    rclpy.init()
    node = MappingSave()
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
