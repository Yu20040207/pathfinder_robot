"""Measure travelled distance and yaw from /wheel/odom, then suggest calibration.

Usage:
  1) start it, press Enter, put the robot on mark A
  2) drive straight to mark B (measure A->B with a tape), press Enter
  3) it prints odom distance; you enter the real distance to get the scale factor
"""
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomProbe(Node):
    def __init__(self):
        super().__init__('odom_calibrate')
        self.pose = None
        self.create_subscription(Odometry, '/wheel/odom', self._cb, 20)

    def _cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def wait_pose(self, timeout=10.0):
        import time
        t0 = time.time()
        while time.time() - t0 < timeout and self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose


def main():
    rclpy.init()
    node = OdomProbe()
    if node.wait_pose() is None:
        print('没有 /wheel/odom 数据（底盘桥没起？）')
        return 2
    input('把车放到标记 A，按回车记录起点...')
    rclpy.spin_once(node, timeout_sec=0.2)
    ax, ay, ayaw = node.pose
    input('直线开到标记 B 并停稳，按回车记录终点...')
    rclpy.spin_once(node, timeout_sec=0.2)
    bx, by, byaw = node.pose
    dist = math.hypot(bx - ax, by - ay)
    dyaw = math.degrees((byaw - ayaw + math.pi) % (2 * math.pi) - math.pi)
    print(f'\n里程计读数：直线 {dist:.3f} m，航向变化 {dyaw:+.2f}°')
    real = input('用卷尺量的实际距离（米，例如 3.00）：').strip()
    try:
        real = float(real)
    except ValueError:
        print('输入无效')
        return 1
    if dist > 0.1 and real > 0:
        print(f'\n距离尺度修正系数 = {real/dist:.4f}')
        print(f'  wheel_circumference_m: 0.210 → {0.210*real/dist:.4f}')
    real_yaw = input('如果做了原地转圈测试，输入实际转过角度（度，可留空）：').strip()
    if real_yaw:
        try:
            real_yaw = float(real_yaw)
        except ValueError:
            real_yaw = None
        if real_yaw and abs(dyaw) > 1.0:
            print(f'\n航向尺度修正系数 = {real_yaw/dyaw:.4f}')
            print(f'  wheel_track_m: 0.263 → {0.263*dyaw/real_yaw:.4f}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
