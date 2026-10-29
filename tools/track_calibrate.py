#!/usr/bin/env python3
"""轮距标定：原地慢转，比较「编码器算出的转角」和「IMU 实测转角」。

真实轮距 = 名义轮距 × (编码器转角 / IMU 转角)
用法： .venv/bin/python tools/track_calibrate.py [每次转多少度] [角速度]
注意：需要在 map/odom 正常、manual_enabled=true 时运行；车前后左右留 0.5 m。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def current_track():
    """读取 base_bridge 里当前实际生效的轮距（避免基准漂移）。"""
    try:
        from fwr_bringup.base_bridge_core import BaseBridgeCore
        return BaseBridgeCore().track_m
    except Exception:  # noqa: BLE001
        return 0.263


NOMINAL_TRACK = current_track()


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Calibrator(Node):
    def __init__(self):
        super().__init__('track_calibrate')
        self.pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self.wheel_yaw = []
        self.imu_yaw = []
        self.create_subscription(Odometry, '/wheel/odom',
                                 lambda m: self.wheel_yaw.append(yaw_of(m.pose.pose.orientation)), 20)
        self.create_subscription(Odometry, '/odom',
                                 lambda m: self.imu_yaw.append(yaw_of(m.pose.pose.orientation)), 20)

    def spin_for(self, deg, rate):
        turns = abs(deg) / 360.0
        dur = turns * 2 * math.pi / rate
        cmd = Twist()
        cmd.angular.z = rate if deg > 0 else -rate
        w0 = self.wheel_yaw[-1] if self.wheel_yaw else 0.0
        i0 = self.imu_yaw[-1] if self.imu_yaw else 0.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.04)
        self.pub.publish(Twist())
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.01)
        w1 = self.wheel_yaw[-1] if self.wheel_yaw else 0.0
        i1 = self.imu_yaw[-1] if self.imu_yaw else 0.0

        def unwrap(a, b):
            d = b - a
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            return d

        return unwrap(w0, w1), unwrap(i0, i1)


def main():
    deg = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
    rate = float(sys.argv[2]) if len(sys.argv) > 2 else 0.35
    rclpy.init()
    node = Calibrator()
    t = time.time()
    while time.time() - t < 3 and (not node.wheel_yaw or not node.imu_yaw):
        rclpy.spin_once(node, timeout_sec=0.1)
    try:
        w_a, i_a = node.spin_for(deg, rate)
        print(f'正向 {deg:+.0f}°: 编码器 {math.degrees(w_a):+.1f}°  IMU {math.degrees(i_a):+.1f}°')
        time.sleep(1.0)
        w_b, i_b = node.spin_for(-deg, rate)
        print(f'反向 {-deg:+.0f}°: 编码器 {math.degrees(w_b):+.1f}°  IMU {math.degrees(i_b):+.1f}°')
        enc_f, imu_f = abs(math.degrees(w_a)), abs(math.degrees(i_a))
        enc_b, imu_b = abs(math.degrees(w_b)), abs(math.degrees(i_b))
        if imu_f < 20 or imu_b < 20:
            print('IMU 转角太小，标定不可靠，请重试')
            return
        r_f, r_b = enc_f / imu_f, enc_b / imu_b
        print(f'  正向比值 {r_f:.4f}   反向比值 {r_b:.4f}   差异 {abs(r_f - r_b) * 100:.2f}%')
        ratio = (r_f + r_b) / 2.0
        enc, imu = enc_f + enc_b, imu_f + imu_b
        print()
        print(f'合计: 编码器 {enc:+.1f}°   IMU {imu:+.1f}°   比值 {ratio:.4f}')
        print(f'当前生效轮距 {NOMINAL_TRACK:.4f} m，实测比值 {ratio:.4f}')
        print(f'建议轮距 = {NOMINAL_TRACK:.4f} × {ratio:.4f} = {NOMINAL_TRACK * ratio:.4f} m')
        if abs(ratio - 1.0) < 0.02:
            print('残差 <2%：已标定到位，剩余偏差是转向打滑（任何轮距值都修不掉）')
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
