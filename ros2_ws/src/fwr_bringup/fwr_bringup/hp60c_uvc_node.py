"""HP60C 相机 → ROS 2 图像话题（直接读 UVC，绕开厂商 SDK）。

发布的都是项目约定的绝对话题名，`fwr_vision` / `depthimage_to_laserscan` 直接可用：

    /robot/camera/color/image_raw     sensor_msgs/Image  bgr8   640x480
    /robot/camera/color/image_raw/compressed  sensor_msgs/CompressedImage (jpeg)
    /robot/camera/color/camera_info   sensor_msgs/CameraInfo
    /robot/camera/depth/image_raw     sensor_msgs/Image  16UC1（毫米，0=无效）
    /robot/camera/depth/camera_info   sensor_msgs/CameraInfo

用法：
    source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
    ros2 run fwr_bringup hp60c_uvc_node

参数（`ros2 param` 可改）：
    device            /dev/video0
    uvc_size          640x642        # UVC 帧尺寸，不是画面尺寸
    publish_depth     true
    publish_info      true
    frame_id_color    camera_color_optical_frame
    frame_id_depth    camera_depth_optical_frame
    fx fy cx cy       554 554 319.5 239.5   # 未标定，按 60° HFOV 估的占位值
    mirror            true           # 本机 HP60C 的原始 UVC 画面是**左右镜像**的
                                     # （用户 2026-11-21 用手机相机对照确认），
                                     # 这里统一翻正：彩色图 + 深度图一起翻，保证对齐

⚠️ 与厂商 `ascamera_node` **不能同时跑**（同一个 /dev/video0）；也不要用 `kill -9`
   杀厂商节点，会把 /dev/video0 弄没（用 /sys/bus/usb/devices/<port>/authorized 0→1 复位）。
"""

from __future__ import annotations

import time

import cv2
import numpy as np

import rclpy
import std_msgs.msg
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from fwr_bringup.hp60c_uvc import (DEFAULT_DEVICE, DEFAULT_SIZE, DEPTH_H, DEPTH_W,
                                   Hp60cUvcStream, decode_depth_mm, decode_rgb)


def _encode_jpeg(bgr) -> bytes:
    """把翻正后的 BGR 图重新编码成 JPEG（只在本机镜像时需要，~5 ms/帧）。"""
    if bgr is None:
        return b''
    ok, buf = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    return buf.tobytes() if ok else b''


class Hp60cUvcNode(Node):
    def __init__(self) -> None:
        super().__init__('hp60c_uvc')
        p = self.declare_parameter
        self.device = p('device', DEFAULT_DEVICE).value
        self.uvc_size = p('uvc_size', DEFAULT_SIZE).value
        # 0 = 不设（实测本机驱动忽略 --set-parm，永远按 30 fps 给帧；CPU 瓶颈不在
        # 设备帧率，而在"别让节点解码原始图"，见 robot_mjpeg_server 的懒订阅）
        self.device_fps = int(p('device_fps', 0).value)
        self.publish_depth = bool(p('publish_depth', True).value)
        self.publish_info = bool(p('publish_info', True).value)
        # 2026-11-21：本机 HP60C 裸流是左右镜像（厂商 SDK 里应该是翻过的），
        # 用户手机对照确认。彩色 JPEG 与深度必须**同时**翻，否则两者错位。
        self.mirror = bool(p('mirror', True).value)
        self.frame_color = p('frame_id_color', 'camera_color_optical_frame').value
        self.frame_depth = p('frame_id_depth', 'camera_depth_optical_frame').value
        self.fx = float(p('fx', 554.0).value)
        self.fy = float(p('fy', 554.0).value)
        self.cx = float(p('cx', 319.5).value)
        self.cy = float(p('cy', 239.5).value)

        # 传感器数据用 best-effort：和项目里其它图像订阅端（qos_profile_sensor_data）兼容
        qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_color = self.create_publisher(Image, '/robot/camera/color/image_raw', qos)
        # 相机本身就是 JPEG，直接转发省一次解码（手机推流走这一路）
        self.pub_jpeg = self.create_publisher(
            CompressedImage, '/robot/camera/color/image_raw/compressed', qos)
        self.pub_depth = self.create_publisher(Image, '/robot/camera/depth/image_raw', qos)
        if self.publish_info:
            info_qos = QoSProfile(depth=2, history=HistoryPolicy.KEEP_LAST,
                                  reliability=ReliabilityPolicy.RELIABLE)
            self.pub_color_info = self.create_publisher(
                CameraInfo, '/robot/camera/color/camera_info', info_qos)
            self.pub_depth_info = self.create_publisher(
                CameraInfo, '/robot/camera/depth/camera_info', info_qos)
        else:
            self.pub_color_info = None
            self.pub_depth_info = None

        self._frames = 0
        self._bad = 0
        self._t_last = time.monotonic()
        self._stats_t = time.monotonic()
        self.stream = Hp60cUvcStream(self.device, self.uvc_size,
                                     fps=self.device_fps)
        self.stream.on_retry = lambda msg: self.get_logger().warn(msg)
        self.get_logger().info(
            'HP60C UVC: %s (%s, %s fps) → /robot/camera/{color,depth}/image_raw'
            % (self.device, self.uvc_size, self.device_fps or '默认'))

    # ------------------------------------------------------------ 发布
    def _stamp(self, header: std_msgs.msg.Header, frame_id: str) -> None:
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

    def _make_image(self, img, frame_id: str, encoding: str) -> Image:
        msg = Image()
        self._stamp(msg.header, frame_id)
        msg.height, msg.width = int(img.shape[0]), int(img.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = int(img.strides[0])
        msg.data = img.tobytes()
        return msg

    def _make_info(self, frame_id: str, width: int, height: int) -> CameraInfo:
        info = CameraInfo()
        self._stamp(info.header, frame_id)
        info.width, info.height = int(width), int(height)
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def on_jpeg(self, jpeg: bytes) -> None:
        """JPEG 一收全就发（不等深度，省 100 ms 级延迟）。"""
        want_raw = self.pub_color.get_subscription_count() > 0
        rgb = None
        if self.mirror or want_raw:
            rgb = decode_rgb(jpeg)
            if rgb is not None and self.mirror:
                rgb = cv2.flip(rgb, 1)
        if self.mirror:
            jpeg = _encode_jpeg(rgb) or jpeg       # 压缩流也必须是翻正的画面
        if self.pub_jpeg is not None:
            comp = CompressedImage()
            self._stamp(comp.header, self.frame_color)
            comp.format = 'jpeg'
            comp.data = jpeg
            self.pub_jpeg.publish(comp)

        # 原始图 921 KB/帧：只有真有人订阅才发（手机走 compressed，不受影响）
        if want_raw:
            if rgb is not None:
                self.pub_color.publish(self._make_image(rgb, self.frame_color, 'bgr8'))
            else:
                self._bad += 1
        if self.pub_color_info is not None:
            h, w = (rgb.shape[0], rgb.shape[1]) if rgb is not None else (480, 640)
            self.pub_color_info.publish(self._make_info(self.frame_color, w, h))
        self._frames += 1

    def _need_depth(self) -> bool:
        """有订阅者才要深度（否则解析器直接丢字节，省 614 KB/帧 的拷贝）。"""
        return bool(self.publish_depth and self.pub_depth is not None
                    and self.pub_depth.get_subscription_count() > 0)

    def publish_depth_image(self, depth_raw: bytes) -> None:
        """深度 614 KB/帧：没人订阅就只统计、不解码不发。"""
        if not self.publish_depth or self.pub_depth is None:
            return
        if self.pub_depth.get_subscription_count() == 0:
            return
        depth = decode_depth_mm(depth_raw)
        if depth is None:
            return
        if self.mirror:
            depth = np.ascontiguousarray(depth[:, ::-1])   # 与彩色图同时翻，保持对齐
        self.pub_depth.publish(self._make_image(depth, self.frame_depth, '16UC1'))
        if self.pub_depth_info is not None:
            self.pub_depth_info.publish(self._make_info(self.frame_depth, DEPTH_W, DEPTH_H))

    # ------------------------------------------------------------ 主循环
    def run(self) -> None:
        for _jpeg, depth in self.stream.frames(on_jpeg=self.on_jpeg,
                                               need_depth=self._need_depth):
            if not rclpy.ok():
                break
            self.publish_depth_image(depth)
            now = time.monotonic()
            if now - self._stats_t >= 5.0:
                fps = self._frames / max(1e-3, now - self._t_last)
                self.get_logger().info(
                    '相机 %.1f fps，累计 %d 帧，坏帧 %d'
                    % (fps, self._frames, self._bad))
                self._frames = 0
                self._bad = 0
                self._t_last = now
                self._stats_t = now
        self.stream.stop()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = Hp60cUvcNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stream.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
