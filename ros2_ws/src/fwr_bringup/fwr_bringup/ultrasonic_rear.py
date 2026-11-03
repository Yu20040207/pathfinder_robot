#!/usr/bin/env python3
"""后向四路 A22 超声（DYP-E08 主机，RS485/Modbus RTU）→ 4 路 sensor_msgs/Range。

硬件链路（实机已验证）：
    A22 探头×4 --UART 115200--> E08 主机 --RS485--> USB 转 485 --> Jetson
    设备：/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0（ttyUSB 编号会变，必须用 by-id）

协议：Modbus RTU 9600 8N1，从机 0x01，功能码 0x03
    读四路：01 03 01 06 00 04 + CRC → 01 03 08 D1 D2 D3 D4 + CRC（单位 mm）
    异常值 0xFFFF(无应答) / 0xFFFE(同频干扰) / 0xFFFD(无目标) 一律按
    **无可靠读数**处理：不发布该路 Range，让其自然过期（绝不当 0 或"空地"）。

设计约束（来自 docs/hardware/rear-a22-ultrasonic-handoff.md）：
    · 只发布测距，不写地图空地、不清除激光障碍（配合 costmap 的
      RangeSensorLayer，clear_threshold=0 / clear_on_max_reading=false）
    · 不因为接入超声而开放自动倒车（车尾盲区是结构性限制）
    · USB 掉线自动重连（该转换器实测掉过一次）
"""
import math
import os
import select
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String

MODBUS_READ = 0x03
REG_FIRST = 0x0106          # 接口 1~4 距离（读它就是触发测距）
CRC_POLY = 0xA001
NO_DATA = {0xFFFF: 'no_response', 0xFFFE: 'interference', 0xFFFD: 'no_target'}


def crc16(data: bytes) -> bytes:
    """Modbus RTU CRC-16，低字节在前。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ CRC_POLY if crc & 1 else crc >> 1
    return bytes((crc & 0xFF, crc >> 8))


def decode_distances(resp: bytes) -> list | None:
    """解析 0x03 应答，返回 4 个原始寄存器值；无法解析返回 None。"""
    if len(resp) < 5:
        return None
    if resp[1] & 0x80:
        raise ValueError(f'Modbus 异常码 0x{resp[2]:02X}')
    if resp[1] != MODBUS_READ:
        return None
    count = resp[2] // 2
    if count < 4:
        return None
    return [(resp[3 + 2 * i] << 8) | resp[4 + 2 * i] for i in range(4)]


def range_from_raw(raw: int, min_m: float, max_m: float) -> float | None:
    """原始 mm → 米；异常码或超量程返回 None（表示无可靠读数）。"""
    if raw in NO_DATA:
        return None
    metres = raw / 1000.0
    if metres < min_m or metres > max_m:
        return None
    return metres


class SerialRtu:
    """极简 Modbus RTU 客户端（纯标准库，带重开）。"""

    def __init__(self, path: str, baud: int, timeout: float = 0.6) -> None:
        self.path = path
        self.baud = baud
        self.timeout = timeout
        self.fd = None
        self.open()

    def open(self) -> None:
        self.close()
        self.fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        attrs[4] = attrs[5] = getattr(termios, 'B%d' % self.baud)
        attrs[2] |= termios.CLOCAL | termios.CREAD
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def close(self) -> None:
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None

    def xfer(self, payload: bytes) -> bytes:
        """发一帧（自动补 CRC），返回带 CRC 的原始应答；无应答返回 b''。"""
        if self.fd is None:
            raise OSError('串口未打开')
        termios.tcflush(self.fd, termios.TCIFLUSH)
        os.write(self.fd, payload + crc16(payload))
        deadline = time.monotonic() + self.timeout
        buf = b''
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                break
            wait = min(left, 0.03) if buf else left
            if not select.select([self.fd], [], [], wait)[0]:
                if buf:
                    break
                continue
            buf += os.read(self.fd, 256)
            # 0x03 应答：addr func bytecount data crc
            if len(buf) >= 3 and buf[1] == MODBUS_READ and len(buf) >= 5 + buf[2]:
                break
        if len(buf) < 5:
            return b''
        if crc16(buf[:-2]) != buf[-2:]:
            raise ValueError(f'CRC 错: {buf.hex(" ")}')
        return buf


class UltrasonicRear(Node):
    def __init__(self) -> None:
        super().__init__('ultrasonic_rear')
        self.declare_parameter(
            'port', '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('poll_hz', 6.0)
        self.declare_parameter('frame_timeout_s', 0.6)
        # 接口 1~4 对应的 frame 与安装位置（见 docs/hardware/rear-a22-ultrasonic-layout.md）
        self.declare_parameter('frames', [
            'ultrasonic_rear_left_out',
            'ultrasonic_rear_left_in',
            'ultrasonic_rear_right_in',
            'ultrasonic_rear_right_out'])
        # 2026-10-31 实车标定（纸箱 0.20/0.30/0.50 m 三点）：
        #   left_out 系统性偏大 +26~30 mm（近距 0.2 m 时 +59 mm，因朝外偏 10°斜射）
        #   left_in / right_in 精度 ±3 mm，right_out 只读背景
        # 偏移取"远中近距离的均值"，近距残差约 ±3 cm，够停车判断用。
        self.declare_parameter('offsets_m', [0.028, 0.0, 0.0, 0.0])
        self.declare_parameter('min_range_m', 0.05)
        self.declare_parameter('max_range_m', 3.50)
        self.declare_parameter('field_of_view_rad', math.radians(60.0))
        self.declare_parameter('stale_timeout_s', 1.0)
        self.declare_parameter('reconnect_backoff_s', 1.0)
        self.declare_parameter('status_topic', '/ultrasonic_rear/status')

        self._frames = [str(x) for x in self.get_parameter('frames').value]
        if len(self._frames) != 4:
            raise ValueError('frames 必须是 4 个 frame_id')
        self._min = float(self.get_parameter('min_range_m').value)
        self._max = float(self.get_parameter('max_range_m').value)
        offs = [float(x) for x in self.get_parameter('offsets_m').value]
        if len(offs) != 4:
            raise ValueError('offsets_m 必须是 4 个值')
        self._offsets = offs
        self._fov = float(self.get_parameter('field_of_view_rad').value)
        self._stale = float(self.get_parameter('stale_timeout_s').value)

        self._pubs = [
            self.create_publisher(Range, f'/ultrasonic_rear/{"_".join(f.split("_")[2:])}', 10)
            for f in self._frames]
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)

        self._last_valid = [0.0] * 4      # 每路最近一次有效读数时刻
        self._last_code = [0] * 4         # 最近一次原始码（含异常码，便于诊断）
        self._link_ok = False
        self._next_open_at = 0.0
        self._bad_frames = 0          # 连续坏帧计数（RS485 偶发帧错）
        self.declare_parameter('max_bad_frames', 3)
        self._rtu = None

        period = 1.0 / max(0.5, float(self.get_parameter('poll_hz').value))
        self.create_timer(period, self._poll)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f'后向超声节点启动：{self.get_parameter("port").value} '
            f'@ {self.get_parameter("baud").value} 8N1，4 路 Range')

    # ---------- 串口管理 ----------
    def _ensure_open(self) -> bool:
        if self._rtu is not None:
            return True
        now = time.monotonic()
        if now < self._next_open_at:
            return False
        try:
            self._rtu = SerialRtu(
                str(self.get_parameter('port').value),
                int(self.get_parameter('baud').value),
                float(self.get_parameter('frame_timeout_s').value))
            self.get_logger().info('串口已打开')
            self._link_ok = True
            return True
        except OSError as exc:
            self._next_open_at = now + float(self.get_parameter('reconnect_backoff_s').value)
            self.get_logger().warn(f'打开串口失败（将重试）：{exc}',
                                   throttle_duration_sec=30.0)
            self._link_ok = False
            return False

    def _drop_link(self, why: str) -> None:
        if self._link_ok:
            self.get_logger().error(f'串口链路异常，关闭后重连：{why}')
        if self._rtu is not None:
            self._rtu.close()
            self._rtu = None
        self._link_ok = False
        self._next_open_at = (time.monotonic() +
                              float(self.get_parameter('reconnect_backoff_s').value))

    # ---------- 轮询 ----------
    def _read_distances(self) -> list | None:
        slave = int(self.get_parameter('slave_id').value)
        payload = bytes((slave, MODBUS_READ, REG_FIRST >> 8, REG_FIRST & 0xFF,
                         0, 4))
        resp = self._rtu.xfer(payload)
        if not resp:
            return None
        return decode_distances(resp)

    def _poll(self) -> None:
        if not self._ensure_open():
            return
        try:
            values = self._read_distances()
        except OSError as exc:                 # 串口本身坏了 → 立刻重开
            self._drop_link(str(exc))
            return
        except ValueError as exc:              # CRC 错等偶发帧错 → 少量容忍
            self._bad_frames += 1
            limit = int(self.get_parameter('max_bad_frames').value)
            if self._bad_frames >= limit:
                self._drop_link(f'连续 {self._bad_frames} 次坏帧: {exc}')
            else:
                self.get_logger().warn(
                    f'帧错（{self._bad_frames}/{limit}）：{exc}',
                    throttle_duration_sec=10.0)
            return
        if values is None:
            self._bad_frames += 1
            limit = int(self.get_parameter('max_bad_frames').value)
            if self._bad_frames >= limit:
                self._drop_link(f'连续 {self._bad_frames} 次无应答')
            return
        self._bad_frames = 0                    # 成功即清零
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        for i, raw in enumerate(values):
            self._last_code[i] = raw
            metres = range_from_raw(raw, self._min, self._max)
            if metres is None:
                continue                     # 无可靠读数：不发布，让其自然过期
            metres = max(self._min, metres - self._offsets[i])
            msg = Range()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = self._frames[i]
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = self._fov
            msg.min_range = self._min
            msg.max_range = self._max
            msg.range = metres
            self._pubs[i].publish(msg)
            self._last_valid[i] = now_s

    def _publish_status(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        parts = []
        for i, frame in enumerate(self._frames):
            age = now_s - self._last_valid[i] if self._last_valid[i] else -1.0
            fresh = age >= 0.0 and age <= self._stale
            code = NO_DATA.get(self._last_code[i], f'{self._last_code[i]}mm')
            parts.append(f'{frame}={"ok" if fresh else "stale"}'
                         f'({age:.1f}s,{code})')
        msg = String()
        msg.data = ('link=' + ('ok' if self._link_ok else 'down') + ' ' + ' '.join(parts))
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    node = UltrasonicRear()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._rtu is not None:
            node._rtu.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
