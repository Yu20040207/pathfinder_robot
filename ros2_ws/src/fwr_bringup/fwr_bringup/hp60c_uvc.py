"""HP60C UVC 裸流解析（纯 Python + numpy，不依赖 ROS）。

为什么需要：厂商 `ascamera` SDK 在本机（2026-11-21 实测）能打开相机、报
`start streaming`，但**恰好 1.003 s 后自己 `stopStreaming`**，之后零帧；
逻辑在闭源 `libAngstrongCameraSdk.so` 里，只能绕开，直接读 UVC 自己拆包。

一帧的真实结构（UVC "MJPG" 640x642，逐字节解析得出）：

    [2560 B 厂商头][JPEG 640x480 RGB][640x480x2 B 深度][512 B 补齐]   ≈ 630 KB

- 深度是 `<u2` 小端，**单位 0.1 mm**（实测中位 14041 → 1.40 m）
- 深度区会随机出现 ``0xFFD8``，所以**必须按整帧长度消费**，
  不能只找 JPEG 边界，否则会误判下一帧起点

对外只暴露三个东西：
    ``iter_frames(stream)``  从任意可读流里拆出 ``(jpeg_bytes, depth_bytes)``
    ``decode_rgb(jpeg)``     -> HxWx3 BGR
    ``decode_depth_mm(raw)`` -> HxW uint16（毫米）
    ``Hp60cUvcStream``       用 ``v4l2-ctl`` 拉流、断流自动重试
"""

from __future__ import annotations

import os
import subprocess
import threading
import time
from typing import Iterator, Optional, Tuple

import cv2
import numpy as np

HEADER_BYTES = 2560          # 每帧开头的厂商头（时间戳/标定等，我们用不到）
DEPTH_W = 640
DEPTH_H = 480
DEPTH_BYTES = DEPTH_W * DEPTH_H * 2
TAIL_BYTES = 512             # 帧尾补齐
DEFAULT_SIZE = '640x642'     # UVC 报的尺寸；画面仍是 640x480
DEFAULT_DEVICE = '/dev/video0'


def decode_rgb(jpeg: bytes) -> Optional[np.ndarray]:
    """JPEG -> BGR；坏帧返回 None。"""
    if not jpeg:
        return None
    arr = np.frombuffer(jpeg, np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


def decode_depth_mm(raw: bytes) -> Optional[np.ndarray]:
    """16 位原始深度（0.1 mm）-> uint16 毫米图。

    0 表示无效点，保持不变（下游按 0 处理）。
    """
    if len(raw) < DEPTH_BYTES:
        return None
    arr = np.frombuffer(raw, '<u2')[:DEPTH_W * DEPTH_H]
    return (arr // 10).astype(np.uint16).reshape(DEPTH_H, DEPTH_W)


def iter_frames(stream, stop: Optional[threading.Event] = None,
                chunk_size: int = 262144,
                max_buffer: int = 8_000_000,
                on_jpeg=None, need_depth=None) -> Iterator[Tuple[bytes, bytes]]:
    """从字节流里持续拆出完整帧，产出 ``(jpeg, depth_raw)``。

    ``stream`` 只要有 ``read(n)`` 即可（文件、BytesIO、子进程 stdout 都行），
    所以这个函数可以脱机单测。

    **低延迟**：JPEG 的 ``FFD9`` 一出现就回调 ``on_jpeg``（不等 614 KB 深度）。
    **低 CPU**：``need_depth()`` 返回 False 时，深度字节直接丢掉、不做 614 KB 拷贝
    （手机推流根本用不到深度，之前这一份拷贝吃掉几十 % 的 CPU）。
    """
    buf = bytearray()
    pending: Optional[bytes] = None      # 已回调过、还在等深度的那一帧 JPEG
    tail = DEPTH_BYTES + TAIL_BYTES
    want_depth = True

    while stop is None or not stop.is_set():
        try:
            chunk = stream.read(chunk_size)
        except (ValueError, OSError):
            break
        if not chunk:
            break
        buf += chunk
        while True:
            if pending is not None:                     # 处理这一帧的深度/补齐
                if len(buf) < tail:
                    break
                if want_depth:
                    depth = bytes(buf[:DEPTH_BYTES])
                else:
                    depth = b''
                del buf[:tail]
                jpeg, pending = pending, None
                yield jpeg, depth
                continue

            soi = buf.find(b'\xff\xd8')
            if soi < 0:
                if len(buf) > max_buffer:               # 全是垃圾，丢掉大半防积压
                    del buf[:len(buf) // 2]
                break
            eoi = buf.find(b'\xff\xd9', soi)
            if eoi < 0:
                break
            jpeg = bytes(buf[soi:eoi + 2])
            del buf[:eoi + 2]                            # 只消费到 JPEG 尾
            if on_jpeg is not None:
                on_jpeg(jpeg)                            # 立刻给出去，不等深度
            pending = jpeg
            if need_depth is not None:
                try:
                    want_depth = bool(need_depth())
                except Exception:                        # noqa: BLE001
                    want_depth = True


class Hp60cUvcStream:
    """用 ``v4l2-ctl`` 拉 HP60C 裸流，断流/掉线自动重试。"""

    def __init__(self, device: str = DEFAULT_DEVICE, size: str = DEFAULT_SIZE,
                 restart_delay: float = 3.0, fps: int = 0) -> None:
        self.device = device
        self.size = size
        #: 设备出帧上限（0=不设）。相机默认 30 fps，一帧 630 KB → 19 MB/s，
        #: Python 侧解析吃满一个核还追不上（实测只出 11 fps、单核 94%）。
        self.fps = int(fps)
        self.restart_delay = float(restart_delay)
        self._stop = threading.Event()
        #: 每次重试前回调（日志用）
        self.on_retry = None

    def command(self) -> list:
        w, h = self.size.lower().split('x')
        cmd = ['v4l2-ctl', '-d', self.device,
               '--set-fmt-video=width=%s,height=%s,pixelformat=MJPG' % (w, h)]
        if self.fps > 0:
            cmd.append('--set-parm=%d' % self.fps)
        cmd += ['--stream-mmap', '--stream-count=0', '--stream-to=-']
        return cmd

    def stop(self) -> None:
        self._stop.set()

    def frames(self, on_jpeg=None, need_depth=None) -> Iterator[Tuple[bytes, bytes]]:
        while not self._stop.is_set():
            if not os.path.exists(self.device):
                if self.on_retry:
                    self.on_retry('等待相机 %s 出现…' % self.device)
                self._stop.wait(2.0)
                continue
            proc = None
            try:
                proc = subprocess.Popen(self.command(), stdout=subprocess.PIPE,
                                        stderr=subprocess.DEVNULL, bufsize=0)
                for frame in iter_frames(proc.stdout, self._stop,
                                         on_jpeg=on_jpeg,
                                         need_depth=need_depth):
                    yield frame
            except (OSError, ValueError) as exc:
                if self.on_retry:
                    self.on_retry('UVC 异常: %s' % exc)
            finally:
                if proc is not None and proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.kill()
            if not self._stop.is_set():
                if self.on_retry:
                    self.on_retry('UVC 断流，%.0f s 后重试' % self.restart_delay)
                self._stop.wait(self.restart_delay)


def open_device(device: str) -> bool:
    """设备在不在（启动前自检用）。"""
    return os.path.exists(device)


__all__ = ['iter_frames', 'decode_rgb', 'decode_depth_mm', 'Hp60cUvcStream',
           'open_device', 'DEFAULT_DEVICE', 'DEFAULT_SIZE',
           'DEPTH_W', 'DEPTH_H', 'DEPTH_BYTES']
