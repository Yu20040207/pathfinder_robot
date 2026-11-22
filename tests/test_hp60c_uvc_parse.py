"""HP60C UVC 拆包逻辑的回归测试（不连相机、不需要 ROS）。

钉住两件事：
1. 一帧 = JPEG + 640x480x2 深度 + 512 补齐，必须**按整帧长度消费**；
2. 深度区里出现的 0xFFD8 不能被当成下一帧的 JPEG 头（实测会随机出现）。
"""

import io
import struct

import numpy as np

from fwr_bringup.hp60c_uvc import (DEPTH_BYTES, DEPTH_H, DEPTH_W, HEADER_BYTES,
                                   TAIL_BYTES, decode_depth_mm, iter_frames)

FAKE_JPEG = b'\xff\xd8' + b'fakejpeg' + b'\xff\xd9'


def _depth_payload(value: int = 12345, inject_soi: bool = False) -> bytes:
    arr = np.full(DEPTH_W * DEPTH_H, value, dtype='<u2')
    raw = bytearray(arr.tobytes())
    if inject_soi:                      # 深度里埋一个假 SOI，检验不会被误切
        raw[100:102] = b'\xff\xd8'
    return bytes(raw)


def _frame(jpeg: bytes = FAKE_JPEG, depth: bytes = None) -> bytes:
    depth = _depth_payload() if depth is None else depth
    return (b'\x00' * HEADER_BYTES + jpeg + depth + b'\x00' * TAIL_BYTES)


def test_single_frame_split():
    frames = list(iter_frames(io.BytesIO(_frame())))
    assert len(frames) == 1
    jpeg, depth = frames[0]
    assert jpeg == FAKE_JPEG
    assert len(depth) == DEPTH_BYTES


def test_multiple_frames_in_order():
    data = _frame() + b'\x00' * 64 + _frame()   # 帧间允许有垃圾字节
    frames = list(iter_frames(io.BytesIO(data)))
    assert len(frames) == 2
    assert all(jpeg == FAKE_JPEG for jpeg, _ in frames)


def test_partial_frame_is_not_emitted():
    full = _frame()
    # 截掉尾部 200 KB：深度没收全，不能吐帧（否则下一帧起点会错位）
    frames = list(iter_frames(io.BytesIO(full[:len(full) - 200_000])))
    assert frames == []


def test_soi_inside_depth_does_not_split():
    data = _frame(depth=_depth_payload(inject_soi=True))
    frames = list(iter_frames(io.BytesIO(data)))
    assert len(frames) == 1, '深度里的 0xFFD8 被误判成新帧'
    assert frames[0][0] == FAKE_JPEG


def test_depth_decodes_to_millimetres():
    depth = decode_depth_mm(_depth_payload(value=14041))
    assert depth is not None
    assert depth.shape == (DEPTH_H, DEPTH_W)
    assert depth.dtype == np.uint16
    assert int(depth[0, 0]) == 1404          # 0.1 mm -> mm


def test_zero_depth_stays_zero():
    depth = decode_depth_mm(_depth_payload(value=0))
    assert int(depth.max()) == 0


def test_short_depth_returns_none():
    assert decode_depth_mm(b'\x00' * 100) is None


def test_chunked_read_splitting_across_reads():
    """按 7 字节小块喂，检验跨块拼接（v4l2 管道就是这种形态）。"""

    class TinyChunks(io.BytesIO):
        def read(self, n=-1):
            return super().read(min(n, 7))

    frames = list(iter_frames(TinyChunks(_frame()), chunk_size=7))
    assert len(frames) == 1
    assert frames[0][0] == FAKE_JPEG


def test_frame_size_matches_measured_layout():
    """实测整帧约 630 KB；用假 JPEG 时也要能推回结构常量。"""
    assert HEADER_BYTES == 2560
    assert TAIL_BYTES == 512
    assert DEPTH_BYTES == 640 * 480 * 2
    assert struct.calcsize('<H') == 2


def test_on_jpeg_fires_once_per_frame():
    """on_jpeg 是按"帧"回调：同一帧只回调一次（曾经因为重复扫描 SOI 调了两次）。"""
    seen = []
    frames = list(iter_frames(io.BytesIO(_frame() + _frame()), on_jpeg=seen.append))
    assert len(seen) == 2
    assert len(frames) == 2
    assert all(f[0] == FAKE_JPEG for f in frames)


def test_on_jpeg_fires_before_depth_complete():
    """JPEG 一收全就该回调，不等后面 614 KB 深度（这就是延迟优化的关键）。"""
    frame = _frame()
    jpeg_end = HEADER_BYTES + len(FAKE_JPEG)
    seen = []

    class Truncated(io.BytesIO):
        def read(self, n=-1):
            return super().read(min(n, jpeg_end))    # 只给到 JPEG 结尾

    gen = iter_frames(Truncated(frame), on_jpeg=seen.append)
    try:
        next(gen)
    except StopIteration:
        pass
    assert len(seen) == 1, 'JPEG 完整后可回调，不必等深度'
    assert seen[0] == FAKE_JPEG


def test_need_depth_false_skips_depth_copy():
    """没人订阅深度时不该产出深度（省一次 614 KB 拷贝）。"""
    frames = list(iter_frames(io.BytesIO(_frame() + _frame()),
                              need_depth=lambda: False))
    assert len(frames) == 2
    assert all(depth == b'' for _, depth in frames)


def test_need_depth_true_keeps_depth():
    frames = list(iter_frames(io.BytesIO(_frame()), need_depth=lambda: True))
    assert len(frames) == 1
    assert len(frames[0][1]) == DEPTH_BYTES
