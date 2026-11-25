"""第二路相机（120° 广角 5MP）取流与路由的回归测试（2026-11-25）。

背景：用户新插了一个 Xitech 5MP 120° 广角摄像头（`0abd:8050`），要求手机端
在 HP60C 画面右上角加一个切换键，点一下看广角。

车端实现（`tools/robot_mjpeg_server.py`）：
* `WideUvcThread` 用 `v4l2-ctl` 拉 MJPEG 裸流 —— **零转码**，但裸流只有负载、
  没有帧长，所以按 JPEG 标记 `FFD8..FFD9` 切帧（`split_jpeg_frames`）；
* 设备路径按 `/dev/v4l/by-id/*5MP_USB_Camera*-video-index0` 找，**不写死 /dev/video2**；
* `/video?cam=wide` 走第二路，其余仍走主摄（HP60C 的 ROS compressed）。

服务端模块在测试环境不一定装得上 aiohttp/rclpy，所以用 AST 直接取纯函数。
"""

from __future__ import annotations

import ast
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
SERVER = ROOT / 'tools' / 'robot_mjpeg_server.py'

JPEG_A = b'\xff\xd8' + b'A' * 40 + b'\xff\xd9'
JPEG_B = b'\xff\xd8' + b'B' * 100 + b'\xff\xd9'
JPEG_C = b'\xff\xd8' + b'C' * 7 + b'\xff\xd9'


def _load_functions(*names: str):
    """从服务端源码里只取指定的纯函数（不 import 整个模块）。"""
    tree = ast.parse(SERVER.read_text(encoding='utf-8'))
    keep = [n for n in tree.body
            if isinstance(n, ast.Assign)]
    keep += [n for n in tree.body
             if isinstance(n, ast.FunctionDef) and n.name in names]
    assert len([n for n in keep if isinstance(n, ast.FunctionDef)]) == len(names), \
        f'没找到函数：{names}'
    ns: dict = {'os': __import__('os'), 'bytes': bytes}
    exec(compile(ast.Module(body=keep, type_ignores=[]), '<server>', 'exec'), ns)
    return [ns[n] for n in names]


split_jpeg_frames, wide_device_path = _load_functions(
    'split_jpeg_frames', 'wide_device_path')


# ---------------------------------------------------------------- 切帧

def test_splits_three_frames_and_keeps_partial_tail():
    data = JPEG_A + JPEG_B + JPEG_C + b'\xff\xd8' + b'tail'
    frames, rest = split_jpeg_frames(data)
    assert frames == [JPEG_A, JPEG_B, JPEG_C]
    assert rest == b'\xff\xd8' + b'tail'      # 半帧要留着，下一块拼上继续切


def test_drops_leading_garbage_before_first_marker():
    frames, rest = split_jpeg_frames(b'\x00garbage' + JPEG_A)
    assert frames == [JPEG_A]
    assert rest == b''


def test_returns_nothing_when_no_marker():
    frames, rest = split_jpeg_frames(b'no jpeg here')
    # 只留最后 1 字节，防止 FFD8 被切在两块数据中间
    assert frames == []
    assert rest == b'e'


def test_keeps_partial_marker_byte():
    frames, rest = split_jpeg_frames(b'xxx\xff')
    assert frames == []
    assert rest == b'\xff'


def test_empty_buffer_is_safe():
    assert split_jpeg_frames(b'') == ([], b'')


def test_two_frames_in_one_chunk_split_correctly():
    frames, rest = split_jpeg_frames(JPEG_B + JPEG_B)
    assert frames == [JPEG_B, JPEG_B]
    assert rest == b''


# ---------------------------------------------------------------- 设备查找

def test_wide_device_path_matches_by_id_symlink(tmp_path):
    (tmp_path / 'usb-NOVATEK_ASJ_ZNX_NVT_510550000000100-video-index0').symlink_to(
        '/dev/null')
    wide = tmp_path / 'usb-Xitech_5MP_USB_Camera_20250923107-video-index0'
    wide.symlink_to('/dev/null')
    (tmp_path / 'usb-Xitech_5MP_USB_Camera_20250923107-video-index1').symlink_to(
        '/dev/null')
    found = wide_device_path(str(tmp_path))
    assert found.endswith('usb-Xitech_5MP_USB_Camera_20250923107-video-index0')


def test_wide_device_path_returns_empty_when_absent(tmp_path):
    (tmp_path / 'usb-NOVATEK_ASJ_ZNX_NVT_510550000000100-video-index0').symlink_to(
        '/dev/null')
    assert wide_device_path(str(tmp_path)) == ''
    assert wide_device_path(str(tmp_path / 'nope')) == ''


# ---------------------------------------------------------------- 路由

def test_video_route_switches_hub_by_cam_param():
    src = SERVER.read_text(encoding='utf-8')
    assert "request.query.get('cam') == 'wide'" in src
    # 主摄/广角两个 hub 都要在 app 里注册，且 build_app 能接收第二路
    assert "app['hub_wide'] = hub_wide" in src
    assert 'hub_wide: ' in src


def test_wide_source_is_zero_transcode_and_retries():
    src = SERVER.read_text(encoding='utf-8')
    assert 'self.hub.publish_jpeg(jpeg)' in src      # 透传，不重新编码
    assert "'--stream-to=-'" in src or '--stream-to=-' in src
    assert '广角相机没找到' in src                     # 拔掉时要能重试，不是崩溃


# ---------------------------------------------------------------- 帧率优化

def test_wide_thread_sets_camera_frame_rate_explicitly():
    """不显式 `--set-parm` 时相机可能停在 10 fps（实测就是 10.0）。"""
    src = SERVER.read_text(encoding='utf-8')
    assert "'--set-parm=%d' % self.cam_fps" in src
    assert 'def capture_fps' in src          # 暴露相机真实帧率，便于现场调参
    assert 'cap_fps' in src                  # /healthz 里能看到


def test_idle_viewer_skips_transcoding():
    """没人看的时候不要解码/重编码（720p 每帧 ~12 ms 白烧）。"""
    src = SERVER.read_text(encoding='utf-8')
    assert 'self.hub.has_clients()' in src
    assert 'resample=False' in src
    assert 'def has_clients' in src


def test_deployed_unit_uses_tuned_wide_args():
    unit = (ROOT / 'deploy/systemd/fwr-media.service').read_text(encoding='utf-8')
    assert '--wide-fps 20' in unit
    assert '--wide-width 480' in unit
    assert '--wide-cam-fps 30' in unit


def test_frame_hub_skips_resample_when_asked(tmp_path):
    """publish_jpeg(resample=False) 必须原样存字节，不经过解码/编码。"""
    import ast
    import types
    import cv2
    import numpy as np
    tree = ast.parse(SERVER.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'FrameHub')
    ns = {'cv2': cv2, 'np': np, 'time': __import__('time'),
          'threading': __import__('threading'), 'Optional': object}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), '<hub>', 'exec'), ns)
    hub = ns['FrameHub'](quality=65, fps=0, width=480)
    raw = b'\xff\xd8raw-bytes-not-a-real-jpeg\xff\xd9'
    hub.publish_jpeg(raw, resample=False)
    jpeg, _seq = hub.latest()
    assert jpeg == raw                     # 一模一样，说明没走 cv2
    assert hub.has_clients() is False
