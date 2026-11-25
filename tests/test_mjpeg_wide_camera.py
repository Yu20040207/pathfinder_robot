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
    assert 'self.hub.publish_jpeg(' in src           # 送 hub（需要缩小时才重编码）
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


# ---------------------------------------------------------------- 抽帧

decimate_step = _load_functions('decimate_step')[0]


def test_decimate_keeps_target_rate_not_time_threshold():
    """30 fps 源、目标 20 fps：30 帧里必须发 20 帧（时间阈值只会发 15 帧）。"""
    acc, sent = 0.0, 0
    for _ in range(30):
        send, acc = decimate_step(acc, 20.0, 30.0)
        sent += 1 if send else 0
    assert sent == 20


def test_decimate_half_rate():
    acc, sent = 0.0, 0
    for _ in range(30):
        send, acc = decimate_step(acc, 15.0, 30.0)
        sent += 1 if send else 0
    assert sent == 15


def test_decimate_passes_through_when_target_above_source():
    acc = 0.0
    for _ in range(10):
        send, acc = decimate_step(acc, 60.0, 30.0)
        assert send


def test_decimate_disabled_when_target_zero():
    assert decimate_step(0.0, 0.0, 30.0) == (True, 0.0)


def test_wide_hub_does_not_time_gate_again():
    """广角 hub 的 fps 必须传 0：再套一层时间阈值会把 20 fps 又压成 15。"""
    src = SERVER.read_text(encoding='utf-8')
    assert 'FrameHub(quality=args.wide_quality, fps=0,' in src
    assert 'target_fps=args.wide_fps' in src


def test_wide_uses_half_size_decode_when_downscaling():
    """720p 源缩到 480 宽时用 IMREAD_REDUCED_COLOR_2（实测 3.60 → 1.43 ms）。"""
    src = SERVER.read_text(encoding='utf-8')
    assert 'cv2.IMREAD_REDUCED_COLOR_2' in src
    assert 'def decode_reduce' in src
    assert 'reduce=self.decode_reduce()' in src


def test_decode_reduce_threshold():
    """目标宽 ≤ 源宽一半才砍半；否则（如源 640、目标 480）保持全尺寸。"""
    src = SERVER.read_text(encoding='utf-8')
    assert 'self.hub.width * 2 <= src_w' in src


# ---------------------------------------------------------------- 4G 自动降帧

target_fps_for = _load_functions('target_fps_for')[0]


def test_4g_falls_back_to_lower_fps():
    """默认路由走 ppp0（4G）时降到 20：4G 上行 ~3.5 Mbps，25 fps 要 3.67 Mbps。"""
    assert target_fps_for('ppp0', 25, 20) == 20
    assert target_fps_for('wwan0', 25, 20) == 20
    assert target_fps_for('usb0', 25, 20) == 20


def test_wifi_keeps_high_fps():
    assert target_fps_for('wlo1', 25, 20) == 25
    assert target_fps_for('eth0', 25, 20) == 25
    assert target_fps_for('', 25, 20) == 25          # 查不出来当 WiFi，别无谓降档


def test_link_is_read_from_kernel_route():
    """判据必须是"去公网实际走哪张卡"，不是读 metric 猜。"""
    src = SERVER.read_text(encoding='utf-8')
    assert "'ip', 'route', 'get'" in src
    assert 'def default_route_iface' in src
    assert 'def refresh_link' in src
    assert "'--wide-fps-4g'" in src
    assert "'link'" in src and "'target_fps'" in src


def test_unit_has_4g_fallback_arg():
    unit = (ROOT / 'deploy/systemd/fwr-media.service').read_text(encoding='utf-8')
    assert '--wide-fps-4g 12' in unit
    assert '--wide-fps 20' in unit


def test_dead_client_write_times_out():
    """手机切走后写会一直挂 → 必须超时断开，否则服务端以为还有人看。"""
    src = SERVER.read_text(encoding='utf-8')
    assert 'STREAM_WRITE_TIMEOUT_S' in src
    assert 'asyncio.wait_for(resp.write(' in src


def test_stream_write_timeout_constant_is_defined():
    """2026-11-25 事故：用了 STREAM_WRITE_TIMEOUT_S 却没定义 → handle_stream 立刻 NameError，
    两路画面全黑。这里钉死"引用了的模块级名字必须在源码里定义"。"""
    src = SERVER.read_text(encoding='utf-8')
    assert 'STREAM_WRITE_TIMEOUT_S = ' in src
    assert 'timeout=STREAM_WRITE_TIMEOUT_S' in src


def test_no_undefined_module_constants_used_in_handlers():
    """粗筛：handle_stream 里用到的全大写名字，必须在模块级定义过。"""
    import re
    src = SERVER.read_text(encoding='utf-8')
    handler = src[src.index('async def handle_stream'):src.index('async def handle_snapshot')]
    code = re.sub(r'#[^\n]*', '', handler)                      # 去注释
    code = re.sub(r"'[^'\n]*'|\"[^\"\n]*\"", '', code)          # 去字符串字面量
    used = set(re.findall(r'\b([A-Z][A-Z0-9_]{3,})\b', code))
    defined = set(re.findall(r'^([A-Z][A-Z0-9_]{3,})\s*=', src, re.M))
    assert used - defined == set(), used - defined


route_from_proc_route = _load_functions('route_from_proc_route')[0]

PROC_ROUTE = """Iface\tDestination\tGateway \tFlags\tRefCnt\tUse\tMetric\tMask\t\tMTU\tWindow\tIRTT
wlo1\t00000000\t0101A8C0\t0003\t0\t0\t600\t00000000\t0\t0\t0
enx020c29a39b6d\t00000000\t012BA8C0\t0003\t0\t0\t20100\t00000000\t0\t0\t0
ppp0\t00000000\t00000000\t0001\t0\t0\t20700\t00000000\t0\t0\t0
wlo1\t0001A8C0\t00000000\t0001\t0\t0\t600\t00FFFFFF\t0\t0\t0
"""


def test_route_picks_lowest_metric_default():
    up = {'wlo1': True, 'enx020c29a39b6d': True, 'ppp0': True}
    assert route_from_proc_route(PROC_ROUTE, up) == 'wlo1'


def test_route_skips_down_interface():
    """WiFi 断了就走 ppp0；RNDIS（enx*）metric 更小但**不通外网**，必须跳过。"""
    up = {'wlo1': False, 'enx020c29a39b6d': True, 'ppp0': True}
    assert route_from_proc_route(PROC_ROUTE, up) == 'ppp0'
    up2 = {'wlo1': True, 'enx020c29a39b6d': True, 'ppp0': True}
    assert route_from_proc_route(PROC_ROUTE, up2) == 'wlo1'


def test_route_ignores_non_default_rows():
    up = {'wlo1': True}
    assert route_from_proc_route(PROC_ROUTE.splitlines()[0] + '\n', up) == ''


def test_link_check_does_not_spawn_subprocess():
    """取流线程里跑 `ip route get` 会把广角从 20 fps 拖到 13（2026-11-25 实测）。"""
    src = SERVER.read_text(encoding='utf-8')
    body = src[src.index('def default_route_iface'):src.index('def decimate_step')]
    assert '/proc/net/route' in body
    assert 'subprocess.run' not in body.split('def _route_iface_by_ip')[0]


def test_decimation_uses_measured_source_rate():
    """相机宣称 30、实际 20：拿 30 当分母会把 20fps 抽成 13.3（2026-11-25 现场）。"""
    src = SERVER.read_text(encoding='utf-8')
    assert 'def effective_source' in src
    assert 'self.effective_source())' in src
    # 源帧率等于目标时，必须每帧都发
    acc, sent = 0.0, 0
    for _ in range(20):
        send, acc = decimate_step(acc, 20.0, 20.0)
        sent += 1 if send else 0
    assert sent == 20


def test_stream_cap_evicts_oldest_connection():
    """浏览器同域只有 6 条并发连接，僵尸连接必须被服务端挤掉，否则新画面永远黑。"""
    src = SERVER.read_text(encoding='utf-8')
    assert 'MAX_STREAM_CLIENTS = 2' in src
    body = src[src.index('async def handle_stream'):src.index('async def handle_snapshot')]
    assert "app['streams']" in body
    assert 'while len(streams) > MAX_STREAM_CLIENTS' in body
    assert 'old.set()' in body and 'mine.is_set()' in body


def test_wide_camera_stops_when_nobody_watches():
    """用户要求：切到主摄之后广角要停流，别一直占着 USB/CPU。"""
    src = SERVER.read_text(encoding='utf-8')
    assert '--wide-idle-pause' in src
    assert 'idle_pause_s' in src
    assert '停流释放设备' in src
