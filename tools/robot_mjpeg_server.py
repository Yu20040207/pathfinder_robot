#!/usr/bin/env python3
"""相机实时画面 HTTP 服务（MJPEG，零额外依赖）。

为什么用它：4G 模块还没到，先用办公室 WiFi 把 HP60C 的画面推到手机。
页面按 `/api/devices/<id>/media` 返回的 `{type, url}` 播放，以后换 4G 只改配置。

用法（机器人上）：
    source /opt/ros/humble/setup.bash
    .venv/bin/python tools/robot_mjpeg_server.py --port 8081

    # 无相机时先验证服务本身（本地生成测试画面）
    .venv/bin/python tools/robot_mjpeg_server.py --test-pattern --port 8081

两种取流方式（相机驱动 SDK 在这台 HP60C 上会在 1 s 后自己停流，所以默认走 UVC 裸流直读）：
    --v4l2 /dev/video0   默认方式：直接读 UVC（v4l2-ctl 子进程 + 自己拆包）
    （不给 --v4l2 时）     走 ROS 话题 /robot/camera/color/image_raw

端点：
    GET /video          multipart/x-mixed-replace 流（<img src> 直接显示）
    GET /snapshot.jpg   单帧抓拍（几十 KB，手机点一下看细节用）
    GET /healthz        JSON：{ok, fps, clients, age_ms, topic}
    GET /boxes          JSON：人体检测结果（跟随模式用，1 秒内没有新结果会标 stale）
    GET /select?id=N    选中/取消跟随目标（N<0 取消），转发到 /robot/vision/select
    GET /follow?on=1    开始/取消跟随（on=1 同时是**心跳**，页面每 1 s 发一次；>2 s 不发给就自动停）
    GET /followstate    跟随状态 JSON（速度/原因/距离/前方净空）
    GET /               自检页（浏览器直接打开看画面）
"""

from __future__ import annotations

import argparse
import asyncio
import io
import json
import math
import os
import sys
import subprocess
import threading
import time
from typing import Optional

import cv2
import numpy as np
from aiohttp import web

BOUNDARY = 'fwrframe'
MAX_WIDTH = 1280

# 人体检测结果桥接（2026-11-21 跟随模式）：person_detector 节点 -> HTTP
VISION_LATEST: dict = {'json': '', 't': 0.0}
VISION_PUB = [None]
FOLLOW_LATEST: dict = {'json': '', 't': 0.0}
FOLLOW_PUB = [None]
CORS = {'Access-Control-Allow-Origin': '*'}


class FrameHub:
    """持有最新一帧 JPEG，供 HTTP 端读取。"""

    def __init__(self, quality: int = 70, fps: float = 12.0, width: int = 0) -> None:
        self.quality = int(quality)
        # fps<=0 表示不限速：压缩帧是原样转发，限速只会把新帧丢掉、越限越卡
        self.min_period = 0.0 if float(fps) <= 0 else 1.0 / max(0.5, float(fps))
        self.width = int(width)
        self._lock = threading.Lock()
        self._jpeg: Optional[bytes] = None
        self._stamp = 0.0            # 采集时间（monotonic）
        self._seq = 0
        self._frames = 0             # 累计帧数
        self._real_frames = 0        # 来自真实相机（非测试图案）的帧数
        self._encode_ms = 0.0
        self._last_encode = 0.0
        self._clients = 0
        self._event = threading.Event()      # 供 asyncio 侧等待（轮询式）
        self.dds_age_ms = 0.0                # DDS 链路时延（节点 stamp -> 服务收到）
        self._fps_window: list[float] = []

    # ---- 采集侧（ROS 线程 / 测试线程调用）----
    def publish(self, image: np.ndarray, real: bool = True) -> None:
        now = time.monotonic()
        if now - self._last_encode < self.min_period:
            return
        img = image
        if self.width and img.shape[1] > self.width:
            scale = self.width / img.shape[1]
            img = cv2.resize(img, (self.width, int(img.shape[0] * scale)),
                             interpolation=cv2.INTER_AREA)
        t0 = time.monotonic()
        ok, buf = cv2.imencode('.jpg', img,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            return
        data = buf.tobytes()
        with self._lock:
            self._jpeg = data
            self._stamp = now
            self._seq += 1
            self._frames += 1
            if real:
                self._real_frames += 1
            self._last_encode = now
            self._encode_ms = (time.monotonic() - t0) * 1000.0
            self._fps_window.append(now)
            if len(self._fps_window) > 120:
                self._fps_window = self._fps_window[-120:]
        self._event.set()

    def publish_jpeg(self, data: bytes) -> None:
        """已经是 JPEG 帧：**需要缩小分辨率时才解码重编码**，否则原样透传。

        2026-11-21：手机端还嫌延迟，把画面降到 `--width 480`（480x360）。
        640x480 q92 一帧 44 KB，480x360 q70 只有 12~16 KB —— 传输时间与手机解码
        都省一半以上。注意这里是**唯一**需要二次编码的路径（宽度设了才会走）。
        """
        now = time.monotonic()
        if self.min_period and now - self._last_encode < self.min_period:
            return
        enc_ms = 0.0
        if self.width:
            t0 = time.monotonic()
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                return
            if img.shape[1] > self.width:
                scale = self.width / img.shape[1]
                img = cv2.resize(img, (self.width, int(round(img.shape[0] * scale))),
                                 interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode('.jpg', img,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
            if not ok:
                return
            data = buf.tobytes()
            enc_ms = (time.monotonic() - t0) * 1000.0
        with self._lock:
            self._jpeg = data
            self._stamp = now
            self._seq += 1
            self._frames += 1
            self._real_frames += 1
            self._last_encode = now
            if self.width:
                self._encode_ms = enc_ms
            self._fps_window.append(now)
            if len(self._fps_window) > 120:
                self._fps_window = self._fps_window[-120:]
        self._event.set()

    # ---- 读取侧 ----
    def latest(self) -> tuple[Optional[bytes], int]:
        with self._lock:
            return self._jpeg, self._seq

    def _fps_locked(self) -> float:
        """调用方必须已持有 self._lock（Lock 不可重入，别再套一层）。"""
        w = self._fps_window
        if len(w) < 2:
            return 0.0
        span = w[-1] - w[0]
        return (len(w) - 1) / span if span > 0.2 else 0.0

    def fps(self) -> float:
        with self._lock:
            return self._fps_locked()

    def snapshot(self) -> dict:
        with self._lock:
            age = (time.monotonic() - self._stamp) * 1000.0 if self._stamp else -1.0
            return {
                'ok': self._jpeg is not None,
                'fps': round(self._fps_locked(), 1),
                'clients': self._clients,
                'frames': self._frames,
                'age_ms': round(age, 0),
                'encode_ms': round(self._encode_ms, 1),
                'kb': round(len(self._jpeg) / 1024.0, 1) if self._jpeg else 0.0,
                'source': 'camera' if self._real_frames else 'test',
            }

    def has_real_frames(self) -> bool:
        with self._lock:
            return self._real_frames > 0

    def add_client(self, delta: int) -> None:
        with self._lock:
            self._clients = max(0, self._clients + delta)


# ---------------------------------------------------------------- HTTP

async def handle_stream(request: web.Request) -> web.StreamResponse:
    hub: FrameHub = request.app['hub']
    resp = web.StreamResponse(status=200, headers={
        'Content-Type': f'multipart/x-mixed-replace; boundary={BOUNDARY}',
        'Cache-Control': 'no-store, no-cache, must-revalidate, max-age=0',
        'Pragma': 'no-cache',
        'Connection': 'close',
        'Access-Control-Allow-Origin': '*',
    })
    await resp.prepare(request)
    hub.add_client(1)
    last_seq = -1
    try:
        while True:
            jpeg, seq = hub.latest()
            if jpeg is None or seq == last_seq:
                if request.transport is None:            # 客户端已断开
                    break
                await asyncio.sleep(0.02)
                continue
            last_seq = seq
            head = (f'--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n'
                    f'Content-Length: {len(jpeg)}\r\n\r\n').encode()
            await resp.write(head + jpeg + b'\r\n')
    except (asyncio.CancelledError, ConnectionResetError, RuntimeError):
        pass
    finally:
        hub.add_client(-1)
    return resp


async def handle_snapshot(request: web.Request) -> web.Response:
    hub: FrameHub = request.app['hub']
    jpeg, _ = hub.latest()
    if jpeg is None:
        return web.Response(status=503, text='no frame yet')
    return web.Response(body=jpeg, content_type='image/jpeg',
                        headers={'Cache-Control': 'no-store',
                                 'Access-Control-Allow-Origin': '*'})


async def handle_health(request: web.Request) -> web.Response:
    hub: FrameHub = request.app['hub']
    body = hub.snapshot()
    body['topic'] = request.app['topic']
    body['dds_ms'] = round(hub.dds_age_ms, 1)
    uvc = request.app.get('uvc')
    if uvc is not None:
        body['depth_mm'] = uvc.depth_median_mm()
    body['uptime_s'] = round(time.monotonic() - request.app['t0'], 0)
    return web.json_response(body)


INDEX_HTML = """<!doctype html><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>相机自检</title>
<style>body{margin:0;background:#000;color:#9fb3c8;font:13px/1.5 system-ui}
img{width:100%;display:block}#h{padding:6px 10px}</style>
<div id="h">相机自检 · /healthz</div>
<img src="/video" alt="相机画面">
<script>fetch('/healthz').then(r=>r.json()).then(d=>{
  document.getElementById('h').textContent =
    `相机自检 · ${d.fps} fps · ${d.kb} KB/帧 · 客户端 ${d.clients}`;
}).catch(()=>{});</script>
"""


async def handle_index(request: web.Request) -> web.Response:
    return web.Response(text=INDEX_HTML, content_type='text/html')


async def handle_boxes(request: web.Request) -> web.Response:
    """人体框（归一化坐标）。1 秒没更新就返回空 + stale，页面显示“未识别到”而不是冻结旧框。"""
    stale = (time.monotonic() - VISION_LATEST['t']) > 2.0
    body = VISION_LATEST['json'] if (VISION_LATEST['json'] and not stale) else ''
    if not body:
        body = json.dumps({'persons': [], 'sel': -1, 'tracking': False,
                           'stale': True})
    return web.Response(text=body, content_type='application/json', headers=CORS)


async def handle_select(request: web.Request) -> web.Response:
    """选中要跟随的人（id<0 取消）。"""
    try:
        sid = int(request.query.get('id', '-1'))
    except ValueError:
        sid = -1
    pub = VISION_PUB[0]
    if pub is None:
        return web.Response(text=json.dumps({'ok': False, 'err': 'vision 未就绪'}),
                            content_type='application/json', status=503, headers=CORS)
    try:
        from std_msgs.msg import String as RosString
        pub.publish(RosString(data=json.dumps({'id': sid})))
    except Exception as exc:            # noqa: BLE001
        return web.Response(text=json.dumps({'ok': False, 'err': str(exc)}),
                            content_type='application/json', status=500, headers=CORS)
    return web.Response(text=json.dumps({'ok': True, 'id': sid}),
                        content_type='application/json', headers=CORS)


async def handle_follow(request: web.Request) -> web.Response:
    """开始/取消跟随。on=1 兼心跳：页面每 1 s 发一次，节点 2 s 收不到就停车。"""
    on = request.query.get('on', '0') in ('1', 'true', 'yes')
    pub = FOLLOW_PUB[0]
    if pub is None:
        return web.Response(text=json.dumps({'ok': False, 'err': 'follow 未就绪'}),
                            content_type='application/json', status=503, headers=CORS)
    try:
        from std_msgs.msg import String as RosString
        pub.publish(RosString(data=json.dumps({'on': on, 't': time.time()})))
    except Exception as exc:            # noqa: BLE001
        return web.Response(text=json.dumps({'ok': False, 'err': str(exc)}),
                            content_type='application/json', status=500, headers=CORS)
    return web.Response(text=json.dumps({'ok': True, 'on': on}),
                        content_type='application/json', headers=CORS)


async def handle_follow_state(request: web.Request) -> web.Response:
    stale = (time.monotonic() - FOLLOW_LATEST['t']) > 2.0
    body = FOLLOW_LATEST['json'] if (FOLLOW_LATEST['json'] and not stale) else ''
    if not body:
        body = json.dumps({'on': False, 'reason': 'no_node', 'stale': True,
                           'v': 0.0, 'w': 0.0, 'dist_m': 0.0})
    return web.Response(text=body, content_type='application/json', headers=CORS)


def build_app(hub: FrameHub, topic: str, uvc: 'UvcSource' = None) -> web.Application:
    app = web.Application()
    app['hub'] = hub
    app['topic'] = topic
    app['uvc'] = uvc
    app['t0'] = time.monotonic()
    app.add_routes([
        web.get('/', handle_index),
        web.get('/video', handle_stream),
        web.get('/snapshot.jpg', handle_snapshot),
        web.get('/healthz', handle_health),
        web.get('/boxes', handle_boxes),
        web.get('/select', handle_select),
        web.get('/follow', handle_follow),
        web.get('/followstate', handle_follow_state),
    ])
    return app


# ---------------------------------------------------------------- 采集

def run_ros(hub: FrameHub, topic: str, node_name: str,
            compressed_topic: str = '') -> None:
    """订阅 sensor_msgs/Image 并把帧送进 hub。"""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image

    rclpy.init()
    node = Node(node_name)
    node.get_logger().info(f'订阅 {topic}（MJPEG 服务）')

    # 跟随模式：把 /robot/vision/persons 缓存下来给 HTTP 取，把 /select 转发给节点
    try:
        from std_msgs.msg import String as RosString
        VISION_PUB[0] = node.create_publisher(RosString, '/robot/vision/select', 10)

        def on_persons(msg) -> None:
            VISION_LATEST['json'] = msg.data
            VISION_LATEST['t'] = time.monotonic()

        node.create_subscription(RosString, '/robot/vision/persons', on_persons, 10)
        FOLLOW_PUB[0] = node.create_publisher(RosString, '/follow/enable', 10)

        def on_follow_state(msg) -> None:
            FOLLOW_LATEST['json'] = msg.data
            FOLLOW_LATEST['t'] = time.monotonic()

        node.create_subscription(RosString, '/follow/state', on_follow_state, 10)
        node.get_logger().info('人体检测桥接：/robot/vision/persons → GET /boxes')
    except Exception as exc:            # noqa: BLE001
        node.get_logger().warning(f'人体检测桥接没挂上：{exc}')

    last_compressed = [0.0]

    def on_compressed(msg: CompressedImage) -> None:
        last_compressed[0] = time.monotonic()
        try:
            now_ns = node.get_clock().now().nanoseconds
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            hub.dds_age_ms = (now_ns - stamp_ns) / 1e6
        except Exception:
            pass
        hub.publish_jpeg(bytes(msg.data))

    def on_image(msg: Image) -> None:
        # 有 compressed 流就用它（相机本来就是 JPEG，别再解一次）
        if time.monotonic() - last_compressed[0] < 2.0:
            return
        try:
            h, w = msg.height, msg.width
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding in ('bgr8', 'rgb8'):
                img = buf.reshape(h, w, 3)
                if msg.encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding in ('mono8', '8UC1'):
                img = cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
            else:                       # 其它格式放弃（如 16UC1 深度）
                return
            hub.publish(np.ascontiguousarray(img))
        except Exception as exc:        # 坏帧不能拖死整个服务
            node.get_logger().warn(f'坏帧: {exc}', throttle_duration_sec=5.0)

    # 2026-11-21：原始图**不能**无条件订阅。相机节点一看有订阅者就每帧解码
    # 34 KB JPEG → 921 KB 原始图再经 DDS 发出去（15 fps ≈ 13.8 MB/s），
    # 实测把 hp60c_uvc_node 的 CPU 从 ~30% 顶到 ~90%，整个主控跟着抖，
    # 手机摇杆就"一卡一卡"。现在：有 compressed 就只订 compressed；
    # 3 秒仍没有 compressed 才退回订阅原始图（兜底），一旦 compressed 到了就退订。
    raw_sub = [None]

    def _fallback_raw() -> None:
        if raw_sub[0] is not None:
            return
        raw_sub[0] = node.create_subscription(
            Image, topic, on_image, qos_profile_sensor_data)
        node.get_logger().warn(
            f'没等到压缩话题，退回订阅原始图 {topic}（相机节点会多解码一次）')

    def _drop_raw() -> None:
        if raw_sub[0] is None:
            return
        try:
            node.destroy_subscription(raw_sub[0])
        except Exception:                # noqa: BLE001
            pass
        raw_sub[0] = None
        node.get_logger().info('压缩话题已到，退订原始图（省一次解码）')

    if compressed_topic:
        def on_compressed_entry(msg) -> None:
            _drop_raw()
            on_compressed(msg)

        node.create_subscription(CompressedImage, compressed_topic,
                                 on_compressed_entry, qos_profile_sensor_data)
        node.get_logger().info('优先使用压缩话题 %s' % compressed_topic)
        node.create_timer(
            1.0, lambda: _fallback_raw()
            if time.monotonic() - last_compressed[0] > 3.0 else None)
    else:
        _fallback_raw()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


# ---------------------------------------------------------------- UVC 直读
#
# 拆包/解码逻辑与 ROS 节点共用一份：ros2_ws/src/fwr_bringup/fwr_bringup/hp60c_uvc.py
# （HP60C 的厂商 SDK 在本机 1 s 后自己停流，只能直接读 UVC，见 docs/hardware/vision.md §3.6.2）

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2_ws', 'src', 'fwr_bringup'))
from fwr_bringup.hp60c_uvc import Hp60cUvcStream, decode_rgb   # noqa: E402


class UvcSource(threading.Thread):
    """把 HP60C 的 UVC 裸流解出 RGB，喂给 hub；断流由 Hp60cUvcStream 自己重试。"""

    def __init__(self, hub: FrameHub, device: str, size: str) -> None:
        super().__init__(daemon=True)
        self.hub = hub
        self.stream = Hp60cUvcStream(device, size)
        self.stream.on_retry = lambda msg: print(msg, flush=True)
        self._depth_mm = 0.0

    def stop(self) -> None:
        self.stream.stop()

    def depth_median_mm(self) -> float:
        return round(self._depth_mm, 0)

    def _sample_depth(self, raw: bytes) -> None:
        try:
            arr = np.frombuffer(raw, '<u2')
            nz = arr[arr > 0]
            if nz.size > 1000:
                self._depth_mm = float(np.median(nz)) / 10.0   # 0.1 mm -> mm
        except (ValueError, TypeError):
            pass

    def run(self) -> None:
        for jpeg, depth in self.stream.frames():
            self._sample_depth(depth)
            img = decode_rgb(jpeg)
            if img is not None:
                self.hub.publish(img, real=True)


def run_test_pattern(hub: FrameHub, stop_when_real: bool = False) -> None:
    """无相机时的自检画面：移动渐变 + 帧号 + 时间。

    stop_when_real=True 时，一旦真实相机开始出帧就退出（用于开机兜底）。
    """
    t0 = time.monotonic()
    idx = 0
    while True:
        if stop_when_real and hub.has_real_frames():
            print('真实相机已出帧，停止测试图案', flush=True)
            return
        idx += 1
        t = time.monotonic() - t0
        h, w = 480, 640
        x = np.linspace(0, 255, w, dtype=np.uint8)
        img = np.tile(x, (h, 1))
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.circle(img, (int(320 + 200 * math.sin(t)), int(240 + 120 * math.cos(t))),
                   40, (0, 200, 255), -1)
        cv2.putText(img, f'TEST {idx}', (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, (255, 255, 255), 3)
        cv2.putText(img, time.strftime('%H:%M:%S'), (20, 440),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        hub.publish(img, real=False)
        time.sleep(0.05)


# ---------------------------------------------------------------- main

def main() -> int:
    ap = argparse.ArgumentParser(description='相机 MJPEG HTTP 服务')
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=8081)
    ap.add_argument('--topic', default='/robot/camera/color/image_raw')
    ap.add_argument('--compressed-topic',
                    default='/robot/camera/color/image_raw/compressed',
                    help='优先订阅该 JPEG 话题（空字符串=不用）')
    ap.add_argument('--quality', type=int, default=70)
    ap.add_argument('--fps', type=float, default=12.0)
    ap.add_argument('--width', type=int, default=640,
                    help='超过该宽度先缩放（0 = 不缩放）')
    ap.add_argument('--v4l2', default='',
                    help='UVC 设备直读，如 /dev/video0（HP60C 只能这样取流）')
    ap.add_argument('--v4l2-size', default='640x642',
                    help='HP60C 的 UVC 帧尺寸（不是画面尺寸：画面是 640x480）')
    ap.add_argument('--test-pattern', action='store_true',
                    help='完全用测试图案（无 ROS）')
    ap.add_argument('--fallback-test-pattern', action='store_true',
                    help='先出测试图案兜底，真实相机一出帧就切走')
    ap.add_argument('--node-name', default='fwr_media')
    args = ap.parse_args()

    hub = FrameHub(quality=args.quality, fps=args.fps, width=args.width)
    uvc = None
    if args.v4l2:
        uvc = UvcSource(hub, args.v4l2, args.v4l2_size)
        if args.fallback_test_pattern:
            threading.Thread(target=run_test_pattern, args=(hub, True),
                             daemon=True).start()
        uvc.start()
    elif args.test_pattern:
        threading.Thread(target=run_test_pattern, args=(hub, False),
                         daemon=True).start()
    else:
        threading.Thread(target=run_ros,
                         args=(hub, args.topic, args.node_name,
                               args.compressed_topic),
                         daemon=True).start()
        if args.fallback_test_pattern:
            threading.Thread(target=run_test_pattern, args=(hub, True),
                             daemon=True).start()

    topic = args.v4l2 or ('test-pattern' if args.test_pattern else args.topic)
    app = build_app(hub, topic, uvc)
    print(f'MJPEG 服务: http://{args.host}:{args.port}/video  '
          f'(topic={app["topic"]}, {args.fps:g} fps, q={args.quality})', flush=True)
    try:
        web.run_app(app, host=args.host, port=args.port, print=None)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
