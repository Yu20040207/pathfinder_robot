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
#: 单帧写操作的最长等待（秒）；超时说明手机已经不看了，断开这条连接
STREAM_WRITE_TIMEOUT_S = 6.0
MAX_WIDTH = 1280

# 人体检测结果桥接（2026-11-21 跟随模式）：person_detector 节点 -> HTTP
VISION_LATEST: dict = {'json': '', 't': 0.0}
VISION_PUB = [None]
FOLLOW_LATEST: dict = {'json': '', 't': 0.0}
FOLLOW_PUB = [None]

# 2026-11-23（用户要求）：视觉/跟随服务**只在切到跟随模式时启动**，
# 离开跟随就停掉。原来两个 systemd 服务常驻，导航时白吃 ~25% CPU
# （YOLO 推理 + 跟随控制），而导航根本用不到它们。
#   cfg  = 最近一次下发的目标状态（True=要跑，False=要停，None=还没定）
#   at   = 最后一次收到页面心跳的时间
#   busy = 已有一个 systemctl 在跑（页面每 1 s 心跳一次，不能一次刷一堆）
FOLLOW_SVC: dict = {'cfg': None, 'at': 0.0, 'busy': False}
FOLLOW_HEARTBEAT_LOST_S = 25.0     # 页面关掉/断网后多久停服务（页面 5 s 一次 prep 心跳）
FOLLOW_LAST_CFG = [None]           # 上一次生效的服务状态（只在变化时打日志）
FOLLOW_UNITS = ('fwr-vision', 'fwr-follow')


def follow_service_target(on: bool, prep: bool, leave: bool) -> bool | None:
    """决定"要不要让视觉/跟随服务跑"：True=起、False=停、None=不动。

    2026-11-24 用户报「有时候提示人体识别没在跑」。真因不是识别坏了，而是
    **页面和服务互相拉扯**：

    * 旧逻辑 ``on=0``（只是"暂停跟随/切后台停车"）也会**停掉 fwr-vision + fwr-follow**；
    * 可页面还在跟随模式里，5 s 后又发 ``prep=1`` 把它们**拉起来**；
    * 于是每十几秒 stop → start 一次。vision 每次收到 SIGINT 都在
      ``main()`` 里二次 ``rclpy.shutdown()`` → ``ExternalShutdownException``
      → core-dump，重启期间手机端就是"人体识别没在跑"。

    现在只有**真的离开跟随模式**（``leave=1``）才停服务；``on=0`` 一律不动。
    """
    if on or prep:
        return True
    if leave:
        return False
    return None


def _follow_services(on: bool, force: bool = False) -> None:
    """按需启动/停止视觉与跟随服务（后台线程，不阻塞 HTTP）。"""
    if FOLLOW_SVC['busy']:
        return
    if not force and FOLLOW_SVC['cfg'] == on:
        return
    FOLLOW_SVC['cfg'] = on
    FOLLOW_SVC['busy'] = True
    if FOLLOW_LAST_CFG[0] != on:
        FOLLOW_LAST_CFG[0] = on
        print('[follow] 视觉/跟随服务 → %s' % ('启动' if on else '停止'), flush=True)

    def run() -> None:
        try:
            if not on:
                time.sleep(0.6)     # 先让跟随节点把 on=0 的零速发出去再停
            subprocess.run(['sudo', '-n', 'systemctl',
                            'start' if on else 'stop', *FOLLOW_UNITS],
                           timeout=40, check=False)
            if not on:
                # 节点收到 SIGTERM 会以 1 退出，systemd 记成 failed（功能没问题，
                # 但 `systemctl is-active` 一直显示 failed 很误导），清一下。
                subprocess.run(['sudo', '-n', 'systemctl', 'reset-failed',
                                *FOLLOW_UNITS], timeout=20, check=False)
        except Exception:           # noqa: BLE001
            pass
        finally:
            FOLLOW_SVC['busy'] = False

    threading.Thread(target=run, daemon=True).start()
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

    def publish_jpeg(self, data: bytes, resample: bool = True,
                     reduce: int = 1) -> None:
        """已经是 JPEG 帧：**需要缩小分辨率时才解码重编码**，否则原样透传。

        `resample=False` 用于"没人看"的时候：只留一张原始帧给 `/snapshot.jpg`，
        不解码不缩放（广角 720p 每帧解码+编码要 ~12 ms，白烧）。

        `reduce=2` 用 `IMREAD_REDUCED_COLOR_2` 按 1/2 尺寸解码（720p → 640x360）：
        实测"解码+缩放+编码" **3.60 → 1.43 ms**（2026-11-25，i5-13500H 本地）。
        目标宽度只有源一半以下时用它，画质无可见差别。

        2026-11-21：手机端还嫌延迟，把画面降到 `--width 480`（480x360）。
        640x480 q92 一帧 44 KB，480x360 q70 只有 12~16 KB —— 传输时间与手机解码
        都省一半以上。注意这里是**唯一**需要二次编码的路径（宽度设了才会走）。
        """
        now = time.monotonic()
        if self.min_period and now - self._last_encode < self.min_period:
            return
        enc_ms = 0.0
        if resample and self.width:
            t0 = time.monotonic()
            flag = (cv2.IMREAD_REDUCED_COLOR_2 if reduce >= 2
                    else cv2.IMREAD_COLOR)
            img = cv2.imdecode(np.frombuffer(data, np.uint8), flag)
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

    def clients(self) -> int:
        with self._lock:
            return self._clients

    def has_clients(self) -> bool:
        """有没有人在看（广角相机用它做"没人看就省电省 CPU"）。"""
        with self._lock:
            return self._clients > 0

    def has_real_frames(self) -> bool:
        with self._lock:
            return self._real_frames > 0

    def add_client(self, delta: int) -> None:
        with self._lock:
            self._clients = max(0, self._clients + delta)


# ---------------------------------------------------------------- HTTP

def hub_for(request: web.Request) -> tuple['FrameHub', str]:
    """`?cam=wide` → 广角相机；其余一律主摄（HP60C）。"""
    wide = request.app.get('hub_wide')
    if request.query.get('cam') == 'wide' and wide is not None:
        return wide, 'wide'
    return request.app['hub'], 'main'


async def handle_stream(request: web.Request) -> web.StreamResponse:
    hub, _cam = hub_for(request)
    resp = web.StreamResponse(status=200, headers={
        'Content-Type': f'multipart/x-mixed-replace; boundary={BOUNDARY}',
        'Cache-Control': 'no-store, no-cache, must-revalidate, max-age=0',
        'Pragma': 'no-cache',
        'Connection': 'close',
        'Access-Control-Allow-Origin': '*',
    })
    await resp.prepare(request)
    hub.add_client(1)
    if hub.clients() > 2:
        print('MJPEG 同时在看的连接数：%d（主摄/广角）' % hub.clients(), flush=True)
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
            # 2026-11-25：手机切走/切后台后 TCP 不一定立刻 RST，写会一直挂着 →
            # 服务端以为"还有人看"，既白烧 CPU 又占浏览器连接名额。超时就当断开。
            try:
                await asyncio.wait_for(resp.write(head + jpeg + b'\r\n'),
                                       timeout=STREAM_WRITE_TIMEOUT_S)
            except asyncio.TimeoutError:
                print('MJPEG 客户端写超时（%.0fs），断开这条连接（可能已切走）'
                      % STREAM_WRITE_TIMEOUT_S, flush=True)
                break
    except (asyncio.CancelledError, ConnectionResetError, RuntimeError):
        pass
    finally:
        hub.add_client(-1)
    return resp


async def handle_snapshot(request: web.Request) -> web.Response:
    hub, _cam = hub_for(request)
    jpeg, _ = hub.latest()
    if jpeg is None:
        return web.Response(status=503, text='no frame yet')
    return web.Response(body=jpeg, content_type='image/jpeg',
                        headers={'Cache-Control': 'no-store',
                                 'Access-Control-Allow-Origin': '*'})


async def handle_health(request: web.Request) -> web.Response:
    hub, _cam = hub_for(request)
    body = hub.snapshot()
    wide = request.app.get('hub_wide')
    if wide is not None:
        body['wide'] = wide.snapshot()
        body['wide']['device'] = request.app.get('wide_device', '')
        src = request.app.get('wide_src')
        if src is not None:
            body['wide']['cap_fps'] = src.capture_fps()
            body['wide']['link'] = getattr(src, 'link', '')
            body['wide']['target_fps'] = getattr(src, 'target_fps', 0)
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
<div id="h">相机自检 · /healthz · <a href="?cam=wide" style="color:#38bdf8">广角</a></div>
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
    # 2026-11-23：prep=1 = 「我要用跟随了，把服务拉起来」但**不进跟随控制**。
    # 页面一进跟随模式就发 prep（不等用户点「开始跟随」），否则要等用户点按钮才启动，
    # 中间一直是「人体识别没在跑」。
    prep = request.query.get('prep', '0') in ('1', 'true', 'yes')
    # 2026-11-24：只有离开跟随模式（leave=1）才停服务；on=0 只是暂停跟随。
    # 旧写法 on=0 也停 → 和页面的 prep 心跳互相拉扯 → 服务被反复重启。
    leave = request.query.get('leave', '0') in ('1', 'true', 'yes')
    target = follow_service_target(on, prep, leave)
    if target is not None:
        _follow_services(target)
    if on or prep:
        FOLLOW_SVC['at'] = time.monotonic()
    if leave:
        return web.Response(
            text=json.dumps({'ok': True, 'leave': True}), content_type='application/json',
            headers=CORS)
    if prep:
        return web.Response(
            text=json.dumps({'ok': True, 'prep': True,
                             'vision': FOLLOW_SVC['cfg']}),
            content_type='application/json', headers=CORS)
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


async def _follow_service_watchdog(_app) -> None:
    """页面关掉/断网后自动收回：>15 s 没有 on=1 心跳就停服务、发 on=0。"""
    while True:
        await asyncio.sleep(2.0)
        if not FOLLOW_SVC['cfg']:
            continue
        if time.monotonic() - FOLLOW_SVC['at'] < FOLLOW_HEARTBEAT_LOST_S:
            continue
        print('[follow] 心跳丢失 %.0f s（页面可能关了）→ 停跟随/视觉服务'
              % (time.monotonic() - FOLLOW_SVC['at']), flush=True)
        pub = FOLLOW_PUB[0]
        if pub is not None:
            try:
                from std_msgs.msg import String as RosString
                pub.publish(RosString(
                    data=json.dumps({'on': False, 't': time.time()})))
            except Exception:       # noqa: BLE001
                pass
        _follow_services(False)


async def _start_follow_watchdog(app) -> None:
    """把守护协程挂成**后台任务**。

    2026-11-23：一开始直接 `app.on_startup.append(_follow_service_watchdog)`，
    而 aiohttp 会 **await** 启动回调 —— 那个函数是死循环，于是服务永远卡在
    启动阶段、8081 根本没监听（手机连不上、curl 空响应）。
    """
    app['follow_task'] = asyncio.create_task(_follow_service_watchdog(app))


async def _follow_services_baseline(_app) -> None:
    """服务启动时先确保视觉/跟随是停的（巡航/导航不需要它们）。"""
    _follow_services(False, force=True)


def build_app(hub: FrameHub, topic: str, uvc: 'UvcSource' = None,
              hub_wide: 'FrameHub' = None, wide_device: str = '',
              wide_src=None) -> web.Application:
    app = web.Application()
    app['hub'] = hub
    app['hub_wide'] = hub_wide
    app['wide_device'] = wide_device
    app['wide_src'] = wide_src
    app['topic'] = topic
    app['uvc'] = uvc
    app['t0'] = time.monotonic()
    app.on_startup.append(_follow_services_baseline)
    app.on_startup.append(_start_follow_watchdog)
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


WIDE_BY_ID_DIR = '/dev/v4l/by-id'
WIDE_ID_HINT = '5MP_USB_Camera'


def wide_device_path(by_id_dir: str = WIDE_BY_ID_DIR,
                     hint: str = WIDE_ID_HINT) -> str:
    """找 120° 广角相机的稳定路径（`/dev/v4l/by-id/...5MP_USB_Camera...index0`）。

    ⚠️ **不要写死 `/dev/video2`**：插拔/重枚举会让编号变（2026-11-25 实测
    深度相机占 video0/1，广角相机是从 video2 开始的，但顺序不保证）。
    找不到就返回空串，采集线程会一直重试。
    """
    try:
        names = sorted(os.listdir(by_id_dir))
    except OSError:
        return ''
    for name in names:
        if hint in name and name.endswith('-video-index0'):
            return os.path.join(by_id_dir, name)
    return ''


CELLULAR_IFACE_PREFIXES = ('ppp', 'wwan', 'rmnet', 'usb')


def target_fps_for(iface: str, lan_fps: float, cell_fps: float) -> float:
    """默认路由是 4G（ppp0 等）就降到 `cell_fps`，WiFi/有线用 `lan_fps`。

    4G 上行实测 ~3.5 Mbps（2026-11-25），25 fps 的广角要 3.67 Mbps，跑在 4G 上会把
    链路吃满 → 主摄、MQTT、页面请求一起卡。所以链路一变就自动降档。
    """
    name = (iface or '').strip()
    if not name:
        return float(lan_fps)          # 查不出来就当 WiFi，别无谓降画质
    if name.startswith(CELLULAR_IFACE_PREFIXES):
        return float(cell_fps)
    return float(lan_fps)


def route_from_proc_route(text: str, link_up: dict) -> str:
    """从 `/proc/net/route` 找默认路由里 metric 最小的那张网卡（纯函数，好测）。

    ⚠️ **不能用 `subprocess` 每几秒去跑一次 `ip route get`**：实测那一下要几十到上百
    毫秒，跑在取流线程里会把广角帧率从 20 拉到 13（2026-11-25 实测）。
    `link_up` 传 {网卡名: True/False}（来自 /sys/class/net/<dev>/operstate）。
    """
    best = ''
    best_metric = None
    for line in text.splitlines()[1:]:
        parts = line.split()
        if len(parts) < 8:
            continue
        iface, dest, _gw, flags, _ref, _use, metric = parts[0], parts[1], parts[2], parts[3], parts[4], parts[5], parts[6]
        if dest != '00000000':
            continue
        try:
            if not (int(flags, 16) & 0x1):        # RTF_UP
                continue
            m = int(metric)
        except ValueError:
            continue
        if link_up and link_up.get(iface) is False:
            continue
        # `enx*` = USB 网卡，本车上就是 4G 模块的 RNDIS 口（192.168.43.1，不通外网）。
        # 它 metric 比 ppp0 还小，一旦抢到默认路由就会把"4G 链路"误判成 WiFi。
        if iface.startswith('enx'):
            continue
        if best_metric is None or m < best_metric:
            best, best_metric = iface, m
    return best


def link_state(iface: str) -> bool:
    try:
        with open('/sys/class/net/%s/operstate' % iface) as fh:
            return fh.read().strip() in ('up', 'unknown', 'dormant')
    except OSError:
        return False


def default_route_iface() -> str:
    """先读 /proc/net/route（零开销）；实在没有默认路由才退回 `ip route get`。"""
    try:
        with open('/proc/net/route') as fh:
            text = fh.read()
        ifaces = set()
        for line in text.splitlines()[1:]:
            parts = line.split()
            if len(parts) >= 8 and parts[1] == '00000000':
                ifaces.add(parts[0])
        up = {name: link_state(name) for name in ifaces}
        picked = route_from_proc_route(text, up)
        if picked:
            return picked
    except OSError:
        pass
    return _route_iface_by_ip()


def _route_iface_by_ip() -> str:
    for target in ('223.5.5.5', '1.1.1.1'):
        try:
            out = subprocess.run(['ip', 'route', 'get', target],
                                 capture_output=True, text=True, timeout=2,
                                 env={'LC_ALL': 'C', 'LANG': 'C',
                                      'PATH': '/usr/sbin:/usr/bin:/sbin:/bin'})
        except (OSError, subprocess.SubprocessError):
            continue
        parts = out.stdout.split()
        if 'dev' in parts:
            idx = parts.index('dev')
            if idx + 1 < len(parts):
                return parts[idx + 1]
    return ''


def decimate_step(acc: float, target: float, source: float) -> tuple[bool, float]:
    """按"每 N 帧取一帧"的累加器抽帧，返回 (这一帧要不要发, 新累加值)。

    ⚠️ **不能用"距上次发送 < 1/target 秒就丢"**：相机 30 fps、目标 20 fps 时，
    50 ms 门限会把节奏卡成"每 2 帧发 1 帧"→ 实际只有 **15 fps**
    （2026-11-25 实测：配置 20、实测 15.1）。累加器能保住平均 20 fps
    （发、丢、发 的 2/3 比例）。
    """
    if target <= 0 or source <= 0:
        return True, 0.0
    acc += target
    if acc >= source:
        return True, acc - source
    return False, acc


def split_jpeg_frames(buf: bytes) -> tuple[list[bytes], bytes]:
    """从 MJPEG 裸流里切出完整 JPEG（FFD8..FFD9），返回 (帧列表, 剩余半帧)。

    `v4l2-ctl --stream-to=-` 只吐负载、不给帧长，所以只能按标记切。
    实测 1280x720 连抓 70 帧 0 坏帧（2026-11-25）。调用方负责把剩余半帧
    拼到下一块数据前面。
    """
    frames: list[bytes] = []
    i = 0
    while True:
        start = buf.find(b'\xff\xd8', i)
        if start < 0:
            break
        end = buf.find(b'\xff\xd9', start + 2)
        if end < 0:
            return frames, buf[start:]      # 半帧：留着，下一块拼上继续切
        frames.append(buf[start:end + 2])
        i = end + 2
    rest = buf[i:]
    if not rest:
        return frames, b''
    # 尾部残留：只留最后 1 字节，防止 FFD8 被切在两块数据中间
    return frames, rest[-1:]


class WideUvcThread(threading.Thread):
    """120° 广角相机（UVC MJPG）取流，**零转码**：直接透传 JPEG 给 hub。

    `v4l2-ctl` 拉裸流 → 按 JPEG 边界切帧 → `hub.publish_jpeg()`（宽高需要缩小时
    才解码重编码）。断流/拔线自动重试，且每次重试都重新按 by-id 找设备。
    """

    MAX_BUFFER = 8 * 1024 * 1024    # 半帧缓冲上限，防把内存吃光

    def probe_parm(self, device: str) -> None:
        """开始取流前问一次相机"你到底按多少 fps 出"（排查掉帧用）。

        `--set-parm` 在不支持的帧率上会**只报错不生效**，而 stderr 以前被丢掉，
        现场就表现为"配了 30 实际只有 20"。这里把设置结果和相机回报都写进日志。
        """
        if self.cam_fps <= 0:
            return
        run = subprocess.run(['v4l2-ctl', '-d', device,
                              '--set-parm=%d' % self.cam_fps],
                             capture_output=True, text=True, timeout=5)
        err = (run.stderr or '').strip()
        try:
            q = subprocess.run(['v4l2-ctl', '-d', device, '--get-parm'],
                               capture_output=True, text=True, timeout=5)
            got = ' '.join(q.stdout.split())
        except (OSError, subprocess.SubprocessError):
            got = ''
        print('广角相机帧率设置 %d fps：%s%s' % (
            self.cam_fps, '失败（' + err.splitlines()[-1] + '）' if err else '成功',
            ('；相机回报 ' + got) if got else ''), flush=True)

    def decode_reduce(self) -> int:
        """源宽度的 1/2 仍 ≥ 目标宽度时，用半尺寸解码省一半 CPU。"""
        try:
            src_w = int(str(self.size).lower().split('x')[0])
        except (ValueError, IndexError):
            return 1
        if self.hub.width and self.hub.width * 2 <= src_w:
            return 2
        return 1

    def __init__(self, hub: 'FrameHub', device: str = '', size: str = '1280x720',
                 restart_delay: float = 3.0, cam_fps: int = 30,
                 target_fps: float = 0.0, cell_fps: float = 20.0,
                 link_check_s: float = 5.0) -> None:
        super().__init__(daemon=True)
        self.hub = hub
        self.fixed_device = device
        self.size = size
        self.restart_delay = float(restart_delay)
        #: 让 `v4l2-ctl` 明确设置相机帧率（不设的话部分 UVC 相机会停在 10 fps）
        self.cam_fps = int(cam_fps)
        #: 发送上限（0=不限，抽帧交给 FrameHub）；用累加器而不是时间阈值
        self.target_fps = float(target_fps)
        self.lan_fps = float(target_fps)
        #: 默认路由落到 4G 时自动降到这个帧率（上行只有 ~3.5 Mbps）
        self.cell_fps = float(cell_fps)
        self.link_check_s = float(link_check_s)
        self.link = 'wifi'
        self._last_link_check = 0.0
        self._send_acc = 0.0
        self._stop = threading.Event()
        self._cap_lock = threading.Lock()
        self._cap_times: list[float] = []
        self._last_idle_frame = 0.0

    def stop(self) -> None:
        self._stop.set()

    def device(self) -> str:
        if self.fixed_device and os.path.exists(self.fixed_device):
            return self.fixed_device
        return wide_device_path()

    def command(self) -> list:
        cmd = ['v4l2-ctl', '-d', self.device(),
               '--set-fmt-video=width=%s,height=%s,pixelformat=MJPG' % tuple(
                   self.size.lower().split('x'))]
        if self.cam_fps > 0:
            cmd.append('--set-parm=%d' % self.cam_fps)
        cmd += ['--stream-mmap', '--stream-count=0', '--stream-to=-']
        return cmd

    def refresh_link(self) -> None:
        """每隔几秒看一次"现在走的是 WiFi 还是 4G"，自动在两档帧率之间切。"""
        self._last_link_check = time.monotonic()
        iface = default_route_iface()
        fps = target_fps_for(iface, self.lan_fps, self.cell_fps)
        link = '4g' if iface and iface.startswith(CELLULAR_IFACE_PREFIXES) else 'wifi'
        if link != self.link:
            self.link = link
            print('广角链路切换：%s（默认路由 %s）→ 发送上限 %g fps' % (
                '4G' if link == '4g' else 'WiFi/有线', iface or '未知', fps),
                flush=True)
        self.target_fps = fps

    def effective_source(self) -> float:
        """抽帧用的"源帧率"必须是**实测**的。

        ⚠️ 相机宣称 30 fps、实际只出 20 fps（本机实测 19.4，`--set-parm` 也改不了）。
        若拿 30 当分母去抽 20，会再抽掉 1/3 → 实测只剩 **13.3 fps**
        （2026-11-25 现场：配 20 出 13.4，就是这条）。
        """
        fps = self.capture_fps()
        return fps if fps > 1.0 else float(self.cam_fps or 30)

    def capture_fps(self) -> float:
        """相机**实际喂过来**的帧率（不受发送限速影响，排查掉帧用）。"""
        with self._cap_lock:
            times = self._cap_times
            if len(times) < 2:
                return 0.0
            span = times[-1] - times[0]
            return round((len(times) - 1) / span, 1) if span > 0.5 else 0.0

    def _note_capture(self, now: float) -> None:
        with self._cap_lock:
            self._cap_times.append(now)
            if len(self._cap_times) > 240:
                del self._cap_times[:-240]

    def run(self) -> None:
        device = ''
        while not self._stop.is_set():
            device = self.device()
            if not device:
                print('广角相机没找到（等插上/重枚举）…', flush=True)
                self._stop.wait(3.0)
                continue
            proc = None
            try:
                self.probe_parm(device)
                print('广角相机取流：%s %s' % (device, self.size), flush=True)
                proc = subprocess.Popen(self.command(), stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, bufsize=0)
                errq: list[bytes] = []
                threading.Thread(target=lambda: errq.extend(proc.stderr), daemon=True).start()
                buf = b''
                while not self._stop.is_set():
                    chunk = proc.stdout.read(65536)
                    if not chunk:
                        break
                    buf += chunk
                    frames, buf = split_jpeg_frames(buf)
                    if time.monotonic() - self._last_link_check >= self.link_check_s:
                        self.refresh_link()
                    for jpeg in frames:
                        now = time.monotonic()
                        self._note_capture(now)
                        if self.hub.has_clients():
                            send, self._send_acc = decimate_step(
                                self._send_acc, self.target_fps,
                                self.effective_source())
                            if send:
                                # 有人看：缩到 480 宽（720p 源用半尺寸解码）
                                self.hub.publish_jpeg(
                                    jpeg, reduce=self.decode_reduce())
                        elif now - self._last_idle_frame > 1.0:
                            # 没人看：只留一张原始帧给 /snapshot.jpg，不解码不编码
                            self._last_idle_frame = now
                            self.hub.publish_jpeg(jpeg, resample=False)
                    if len(buf) > self.MAX_BUFFER:
                        buf = b''
            except (OSError, ValueError) as exc:
                print('广角相机异常：%s' % exc, flush=True)
            finally:
                if proc is not None and proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.kill()
            if errq:
                msg = b''.join(errq).decode('utf-8', 'replace').strip()
                if msg:
                    print('广角 v4l2-ctl 输出：%s' % msg.splitlines()[-3:], flush=True)
            if not self._stop.is_set():
                print('广角相机断流，%.0f s 后重试' % self.restart_delay, flush=True)
                self._stop.wait(self.restart_delay)


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
    ap.add_argument('--wide-device', default='',
                    help='120° 广角相机设备；空=按 /dev/v4l/by-id 里的 5MP_USB_Camera 自动找')
    ap.add_argument('--no-wide', action='store_true', help='完全不启用广角相机这一路')
    ap.add_argument('--wide-size', default='1280x720', help='广角相机采集尺寸')
    ap.add_argument('--wide-width', type=int, default=480,
                    help='广角画面送到手机前的宽度（0=原样透传 1280 宽，4G 会吃不消）')
    ap.add_argument('--wide-fps', type=float, default=20.0,
                    help='广角画面发送上限帧率（和主摄 ~19 fps 对齐）')
    ap.add_argument('--wide-quality', type=int, default=60,
                    help='广角重编码画质（越小越省 4G 流量）')
    ap.add_argument('--wide-fps-4g', type=float, default=20.0,
                    help='默认路由变成 4G 时自动降到这个帧率（4G 上行 ~3.5 Mbps）')
    ap.add_argument('--wide-cam-fps', type=int, default=30,
                    help='让相机按这个帧率出流（0=不设置，用相机默认）')
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

    # 第二路：120° 广角相机（UVC MJPG，零转码）。默认自动找 by-id，找不到也不影响主画面
    hub_wide = None
    wide_device = ''
    if not args.no_wide:
        hub_wide = FrameHub(quality=args.wide_quality, fps=0,
                            width=args.wide_width)
        wide = WideUvcThread(hub_wide, args.wide_device, args.wide_size,
                             cam_fps=args.wide_cam_fps,
                             target_fps=args.wide_fps,
                             cell_fps=args.wide_fps_4g)
        wide_device = wide.device()
        wide.start()
        print('广角相机：%s（%s，发送 %g fps / 4G 时 %g fps / 相机 %d fps，%d 宽 q%d）' % (
            wide_device or '未找到（会持续重试）', args.wide_size,
            args.wide_fps, args.wide_fps_4g, args.wide_cam_fps,
            args.wide_width, args.wide_quality), flush=True)

    topic = args.v4l2 or ('test-pattern' if args.test_pattern else args.topic)
    app = build_app(hub, topic, uvc, hub_wide, wide_device, wide if hub_wide else None)
    print(f'MJPEG 服务: http://{args.host}:{args.port}/video  '
          f'(topic={app["topic"]}, {args.fps:g} fps, q={args.quality})', flush=True)
    try:
        web.run_app(app, host=args.host, port=args.port, print=None)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
