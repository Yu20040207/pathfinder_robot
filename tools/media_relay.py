#!/usr/bin/env python3
"""车端媒体中继：把本机 MJPEG 画面主动推到云端，供**异地**手机观看。

为什么需要它
------------
手机在办公室 WiFi 里时，画面走 `GET /api/devices/<id>/media` 下发的
`http://<lan_ip>:8081/video` —— 那是**局域网地址**，换成 4G 或别的 WiFi
就不可达（画面黑、提示「摄像头连不上」）。

控制链路本来就是云的（车主动连云端 MQTT `8.163.69.109:1883`），所以只要把
画面也做成"车主动拨出"，异地就能看，而且车在运营商 CGNAT 后面也没关系。

协议（一条 WebSocket，双向）
---------------------------
    云端 → 车（文本 JSON）
        {"cmd": "play", "ch": 0, "profile": "low"}   ch 0=主摄 1=广角
        {"cmd": "stop", "ch": 0}
        {"cmd": "ping"}
    车 → 云端
        文本 {"type": "hello", "device": "car-001", "cams": [0, 1]}
        文本 {"type": "error", "ch": 0, "msg": "..."}
        文本 {"type": "pong"}
        **二进制** = 1 字节通道号 + 完整 JPEG（`bytes([ch]) + jpeg`）

设计要点
--------
* **最新的帧才是好的帧**：每个通道只留 1 帧（`latest[ch]`），云端发得慢就丢旧的，
  绝不排长队 —— 排队等于给画面故意加延迟。
* **本地永远拉低码率档**（`?profile=low`，320 宽 12 fps ≈ 1.0 Mbps），
  因为云端出网要按客户数算钱；局域网直连走的是另一条路，不受影响。
* 断线重连用指数退避（1→30 s，±20% 抖动），云端重连后会自动补发 play。
* 本进程只做"搬运"，不解码不转码（转码在 `robot_mjpeg_server.py` 的低码率分身里做）。

手动运行（调试用）
------------------
    python3 tools/media_relay.py --device car-001 --token xxx \
        --cloud ws://127.0.0.1:8444/ws/media --local http://127.0.0.1:8081
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import random
import signal
import sys
import time
from typing import Dict, Optional

SOI = b'\xff\xd8'          # JPEG 开始
EOI = b'\xff\xd9'          # JPEG 结束
CH_MAIN = 0
CH_WIDE = 1


# ---------------------------------------------------------------- 纯逻辑

def jpeg_frames(buf: bytearray) -> list[bytes]:
    """从累积缓冲区里切出完整的 JPEG 帧，并把已消费的部分删掉。

    服务端发的是 multipart（`--fwrframe` 边界 + 头 + JPEG 体），这里不去解析
    multipart 头，直接按 JPEG 的 `FFD8 ... FFD9` 切 —— 更抗协议细节变化，
    而且坏帧只影响一帧。
    """
    out: list[bytes] = []
    while True:
        start = buf.find(SOI)
        if start < 0:
            # 没有帧头：留最后 1 字节（可能是被切开的 FFD8 的前半）
            if len(buf) > 1:
                del buf[:len(buf) - 1]
            return out
        end = buf.find(EOI, start + 2)
        if end < 0:
            if start:
                del buf[:start]
            return out
        out.append(bytes(buf[start:end + 2]))
        del buf[:end + 2]


def control_action(msg: dict) -> tuple[str, int]:
    """把云端控制消息翻译成 (动作, 通道)。非法消息 → ('ignore', -1)。"""
    cmd = str(msg.get('cmd') or '')
    if cmd == 'ping':
        return 'pong', -1
    if cmd not in ('play', 'stop'):
        return 'ignore', -1
    try:
        ch = int(msg.get('ch', CH_MAIN))
    except (TypeError, ValueError):
        return 'ignore', -1
    if ch not in (CH_MAIN, CH_WIDE):
        return 'ignore', -1
    return cmd, ch


def backoff_delay(attempt: int, base: float = 1.0, cap: float = 30.0) -> float:
    """指数退避 + ±20% 抖动（多台车同时掉线时别一起回来）。"""
    raw = min(cap, base * (2 ** max(0, attempt - 1)))
    return raw * random.uniform(0.8, 1.2)


def channel_url(local_base: str, ch: int, profile: str = 'low') -> str:
    cam = 'wide' if ch == CH_WIDE else 'main'
    base = local_base.rstrip('/')
    return f'{base}/video?cam={cam}&profile={profile}'


def relay_url(cloud_base: str, device: str, token: str) -> str:
    base = cloud_base.rstrip('/')
    return f'{base}/{device}?token={token}'


# ---------------------------------------------------------------- 搬运

class MediaRelay:
    def __init__(self, cloud: str, device: str, token: str,
                 local: str = 'http://127.0.0.1:8081',
                 profile: str = 'low', verbose: bool = True) -> None:
        self.cloud = cloud
        self.device = device
        self.token = token
        self.local = local
        self.profile = profile
        self.verbose = verbose
        self.latest: Dict[int, bytes] = {}
        self.pending = asyncio.Event()
        self.pullers: Dict[int, asyncio.Task] = {}
        self._stop = asyncio.Event()
        self.frames_sent = 0
        self.bytes_sent = 0
        self.started = time.monotonic()

    # ---- 日志 ----
    def log(self, text: str) -> None:
        if self.verbose:
            print(f'[relay {time.strftime("%H:%M:%S")}] {text}', flush=True)

    # ---- 通道拉流 ----
    async def _puller(self, session, ch: int) -> None:
        url = channel_url(self.local, ch, self.profile)
        while not self._stop.is_set():
            try:
                async with session.get(url, timeout=None) as resp:
                    if resp.status != 200:
                        raise RuntimeError(f'HTTP {resp.status}')
                    self.log(f'本地流已连上 ch{ch}（{url}）')
                    buf = bytearray()
                    async for chunk in resp.content.iter_any():
                        buf += chunk
                        for frame in jpeg_frames(buf):
                            self.latest[ch] = frame      # 只留最新一帧
                            self.pending.set()
                        if self._stop.is_set():
                            break
            except asyncio.CancelledError:
                raise
            except Exception as exc:              # noqa: BLE001
                self.log(f'本地流断开 ch{ch}：{exc}（2 s 后重试）')
            await asyncio.sleep(2.0)

    def start_puller(self, session, ch: int) -> None:
        task = self.pullers.get(ch)
        if task is not None and not task.done():
            return
        self.pullers[ch] = asyncio.create_task(self._puller(session, ch))
        self.log(f'开始推流 ch{ch}')

    def stop_puller(self, ch: int) -> None:
        task = self.pullers.pop(ch, None)
        if task is not None:
            task.cancel()
        self.latest.pop(ch, None)
        self.log(f'停止推流 ch{ch}')

    # ---- 上行发送 ----
    async def _sender(self, ws) -> None:
        while not self._stop.is_set():
            await self.pending.wait()
            self.pending.clear()
            for ch in (CH_MAIN, CH_WIDE):
                frame = self.latest.pop(ch, None)
                if not frame:
                    continue
                await ws.send_bytes(bytes([ch]) + frame)
                self.frames_sent += 1
                self.bytes_sent += len(frame)

    # ---- 单次连接 ----
    async def _session_once(self, url: str) -> None:
        import aiohttp
        headers = {'User-Agent': 'fwr-media-relay/1'}
        async with aiohttp.ClientSession() as http, \
                http.ws_connect(url, heartbeat=20.0, max_msg_size=8 * 1024 * 1024,
                                headers=headers) as ws:
            self.log(f'已连上云端 {url.split("?")[0]}')
            await ws.send_str(json.dumps({'type': 'hello', 'device': self.device,
                                          'cams': [CH_MAIN, CH_WIDE]}))
            sender = asyncio.create_task(self._sender(ws))
            try:
                async for msg in ws:
                    if msg.type == aiohttp.WSMsgType.TEXT:
                        try:
                            data = json.loads(msg.data)
                        except ValueError:
                            continue
                        action, ch = control_action(data)
                        if action == 'pong':
                            await ws.send_str(json.dumps({'type': 'pong'}))
                        elif action == 'play':
                            self.start_puller(http, ch)
                        elif action == 'stop':
                            self.stop_puller(ch)
                    elif msg.type in (aiohttp.WSMsgType.CLOSED,
                                      aiohttp.WSMsgType.ERROR):
                        break
            finally:
                sender.cancel()
                for ch in list(self.pullers):
                    self.stop_puller(ch)

    # ---- 主循环 ----
    async def run(self) -> None:
        url = relay_url(self.cloud, self.device, self.token)
        attempt = 0
        while not self._stop.is_set():
            attempt += 1
            try:
                await self._session_once(url)
                self.log('云端连接关闭')
                attempt = 0
            except asyncio.CancelledError:
                raise
            except Exception as exc:              # noqa: BLE001
                self.log(f'连不上云端：{exc}')
            if self._stop.is_set():
                break
            delay = backoff_delay(max(1, attempt))
            self.log(f'{delay:.1f} s 后重连')
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=delay)
            except asyncio.TimeoutError:
                pass

    def request_stop(self) -> None:
        self._stop.set()
        self.pending.set()


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description='车端媒体中继（异地看画面用）')
    ap.add_argument('--cloud', default=os.environ.get('MEDIA_RELAY_CLOUD',
                                                    'ws://jccontrol.xyz/ws/media'))
    ap.add_argument('--device', default=os.environ.get('MEDIA_RELAY_DEVICE',
                                                      'car-001'))
    ap.add_argument('--token', default=os.environ.get('MEDIA_RELAY_TOKEN', ''))
    ap.add_argument('--local', default=os.environ.get('MEDIA_RELAY_LOCAL',
                                                     'http://127.0.0.1:8081'))
    ap.add_argument('--profile', default='low', choices=['low', 'full'],
                    help='推哪一档画质（low=320 宽 12 fps，异地用）')
    ap.add_argument('--quiet', action='store_true')
    return ap


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.token:
        print('缺少 --token（或环境变量 MEDIA_RELAY_TOKEN）', file=sys.stderr)
        return 2
    relay = MediaRelay(args.cloud, args.device, args.token, args.local,
                       args.profile, verbose=not args.quiet)
    loop = asyncio.new_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, relay.request_stop)
        except NotImplementedError:              # pragma: no cover
            pass
    try:
        loop.run_until_complete(relay.run())
    finally:
        loop.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
