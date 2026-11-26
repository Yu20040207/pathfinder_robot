"""端到端自测：假本地 MJPEG + 真 jc-api + 真车端中继 → 拉流验证。

本机就能跑（不需要机器人），用来证明：
  1) 车端中继能拨上云端并接受 play/stop；
  2) 云端 `/api/devices/<id>/stream` 输出浏览器认的 multipart MJPEG；
  3) 逐帧透传不缓冲（延迟 ≈ 一帧时间，不是几秒）；
  4) 码率/帧率符合异地档（320 宽 12 fps ≈ 1 Mbps）。
"""
from __future__ import annotations

import asyncio
import json
import os
import socket
import subprocess
import sys
import time
import uuid

import httpx
from aiohttp import web

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOKEN = 'e2e-media-token'
DEV = 'car-001'


def free_port() -> int:
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


async def fake_mjpeg_server(port: int, fps: int = 15, size: tuple = (480, 360)):
    """模拟车端 `robot_mjpeg_server`：multipart 里塞带时间戳的 JPEG。"""
    import cv2
    import numpy as np

    async def video(request: web.Request):
        # 和车端一样按 `?profile=` 转码：eco = 320 宽 q55 / 10 fps（弱网档约 0.5 Mbps）
        eco = (request.query.get('profile') == 'eco')
        want_w, quality, use_fps = (320, 55, 10) if eco else (480, 60, fps)
        resp = web.StreamResponse(headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=fwrframe',
            'Cache-Control': 'no-store'})
        await resp.prepare(request)
        period = 1.0 / fps
        while True:
            # 帧要像真机那样"有细节但可压缩"（480 宽 q60 约 15~18 KB）。
            # 直接用纯噪声会压到 80 KB，e2e 的码率闸门会误判失败。
            rng = np.random.default_rng(int(time.time() * 1000) % 65536)
            img = cv2.GaussianBlur(
                rng.integers(0, 255, (size[1], size[0], 3), dtype=np.uint8),
                (11, 11), 0)
            if want_w != size[0]:
                img = cv2.resize(img, (want_w, int(size[1] * want_w / size[0])),
                                 interpolation=cv2.INTER_AREA)
            cv2.putText(img, f'{time.time():.3f}', (6, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
            data = buf.tobytes()
            # COM 段（FFFE）里放"生成时刻"，用来量端到端延迟（生产帧没有这一段）
            stamp = ('%.6f' % time.time()).encode()
            com = b'\xff\xfe' + (len(stamp) + 2).to_bytes(2, 'big') + stamp
            data = data[:2] + com + data[2:]
            try:
                await resp.write(b'--fwrframe\r\nContent-Type: image/jpeg\r\n'
                                 b'Content-Length: ' + str(len(data)).encode()
                                 + b'\r\n\r\n' + data + b'\r\n')
            except Exception:            # 客户端断开：假服务直接收工
                break
            await asyncio.sleep(1.0 / use_fps)

    async def healthz(request: web.Request):
        # 跟随模式的面板接口（/boxes /select /follow /healthz）经中继转发，
        # e2e 用这个端点验证"云端 → 中继 → 本机 8081"这条回程链路。
        return web.json_response({'ok': True, 'fake': True, 'boxes': 0})

    app = web.Application()
    app.router.add_get('/video', video)
    app.router.add_get('/healthz', healthz)
    runner = web.AppRunner(app, access_log=None)
    await runner.setup()
    site = web.TCPSite(runner, '127.0.0.1', port)
    await site.start()
    return runner


async def main(read_kbps: float = 0.0, profile: str = 'low') -> int:
    cloud_port = free_port()
    local_port = free_port()
    db = f'/tmp/e2e-{uuid.uuid4().hex}.db'
    env = dict(os.environ, JC_DB=db, JC_SECRET='e2e-secret-0123456789abcdef0123456789ab',
               JC_MEDIA_TOKEN=TOKEN, JC_BIND_CODE='123456', JC_MQTT_HOST='127.0.0.1',
               JC_MEDIA_STREAM_TTL='120')
    fake = await fake_mjpeg_server(local_port)
    api = subprocess.Popen(
        [sys.executable, '-m', 'uvicorn', 'app:app', '--host', '127.0.0.1',
         '--port', str(cloud_port), '--log-level', 'warning'],
        cwd=os.path.join(ROOT, 'cloud', 'api'), env=env)
    relay = None
    try:
        async with httpx.AsyncClient(base_url=f'http://127.0.0.1:{cloud_port}',
                                     timeout=10) as http:
            for _ in range(50):
                try:
                    if (await http.get('/api/health')).status_code == 200:
                        break
                except httpx.HTTPError:
                    await asyncio.sleep(0.2)
            await http.post('/api/register', json={'email': 'e2e@x.y', 'password': 'pw1234'})
            tok = (await http.post('/api/login', json={'email': 'e2e@x.y',
                                                       'password': 'pw1234'})).json()['access_token']
            hdr = {'Authorization': f'Bearer {tok}'}
            await http.post('/api/devices/bind', headers=hdr,
                            json={'device_id': DEV, 'code': '123456'})
            media = (await http.get(f'/api/devices/{DEV}/media', headers=hdr)).json()
            print('异地入口 :', json.dumps(media['remote'], ensure_ascii=False))

            relay = subprocess.Popen(
                [sys.executable, os.path.join(ROOT, 'tools', 'media_relay.py'),
                 '--cloud', f'ws://127.0.0.1:{cloud_port}/ws/media', '--device', DEV,
                 '--token', TOKEN, '--local', f'http://127.0.0.1:{local_port}'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            for _ in range(40):
                st = (await http.get('/api/media/health')).json()
                if st['devices'].get(DEV, {}).get('online'):
                    break
                await asyncio.sleep(0.25)
            print('车端在线 :', (await http.get('/api/media/health')).json()['devices'].get(DEV))

            url = (f"http://127.0.0.1:{cloud_port}/api/devices/{DEV}/stream"
                   f"?cam=main&profile={profile}&t={media['remote']['token']}")
            t0 = time.time()
            total = 0
            frames = 0
            lat = []
            # --read-kbps 模拟手机下行带宽：读取端按这条速率消费，堵住时看延迟怎么涨
            budget = 0.0
            pace_start = time.time()
            async with httpx.AsyncClient(timeout=None) as raw:
                async with raw.stream('GET', url) as resp:
                    print('拉流状态 :', resp.status_code, resp.headers.get('content-type'))
                    buf = bytearray()
                    async for chunk in resp.aiter_bytes():
                        buf += chunk
                        if read_kbps > 0:      # 限速：允许的时间额度用完了就等
                            budget += len(chunk) * 8 / 1000.0
                            ahead = budget - (time.time() - pace_start) * read_kbps
                            if ahead > 0:
                                await asyncio.sleep(ahead / read_kbps)
                        while True:
                            e = buf.find(b'\xff\xd9')
                            if e < 0:
                                break
                            s = buf.find(b'\xff\xd8')
                            if s < 0:
                                break
                            frame = bytes(buf[s:e + 2])
                            del buf[:e + 2]
                            total += len(frame)
                            frames += 1
                            at = frame.find(b'\xff\xfe')
                            if at >= 0 and at + 4 <= len(frame):
                                n = int.from_bytes(frame[at + 2:at + 4], 'big')
                                try:
                                    gen = float(frame[at + 4:at + 2 + n])
                                    lat.append((time.time() - gen) * 1000.0)
                                except ValueError:
                                    pass
                            if frames >= 40:
                                break
                        if frames >= 40:
                            break
            span = time.time() - t0
            if lat:
                lat.sort()
                mid = lat[len(lat) // 2]
                p90 = lat[min(len(lat) - 1, int(len(lat) * 0.9))]
                print(f'中继延迟 : 中位 {mid:.0f} ms / P90 {p90:.0f} ms'
                      f'（{len(lat)} 帧，生成→到达；本机回环）')
            print(f'收到 {frames} 帧 / {span:.2f} s → {frames / span:.1f} fps, '
                  f'{total / max(1, frames) / 1024:.1f} KB/帧, '
                  f'{total * 8 / span / 1e6:.2f} Mbps')
            final = (await http.get('/api/media/health')).json()
            print('结束统计 :', json.dumps(final['devices'].get(DEV), ensure_ascii=False))
            # 面板接口代理（跟随模式异地要用）：必须能透到本机 8081
            proxy = await http.get(f'/api/devices/{DEV}/vision/healthz', headers=hdr)
            print('面板代理 :', proxy.status_code, proxy.text[:80])
            proxy_ok = (proxy.status_code == 200 and
                        bool((proxy.json() or {}).get('ok')))
            ok = proxy_ok and frames >= 12 and 0.2 <= total * 8 / span / 1e6 <= 4.0
            print('结果     :', 'PASS' if ok else 'FAIL')
            return 0 if ok else 1
    finally:
        if relay is not None:
            relay.terminate()
            try:
                out = relay.communicate(timeout=5)[0]
                print('--- 中继日志尾部 ---')
                print('\n'.join(out.strip().splitlines()[-6:]))
            except Exception:
                relay.kill()
        api.terminate()
        try:
            api.wait(timeout=5)
        except Exception:
            api.kill()
        await fake.cleanup()


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--read-kbps', type=float, default=0.0,
                    help='读取端限速（模拟 4G 下行），0=不限')
    ap.add_argument('--profile', default='low', choices=['low', 'eco'])
    a = ap.parse_args()
    raise SystemExit(asyncio.run(main(a.read_kbps, a.profile)))
