"""jccontrol 云端 API：账号 / 设备 / 控制转发。

架构（阶段 1）：
    App/网页 ──HTTP+WS──► 本服务 ──MQTT(本机 EMQX)──► 小车 mqtt_bridge
                          └─ SQLite 用户/设备

端点：
    POST /api/register              {email, password}
    POST /api/login                 {email, password} -> {access_token}
    POST /api/devices/bind          {device_id, code}      需登录
    GET  /api/devices               我的设备（含最近状态）
    GET  /api/devices/{id}/state    最近状态
    GET  /api/devices/{id}/media    摄像头播放地址（{type, url, snapshot, ttl}）
    WS   /ws/control/{device_id}?token=...   实时控制 + 状态推送
    GET  /api/health

环境变量：
    JC_DB          SQLite 路径（默认 /opt/jc/data/jc.db）
    JC_SECRET      JWT 密钥（生产必须改）
    JC_BIND_CODE   设备绑定码（默认 123456，试用期）
    JC_MQTT_HOST/PORT   EMQX（默认 127.0.0.1:1883）
    JC_MEDIA_HOSTS     摄像头地址表 JSON，如 {"car-001": "192.168.0.20"}
    JC_MEDIA_PORT      机器人 MJPEG 服务端口（默认 8081）
    JC_MEDIA_TOKEN     车端媒体中继的共享密钥（不配则中继端点关闭）
    JC_MEDIA_MAX_CLIENTS  同时中继的上限（默认 6，按云端出网带宽定）
"""
import asyncio
import hashlib
import json
import os
import sqlite3
import time
import itertools
from typing import Dict, Optional, Set

import jwt
from fastapi import (Depends, FastAPI, Header, HTTPException, Request,
                     WebSocket, WebSocketDisconnect)
from fastapi.responses import Response, StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

try:
    import paho.mqtt.client as mqtt
except ImportError:                      # pragma: no cover
    mqtt = None

DB_PATH = os.environ.get('JC_DB', '/opt/jc/data/jc.db')
SECRET = os.environ.get('JC_SECRET', 'dev-secret-change-me')
BIND_CODE = os.environ.get('JC_BIND_CODE', '123456')
MQTT_HOST = os.environ.get('JC_MQTT_HOST', '127.0.0.1')
MQTT_PORT = int(os.environ.get('JC_MQTT_PORT', '1883'))
TOKEN_TTL = int(os.environ.get('JC_TOKEN_TTL', '86400'))
MAX_LINEAR = float(os.environ.get('JC_MAX_LINEAR', '0.15'))
MAX_ANGULAR = float(os.environ.get('JC_MAX_ANGULAR', '0.6'))

app = FastAPI(title='jccontrol')
app.add_middleware(CORSMiddleware, allow_origins=['*'],
                   allow_methods=['*'], allow_headers=['*'])

# device_id -> 最近状态
STATE_CACHE: Dict[str, dict] = {}
MAP_CACHE: Dict[str, dict] = {}


def _media_hosts() -> Dict[str, str]:
    """机器人 LAN 地址表：JC_MEDIA_HOSTS 的 JSON，解析失败就当没配。"""
    raw = os.environ.get('JC_MEDIA_HOSTS', '').strip()
    if not raw:
        return {}
    try:
        data = json.loads(raw)
    except Exception:
        return {}
    return {str(k): str(v) for k, v in data.items()} if isinstance(data, dict) else {}


MEDIA_PORT = os.environ.get('JC_MEDIA_PORT', '8081')
# 车端媒体中继（异地看画面）：车主动拨出，云端按 device_id 路由
MEDIA_TOKEN = os.environ.get('JC_MEDIA_TOKEN', '')
MEDIA_MAX_CLIENTS = int(os.environ.get('JC_MEDIA_MAX_CLIENTS', '6'))
MEDIA_PROXY_TIMEOUT = 2.5
_PROXY_SEQ = itertools.count(1)

MEDIA_STREAM_TTL = int(os.environ.get('JC_MEDIA_STREAM_TTL', '180'))

# device_id -> {'ws': WebSocket|None, 'subs': {slot: set[Queue]}, 'frames': int, 'bytes': int}
#: 槽位 = 通道(0 主摄/1 广角) + 画质档偏移(0 高清 low / 2 弱网 eco)，与车端 `media_relay` 一致
MEDIA_PROFILE_OFFSET = {'low': 0, 'eco': 2}


def media_slot(cam: str, profile: str) -> int:
    """(摄像头, 画质档) → 0..3 槽位号。"""
    return (1 if cam == 'wide' else 0) + MEDIA_PROFILE_OFFSET.get(profile, 0)
MEDIA_HUBS: Dict[str, dict] = {}
# device_id -> 活跃 WebSocket 集合
CONNECTIONS: Dict[str, Set[WebSocket]] = {}
LOOP: Optional[asyncio.AbstractEventLoop] = None
SEQ = 0


def db() -> sqlite3.Connection:
    conn = sqlite3.connect(DB_PATH, timeout=5)
    conn.row_factory = sqlite3.Row
    return conn


def init_db() -> None:
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    with db() as conn:
        conn.executescript('''
            CREATE TABLE IF NOT EXISTS users (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                email TEXT UNIQUE NOT NULL,
                pw_hash TEXT NOT NULL,
                created REAL NOT NULL
            );
            CREATE TABLE IF NOT EXISTS devices (
                device_id TEXT PRIMARY KEY,
                owner_id INTEGER,
                name TEXT,
                updated REAL
            );
        ''')


def hash_pw(password: str, salt: bytes = b'jc-salt') -> str:
    digest = hashlib.pbkdf2_hmac('sha256', password.encode(), salt, 120_000)
    return digest.hex()


def make_token(user_id: int) -> str:
    return jwt.encode({'sub': str(user_id), 'exp': time.time() + TOKEN_TTL},
                      SECRET, algorithm='HS256')


def user_from_token(token: str) -> int:
    try:
        payload = jwt.decode(token, SECRET, algorithms=['HS256'])
        return int(payload['sub'])
    except Exception:                       # noqa: BLE001
        raise HTTPException(status_code=401, detail='token 无效')


def current_user(authorization: str = Header(default='')) -> int:
    if not authorization.startswith('Bearer '):
        raise HTTPException(status_code=401, detail='缺少 token')
    return user_from_token(authorization[7:])


class Credentials(BaseModel):
    email: str
    password: str


class PasswordRequest(BaseModel):
    old_password: str
    new_password: str


class PowerRequest(BaseModel):
    action: str


class BindRequest(BaseModel):
    device_id: str
    code: str
    name: Optional[str] = None


@app.on_event('startup')
async def on_startup():
    global LOOP
    LOOP = asyncio.get_running_loop()
    init_db()
    start_mqtt()


@app.get('/api/health')
def health():
    return {'ok': True, 'mqtt': mqtt_connected(), 'ts': time.time()}


@app.post('/api/register')
def register(cred: Credentials):
    email = cred.email.strip().lower()
    if len(cred.password) < 6:
        raise HTTPException(400, '密码至少 6 位')
    try:
        with db() as conn:
            conn.execute('INSERT INTO users(email, pw_hash, created) VALUES(?,?,?)',
                         (email, hash_pw(cred.password), time.time()))
    except sqlite3.IntegrityError:
        raise HTTPException(409, '邮箱已注册')
    return {'ok': True}


@app.post('/api/login')
def login(cred: Credentials):
    with db() as conn:
        row = conn.execute('SELECT id, pw_hash FROM users WHERE email=?',
                           (cred.email.strip().lower(),)).fetchone()
    if row is None or not hmac_compare(row['pw_hash'], hash_pw(cred.password)):
        raise HTTPException(401, '邮箱或密码错误')
    return {'access_token': make_token(row['id']), 'token_type': 'bearer'}


def hmac_compare(a: str, b: str) -> bool:
    import hmac
    return hmac.compare_digest(a, b)


@app.post('/api/devices/bind')
def bind_device(req: BindRequest, user_id: int = Depends(current_user)):
    if req.code != BIND_CODE:
        raise HTTPException(403, '绑定码错误')
    # 绑定码就是设备凭证：校验通过后允许转移归属（试用期简化逻辑）
    with db() as conn:
        conn.execute('INSERT INTO devices(device_id, owner_id, name, updated) '
                     'VALUES(?,?,?,?) ON CONFLICT(device_id) DO UPDATE SET '
                     'owner_id=excluded.owner_id, name=COALESCE(excluded.name, name)',
                     (req.device_id, user_id, req.name, time.time()))
    return {'ok': True, 'device_id': req.device_id}


@app.get('/api/devices')
def list_devices(user_id: int = Depends(current_user)):
    with db() as conn:
        rows = conn.execute('SELECT device_id, name FROM devices WHERE owner_id=?',
                            (user_id,)).fetchall()
    return {'devices': [
        {'device_id': r['device_id'], 'name': r['name'],
         'state': STATE_CACHE.get(r['device_id'])} for r in rows]}


@app.get('/api/devices/{device_id}/state')
def device_state(device_id: str, user_id: int = Depends(current_user)):
    return STATE_CACHE.get(device_id, {})


@app.get('/api/devices/{device_id}/media')
def device_media(device_id: str, user_id: int = Depends(current_user)):
    """摄像头播放地址。

    页面只认 {type, url}：现在是 mjpeg（办公室 WiFi 直连机器人），
    以后接 4G 改成中继时只换这里的返回，页面不用改。

    地址来源优先级：JC_MEDIA_HOSTS[device_id] → 机器人上报的 lan_ip。
    """
    hosts = _media_hosts()
    host = hosts.get(device_id)
    if not host:
        state = STATE_CACHE.get(device_id) or {}
        host = str(state.get('lan_ip') or '')
    # 车在 4G / 只有云端中继时没有可用的局域网地址：不再 404，
    # 返回空 url + remote 让页面直接走中继（否则异地机器人连播放地址都拿不到）。
    base = ''
    if host:
        host = host.replace('http://', '').replace('/', '')
        port = MEDIA_PORT
        if ':' in host:                  # 允许 "192.168.0.20:8081" 覆盖端口
            host, port = host.split(':', 1)
        base = f'http://{host}:{port}'
    return {
        'type': 'mjpeg',
        'url': f'{base}/video' if base else '',
        'snapshot': f'{base}/snapshot.jpg' if base else '',
        'health': f'{base}/healthz' if base else '',
        'page': f'{base}/' if base else '',   # 自检页（HTTPS 页面被拦时用）
        'ttl': 60,
        # 异地（不在同一 WiFi）走云端中继：同源、同端口 80，页面不用管 CORS
        'remote': {
            'url': f'/api/devices/{device_id}/stream',
            'token': make_stream_token(device_id, user_id),
            'online': bool((MEDIA_HUBS.get(device_id) or {}).get('ws')),
        },
    }


# ---------------------------------------------------------------- 异地看画面
# 车端 `tools/media_relay.py` 主动拨出到 `/ws/media/<id>`，云端按设备路由给手机。
# 手机拿到的仍是同源 MJPEG（`<img src=/api/devices/<id>/stream?...>`），
# 所以在任何网络下都能看，而且不需要 CORS / 不需要 HTTPS 混合内容例外。


def make_stream_token(device_id: str, user_id: int) -> str:
    """短时观看令牌：只能看这一台设备，`MEDIA_STREAM_TTL` 秒后失效。"""
    return jwt.encode({'sub': str(user_id), 'dev': device_id, 'kind': 'stream',
                       'exp': time.time() + MEDIA_STREAM_TTL},
                      SECRET, algorithm='HS256')


def _stream_user(device_id: str, token: str, authorization: str) -> int:
    """`<img>` 带不了自定义头，所以令牌也可以走 `?t=`（只认观看令牌，不认登录令牌）。"""
    if token:
        try:
            payload = jwt.decode(token, SECRET, algorithms=['HS256'])
        except Exception:                    # noqa: BLE001
            raise HTTPException(401, '观看令牌无效或已过期')
        if payload.get('kind') != 'stream' or payload.get('dev') != device_id:
            raise HTTPException(401, '观看令牌与设备不匹配')
        return int(payload['sub'])
    return current_user(authorization=authorization)


def _require_owner(user_id: int, device_id: str) -> None:
    with db() as conn:
        row = conn.execute('SELECT owner_id FROM devices WHERE device_id=?',
                           (device_id,)).fetchone()
    if row is None or row['owner_id'] != user_id:
        raise HTTPException(403, '设备不属于当前账号')


def _media_hub(device_id: str) -> dict:
    hub = MEDIA_HUBS.get(device_id)
    if hub is None:
        hub = {'ws': None, 'subs': {}, 'frames': 0, 'bytes': 0, 'proxy': {}}
        MEDIA_HUBS[device_id] = hub
    return hub


def active_streams() -> int:
    return sum(len(queues) for hub in MEDIA_HUBS.values()
               for queues in hub['subs'].values())


"""多机代理：跟随模式的 `/boxes` `/select` `/follow` 都打机器人 8081，
手机在 4G 上连不到 → 云端用同一条中继 WS 把请求转下去（车端只放行白名单路径）。"""


async def _relay_http(device_id: str, path: str) -> tuple[int, str]:
    hub = MEDIA_HUBS.get(device_id) or {}
    ws = hub.get('ws')
    if ws is None:
        raise HTTPException(503, '机器人不在线：先用手机连上机器人（云端中继）')
    rid = str(next(_PROXY_SEQ))
    fut = asyncio.get_running_loop().create_future()
    hub.setdefault('proxy', {})[rid] = fut
    try:
        await ws.send_json({'cmd': 'proxy', 'id': rid, 'path': path})
        msg = await asyncio.wait_for(fut, MEDIA_PROXY_TIMEOUT)
    except asyncio.TimeoutError:
        raise HTTPException(504, '机器人响应超时（可能是车端媒体服务在重启）')
    except Exception:                        # noqa: BLE001
        raise HTTPException(503, '车端连接刚刚断开，请刷新重试')
    finally:
        hub.get('proxy', {}).pop(rid, None)
    return int(msg.get('status') or 502), str(msg.get('body') or '')


@app.get('/api/devices/{device_id}/vision/{sub:path}')
async def device_vision_proxy(device_id: str, sub: str, request: Request,
                              authorization: str = Header(default='')):
    """`/vision/boxes` `/vision/select` `/vision/follow` `/vision/followstate` 的异地转发。"""
    user_id = current_user(authorization)
    _require_owner(user_id, device_id)
    if sub.split('?')[0] not in ('boxes', 'select', 'follow', 'followstate', 'healthz'):
        raise HTTPException(404, '不支持的面板接口')
    qs = request.url.query
    status, body = await _relay_http(device_id, '/' + sub + (('?' + qs) if qs else ''))
    return Response(content=body, status_code=status,
                    media_type='application/json')


@app.get('/api/media/health')
def media_health():
    """中继自检：谁在线、有几路在看、累计转了多少字节（排查用）。"""
    return {
        'token': bool(MEDIA_TOKEN),
        'max_clients': MEDIA_MAX_CLIENTS,
        'active': active_streams(),
        'devices': {dev: {'online': bool(hub.get('ws')),
                          'frames': hub.get('frames', 0),
                          'bytes': hub.get('bytes', 0),
                          'viewers': {str(ch): len(queues)
                                      for ch, queues in hub['subs'].items()}}
                    for dev, hub in MEDIA_HUBS.items()},
    }


@app.websocket('/ws/media/{device_id}')
async def media_ws(ws: WebSocket, device_id: str, token: str = ''):
    """车端中继拨入：二进制帧 = 1 字节通道号 + 完整 JPEG。"""
    await ws.accept()
    if not MEDIA_TOKEN or not hmac_compare(token, MEDIA_TOKEN):
        await ws.close(code=4401)
        return
    hub = _media_hub(device_id)
    hub['ws'] = ws
    # 车重连后，把还在等画面的通道重新播起来（旧连接可能已经被掐掉）
    for slot in list(hub['subs']):
        if hub['subs'][slot]:
            try:
                await ws.send_json({'cmd': 'play', 'ch': slot % 2,
                                    'profile': 'eco' if slot >= 2 else 'low'})
            except Exception:                # noqa: BLE001
                break
    try:
        while True:
            raw = await ws.receive()
            if raw.get('type') == 'websocket.disconnect':
                break
            data = raw.get('bytes')
            if data:
                slot = data[0]
                frame = data[1:]
                hub['frames'] = hub.get('frames', 0) + 1
                hub['bytes'] = hub.get('bytes', 0) + len(frame)
                for queue in list(hub['subs'].get(slot, ())):
                    if queue.full():         # 慢客户端：丢最旧，绝不排长队
                        try:
                            queue.get_nowait()
                        except asyncio.QueueEmpty:
                            pass
                    try:
                        queue.put_nowait(frame)
                    except asyncio.QueueFull:
                        pass
            elif raw.get('text'):
                try:
                    msg = json.loads(raw['text'])
                except ValueError:
                    continue
                if msg.get('type') == 'hello':
                    STATE_CACHE.setdefault(device_id, {})['media_relay'] = True
                elif msg.get('type') == 'proxy':
                    fut = hub.get('proxy', {}).pop(str(msg.get('id')), None)
                    if fut is not None and not fut.done():
                        fut.set_result(msg)
    except WebSocketDisconnect:
        pass
    finally:
        if hub.get('ws') is ws:
            hub['ws'] = None
        state = STATE_CACHE.get(device_id)
        if state is not None:
            state['media_relay'] = False


@app.get('/api/devices/{device_id}/stream')
async def device_stream(device_id: str, cam: str = 'main', profile: str = 'low',
                        t: str = '', authorization: str = Header(default='')):
    """把车端中继上来的 JPEG 转成浏览器认的 multipart MJPEG（异地观看）。

    ⚠️ 必须**边收边发**：任何一层缓冲（uvicorn/nginx/浏览器）都会把 150 ms
    的延迟放大成好几秒，所以这里逐帧 yield、并显式 `X-Accel-Buffering: no`。
    """
    user_id = _stream_user(device_id, t, authorization)
    _require_owner(user_id, device_id)
    hub = MEDIA_HUBS.get(device_id)
    if not hub or hub.get('ws') is None:
        raise HTTPException(503, '机器人不在线：异地观看需要车端媒体中继在跑（fwr-media-relay）')
    if active_streams() >= MEDIA_MAX_CLIENTS:
        raise HTTPException(429, '当前观看人数已满，请稍后再试')
    if profile not in ('low', 'eco'):
        profile = 'low'
    ch = 1 if cam == 'wide' else 0
    slot = media_slot(cam, profile)
    queue: asyncio.Queue = asyncio.Queue(maxsize=2)
    queues = hub['subs'].setdefault(slot, set())
    queues.add(queue)
    try:
        await hub['ws'].send_json({'cmd': 'play', 'ch': ch, 'profile': profile})
    except Exception:                        # noqa: BLE001
        queues.discard(queue)
        raise HTTPException(503, '车端连接刚刚断开，请刷新重试')

    async def frames():
        try:
            while True:
                frame = await queue.get()
                yield (b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                       + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
        finally:
            queues.discard(queue)
            if not queues:
                hub['subs'].pop(slot, None)
                ws = hub.get('ws')
                if ws is not None:
                    try:
                        await ws.send_json({'cmd': 'stop', 'ch': ch, 'profile': profile})
                    except Exception:        # noqa: BLE001
                        pass

    return StreamingResponse(
        frames(), media_type='multipart/x-mixed-replace; boundary=frame',
        headers={'Cache-Control': 'no-store, no-cache',
                 'X-Accel-Buffering': 'no'})


@app.post('/api/admin/reset_password')
def admin_reset_password(req: PasswordRequest, x_admin_token: str = Header(default='')):
    """管理员重置用户密码（忘记密码时用，需 JC_ADMIN_TOKEN）。"""
    expected = os.environ.get('JC_ADMIN_TOKEN', '')
    if not expected or x_admin_token != expected:
        raise HTTPException(403, '无权重置')
    if len(req.new_password) < 6:
        raise HTTPException(400, '新密码至少 6 位')
    with db() as conn:
        row = conn.execute('SELECT id FROM users WHERE email=?',
                           (req.old_password,)).fetchone()
        if row is None:
            raise HTTPException(404, '用户不存在')
        conn.execute('UPDATE users SET pw_hash=? WHERE id=?',
                     (hash_pw(req.new_password), row['id']))
    return {'ok': True}


@app.post('/api/change_password')
def change_password(req: PasswordRequest, user_id: int = Depends(current_user)):
    if len(req.new_password) < 6:
        raise HTTPException(400, '新密码至少 6 位')
    with db() as conn:
        row = conn.execute('SELECT pw_hash FROM users WHERE id=?',
                           (user_id,)).fetchone()
        if row is None:
            raise HTTPException(404, '用户不存在')
        if row['pw_hash'] != hash_pw(req.old_password):
            raise HTTPException(400, '原密码不正确')
        conn.execute('UPDATE users SET pw_hash=? WHERE id=?',
                     (hash_pw(req.new_password), user_id))
    return {'ok': True}


@app.post('/api/devices/{device_id}/power')
def device_power(device_id: str, req: PowerRequest,
                 user_id: int = Depends(current_user)):
    with db() as conn:
        row = conn.execute('SELECT owner_id FROM devices WHERE device_id=?',
                           (device_id,)).fetchone()
    if row is None or row['owner_id'] != user_id:
        raise HTTPException(403, '无权操作该设备')
    if req.action not in ('shutdown', 'reboot'):
        raise HTTPException(400, 'action 只能是 shutdown 或 reboot')
    publish_json(device_id, 'power', {'action': req.action})
    return {'ok': True, 'action': req.action}


@app.get('/api/devices/{device_id}/map')
def device_map(device_id: str, user_id: int = Depends(current_user)):
    with db() as conn:
        row = conn.execute('SELECT owner_id FROM devices WHERE device_id=?',
                           (device_id,)).fetchone()
    if row is None or row['owner_id'] != user_id:
        raise HTTPException(403, '无权访问该设备')
    payload = MAP_CACHE.get(device_id)
    if payload is None:
        raise HTTPException(404, '地图尚未上传')
    return payload


@app.websocket('/ws/control/{device_id}')
async def control_ws(ws: WebSocket, device_id: str, token: str = ''):
    # 2026-11-21：以前直接 `ws.close(code=4401)` 而不 accept，握手阶段会被
    # uvicorn 变成 HTTP 403 → 浏览器只看到 onclose(1006)，分不清"token 过期"
    # 和网络抖动，于是页面用过期 token 无限重连，永远显示「云端断开」。
    # 现在先 accept 再用 4401 关闭，页面收到就能直接跳登录页。
    try:
        user_id = user_from_token(token)
        auth_error = None
    except HTTPException:
        user_id = None
        auth_error = 4401
    with db() as conn:
        row = conn.execute('SELECT owner_id FROM devices WHERE device_id=?',
                           (device_id,)).fetchone()
    if user_id is not None and (row is None or row['owner_id'] != user_id):
        auth_error = 4403
    await ws.accept()
    if auth_error:
        await ws.close(code=auth_error)
        return
    CONNECTIONS.setdefault(device_id, set()).add(ws)
    cached = STATE_CACHE.get(device_id)
    if cached:
        await ws.send_json({'type': 'state', **cached})
    try:
        while True:
            raw = await ws.receive_json()
            if raw.get('type') == 'ping':
                # 2026-11-20：手机顶部 WiFi 图标的延迟测量（原样回传 t，页面算 RTT）
                await ws.send_json({'type': 'pong', 't': raw.get('t')})
                continue
            if raw.get('type') == 'preview':
                points = raw.get('points') or []
                if points:
                    publish_json(device_id, 'preview', {'points': points})
                continue
            if raw.get('type') == 'diag':
                # 手机页面把屏幕上的诊断条文本发回来，便于远程看它到底卡在哪一层
                try:
                    with open('/opt/jc/logs/diag.log', 'a', encoding='utf-8') as f:
                        f.write(f"{time.time():.0f} {device_id} "
                                f"{str(raw.get('text'))[:300]}\n")
                except Exception:            # noqa: BLE001
                    pass
                continue
            if raw.get('type') == 'lmap_plain':
                # 手机是 http（非安全上下文）时没有 DecompressionStream，
                # 请求桥接改发未压缩的实时地图。
                publish_json(device_id, 'lmap_plain',
                             {'plain': bool(raw.get('plain'))})
                continue
            if raw.get('type') == 'map_edit':
                # 手机编辑好的地图（栅格字节 base64，可 gzip）回传给车端保存
                body = {
                    'name': str(raw.get('name') or ''),
                    'width': int(raw.get('width') or 0),
                    'height': int(raw.get('height') or 0),
                    'resolution': float(raw.get('resolution') or 0.05),
                    'origin': raw.get('origin') or {},
                    'pgm': str(raw.get('pgm') or ''),
                    'gz': bool(raw.get('gz')),
                }
                if body['width'] > 0 and body['height'] > 0 and body['pgm']:
                    publish_json(device_id, 'map_edit', body)
                continue
            if raw.get('type') == 'goal':
                points = raw.get('points') or []
                if 1 <= len(points) <= 20:
                    publish_json(device_id, 'goal', {'points': points})
                continue
            if raw.get('type') == 'nav':
                body = {'action': raw.get('action', 'stop')}
                if raw.get('name'):
                    body['name'] = str(raw['name'])
                if raw.get('map'):
                    body['map'] = str(raw['map'])
                # 2026-11-24：这里以前是**字段白名单**，手机切模式发的 `mode` 被丢掉，
                # 车端只收到 {"action":"mode"} → `_switch_mode(None)` 回落成手动。
                # 现场表现：点「导航模式」等 1 min+ 一直「等待导航栈就绪」，车不动。
                if raw.get('mode'):
                    body['mode'] = str(raw['mode'])
                publish_json(device_id, 'nav', body)
                continue
            if raw.get('type') == 'stop':
                publish_cmd(device_id, 0.0, 0.0)
                continue
            linear = clamp(float(raw.get('linear', 0.0)), MAX_LINEAR)
            angular = clamp(float(raw.get('angular', 0.0)), MAX_ANGULAR)
            try:
                ttl = int(raw.get('ttl_ms', 800))
            except (TypeError, ValueError):
                ttl = 800
            publish_cmd(device_id, linear, angular, ttl)
    except WebSocketDisconnect:
        pass
    except (ValueError, TypeError):
        pass
    finally:
        publish_cmd(device_id, 0.0, 0.0)
        CONNECTIONS.get(device_id, set()).discard(ws)


def clamp(value: float, limit: float) -> float:
    if value != value:                       # NaN
        return 0.0
    return max(-limit, min(limit, value))


# ---------------- MQTT ----------------
MQTT_CLIENT: Optional['mqtt.Client'] = None
MQTT_OK = False


def mqtt_connected() -> bool:
    return MQTT_OK


def start_mqtt() -> None:
    global MQTT_CLIENT
    if mqtt is None:
        return
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id=f'jc-api-{os.getpid()}')
    client.on_connect = _on_connect
    client.on_disconnect = _on_disconnect
    client.on_message = _on_message
    client.connect_async(MQTT_HOST, MQTT_PORT, keepalive=30)
    client.loop_start()
    MQTT_CLIENT = client


def _on_connect(client, _u, _f, reason, _p=None):
    global MQTT_OK
    MQTT_OK = reason == 0
    if MQTT_OK:
        client.subscribe([('robot/+/state', 0), ('robot/+/online', 0),
                          ('robot/+/event', 1), ('robot/+/map', 1),
                          ('robot/+/goal_result', 1), ('robot/+/path', 0),
                          ('robot/+/scan', 0),
                          ('robot/+/local_plan', 0),
                          ('robot/+/preview_path', 1),
                          ('robot/+/live_map', 0),
                          ('robot/+/maps', 1)])


def _on_disconnect(_client, _u, _f, _reason, _p=None):
    global MQTT_OK
    MQTT_OK = False


def _on_message(_client, _u, msg):
    parts = msg.topic.split('/')
    if len(parts) != 3:
        return
    device_id = parts[1]
    # 在线状态是纯文本（online/offline），不是 JSON，必须单独处理
    if parts[2] == 'online':
        value = msg.payload.decode(errors='ignore')
        STATE_CACHE.setdefault(device_id, {})['online'] = value
        for ws in list(CONNECTIONS.get(device_id, ())):
            try:
                asyncio.run_coroutine_threadsafe(
                    ws.send_json({'type': 'online', 'online': value}), LOOP)
            except Exception:                    # noqa: BLE001
                CONNECTIONS.get(device_id, set()).discard(ws)
        return
    try:
        payload = json.loads(msg.payload.decode())
    except (ValueError, UnicodeDecodeError):
        return
    if parts[2] == 'map':
        MAP_CACHE[device_id] = payload
        return
    if parts[2] == 'state':
        STATE_CACHE[device_id] = payload
    elif parts[2] == 'online':
        STATE_CACHE.setdefault(device_id, {})['online'] = \
            msg.payload.decode(errors='ignore')
    for ws in list(CONNECTIONS.get(device_id, ())):
        try:
            asyncio.run_coroutine_threadsafe(
                ws.send_json({'type': parts[2], **payload}), LOOP)
        except Exception:                    # noqa: BLE001
            CONNECTIONS.get(device_id, set()).discard(ws)


def publish_json(device_id: str, suffix: str, body: dict) -> None:
    if MQTT_CLIENT is None or not MQTT_OK:
        return
    MQTT_CLIENT.publish(f'robot/{device_id}/{suffix}',
                        json.dumps(body, ensure_ascii=False), qos=1)


def publish_cmd(device_id: str, linear: float, angular: float,
                ttl_ms: int = 800) -> None:
    global SEQ
    if MQTT_CLIENT is None or not MQTT_OK:
        return
    SEQ += 1
    body = json.dumps({'seq': SEQ, 'ttl_ms': ttl_ms,
                       'linear': round(linear, 4),
                       'angular': round(angular, 4)})
    MQTT_CLIENT.publish(f'robot/{device_id}/cmd', body, qos=0)
