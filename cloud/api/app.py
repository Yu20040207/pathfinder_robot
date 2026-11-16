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
    WS   /ws/control/{device_id}?token=...   实时控制 + 状态推送
    GET  /api/health

环境变量：
    JC_DB          SQLite 路径（默认 /opt/jc/data/jc.db）
    JC_SECRET      JWT 密钥（生产必须改）
    JC_BIND_CODE   设备绑定码（默认 123456，试用期）
    JC_MQTT_HOST/PORT   EMQX（默认 127.0.0.1:1883）
"""
import asyncio
import hashlib
import json
import os
import sqlite3
import time
from typing import Dict, Optional, Set

import jwt
from fastapi import Depends, FastAPI, Header, HTTPException, WebSocket, WebSocketDisconnect
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
    try:
        user_id = user_from_token(token)
    except HTTPException:
        await ws.close(code=4401)
        return
    with db() as conn:
        row = conn.execute('SELECT owner_id FROM devices WHERE device_id=?',
                           (device_id,)).fetchone()
    if row is None or row['owner_id'] != user_id:
        await ws.close(code=4403)
        return
    await ws.accept()
    CONNECTIONS.setdefault(device_id, set()).add(ws)
    cached = STATE_CACHE.get(device_id)
    if cached:
        await ws.send_json({'type': 'state', **cached})
    try:
        while True:
            raw = await ws.receive_json()
            if raw.get('type') == 'preview':
                points = raw.get('points') or []
                if points:
                    publish_json(device_id, 'preview', {'points': points})
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
                publish_json(device_id, 'nav', body)
                continue
            if raw.get('type') == 'stop':
                publish_cmd(device_id, 0.0, 0.0)
                continue
            linear = clamp(float(raw.get('linear', 0.0)), MAX_LINEAR)
            angular = clamp(float(raw.get('angular', 0.0)), MAX_ANGULAR)
            publish_cmd(device_id, linear, angular)
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


def publish_cmd(device_id: str, linear: float, angular: float) -> None:
    global SEQ
    if MQTT_CLIENT is None or not MQTT_OK:
        return
    SEQ += 1
    body = json.dumps({'seq': SEQ, 'linear': round(linear, 4),
                       'angular': round(angular, 4), 'ttl_ms': 300})
    MQTT_CLIENT.publish(f'robot/{device_id}/cmd', body, qos=0)
