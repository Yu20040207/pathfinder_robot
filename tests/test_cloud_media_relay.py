"""云端媒体中继（异地看画面）的回归。

跑这个文件需要 fastapi/httpx（云端才有），本地开发机缺少时整体跳过：

    python3 -m venv /tmp/jcvenv
    /tmp/jcvenv/bin/pip install fastapi httpx aiohttp pyjwt paho-mqtt pytest
    /tmp/jcvenv/bin/python -m pytest tests/test_cloud_media_relay.py -q
"""
import importlib
import pathlib
import sys
import time

import pytest

pytest.importorskip('fastapi')
pytest.importorskip('httpx')

ROOT = pathlib.Path(__file__).resolve().parents[1]
API_DIR = ROOT / 'cloud' / 'api'

TOKEN = 'test-media-token'
DEV = 'car-001'


def load_app(tmp_path, monkeypatch, media_token: str = TOKEN):
    monkeypatch.setenv('JC_DB', str(tmp_path / 'jc.db'))
    monkeypatch.setenv('JC_SECRET', 'test-secret')
    monkeypatch.setenv('JC_BIND_CODE', '123456')
    monkeypatch.setenv('JC_MEDIA_TOKEN', media_token)
    monkeypatch.setenv('JC_MQTT_HOST', '127.0.0.1')
    if str(API_DIR) not in sys.path:
        sys.path.insert(0, str(API_DIR))
    if 'app' in sys.modules:
        del sys.modules['app']
    return importlib.import_module('app')


@pytest.fixture()
def cloud(tmp_path, monkeypatch):
    app_mod = load_app(tmp_path, monkeypatch)
    from fastapi.testclient import TestClient
    # on_startup 会去连 MQTT（测试环境没有），关掉省得等
    app_mod.start_mqtt = lambda *a, **k: None
    client = TestClient(app_mod.app)
    with client:
        yield app_mod, client


def auth(client, cloud):
    client.post('/api/register', json={'email': 'a@b.c', 'password': 'pw1234'})
    r = client.post('/api/login', json={'email': 'a@b.c', 'password': 'pw1234'})
    token = r.json()['access_token']
    client.post('/api/devices/bind', json={'device_id': DEV, 'code': '123456'},
                headers={'Authorization': f'Bearer {token}'})
    return token


# ---------------------------------------------------------------- 令牌

def test_media_lists_remote_url_with_short_token(cloud):
    app_mod, client = cloud
    token = auth(client, app_mod)
    app_mod.STATE_CACHE[DEV] = {'lan_ip': '192.168.0.24'}   # 车报上来的局域网地址
    r = client.get(f'/api/devices/{DEV}/media',
                   headers={'Authorization': f'Bearer {token}'})
    assert r.status_code == 200
    body = r.json()
    assert body['url'].startswith('http://')          # 局域网直连仍然保留
    remote = body['remote']
    assert remote['url'] == f'/api/devices/{DEV}/stream'
    assert remote['online'] is False                   # 车还没连上来
    payload = app_mod.jwt.decode(remote['token'], app_mod.SECRET, algorithms=['HS256'])
    assert payload['kind'] == 'stream' and payload['dev'] == DEV


def test_stream_token_is_scoped_to_one_device(cloud):
    app_mod, client = cloud
    token = auth(client, app_mod)
    good = app_mod.make_stream_token(DEV, 1)
    assert app_mod._stream_user(DEV, good, '') == 1
    with pytest.raises(app_mod.HTTPException) as exc:
        app_mod._stream_user('car-002', good, '')
    assert exc.value.status_code == 401


def test_login_token_is_not_accepted_as_stream_token(cloud):
    """登录令牌 24 h 有效；观看令牌必须短时，否则 `<img>` 里的链接能长期复用。"""
    app_mod, client = cloud
    login = auth(client, app_mod)
    with pytest.raises(app_mod.HTTPException) as exc:
        app_mod._stream_user(DEV, login, '')
    assert exc.value.status_code == 401


def test_stream_rejects_other_owners_device(cloud):
    app_mod, client = cloud
    auth(client, app_mod)                              # 用户 A 绑定 car-001
    client.post('/api/register', json={'email': 'b@b.c', 'password': 'pw1234'})
    token_b = client.post('/api/login',
                          json={'email': 'b@b.c', 'password': 'pw1234'}).json()['access_token']
    r = client.get(f'/api/devices/{DEV}/stream',
                   params={'t': app_mod.make_stream_token(DEV, 2)},
                   headers={'Authorization': f'Bearer {token_b}'})
    assert r.status_code == 403


# ---------------------------------------------------------------- 车端拨入

def test_stream_returns_503_when_robot_offline(cloud):
    app_mod, client = cloud
    token = auth(client, app_mod)
    r = client.get(f'/api/devices/{DEV}/stream',
                   params={'t': app_mod.make_stream_token(DEV, 1)},
                   headers={'Authorization': f'Bearer {token}'})
    assert r.status_code == 503
    assert '中继' in r.json()['detail']


def test_relay_ws_rejects_bad_token(cloud):
    app_mod, client = cloud
    auth(client, app_mod)
    with pytest.raises(Exception):
        with client.websocket_connect(f'/ws/media/{DEV}?token=wrong') as ws:
            ws.receive_bytes()


def test_relay_ws_counts_frames_and_bytes(cloud):
    app_mod, client = cloud
    auth(client, app_mod)
    jpeg = b'\xff\xd8hello\xff\xd9'
    with client.websocket_connect(f'/ws/media/{DEV}?token={TOKEN}') as ws:
        ws.send_text('{"type":"hello","device":"car-001"}')
        ws.send_bytes(b'\x00' + jpeg)                  # ch0 = 主摄
        ws.send_bytes(b'\x01' + jpeg)                  # ch1 = 广角
        deadline = time.time() + 3
        while time.time() < deadline:
            health = client.get('/api/media/health').json()
            if health['devices'].get(DEV, {}).get('frames', 0) >= 2:
                break
            time.sleep(0.05)
    health = client.get('/api/media/health').json()
    dev = health['devices'][DEV]
    # 连接关闭后 online 归 False，但累计帧数/字节保留（页面自检要看它）
    assert dev['online'] is False
    assert dev['frames'] == 2
    assert dev['bytes'] == 2 * len(jpeg)


def test_relay_reconnect_keeps_hub_and_clears_online(cloud):
    app_mod, client = cloud
    auth(client, app_mod)
    with client.websocket_connect(f'/ws/media/{DEV}?token={TOKEN}') as ws:
        ws.send_bytes(b'\x00\xff\xd8x\xff\xd9')
    # 断开后仍然保留 hub（累计字节、重启后页面还能看到统计）
    health = client.get('/api/media/health').json()
    assert DEV in health['devices']
    assert health['devices'][DEV]['online'] is False


def test_max_clients_guard_exists(cloud):
    app_mod, client = cloud
    assert app_mod.MEDIA_MAX_CLIENTS >= 2
    health = client.get('/api/media/health').json()
    assert health['max_clients'] == app_mod.MEDIA_MAX_CLIENTS


# ---------------------------------------------------------------- 静态回归

def test_stream_sets_no_buffering_header():
    """三层里任何一层缓冲都会把 150 ms 放大成几秒。"""
    text = (ROOT / 'cloud' / 'api' / 'app.py').read_text(encoding='utf-8')
    assert "'X-Accel-Buffering': 'no'" in text
    assert 'StreamingResponse' in text
    assert 'multipart/x-mixed-replace' in text


def test_stream_never_buffers_more_than_two_frames():
    text = (ROOT / 'cloud' / 'api' / 'app.py').read_text(encoding='utf-8')
    assert 'asyncio.Queue(maxsize=2)' in text
