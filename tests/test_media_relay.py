"""车端媒体中继（异地看画面）的纯逻辑与静态回归。

现场背景（2026-11-25）：手机在办公室 WiFi 里看画面走的是
`http://<lan_ip>:8081/video`，换 4G / 别的 WiFi 就不可达 → 黑屏。
控制链路本来就是云端的（车主动连 MQTT），所以把画面也做成"车主动拨出"。
"""
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))

import media_relay as relay  # noqa: E402


# ---------------------------------------------------------------- 切帧

def test_splits_two_frames_in_one_chunk():
    buf = bytearray(b'\xff\xd8AAA\xff\xd9\xff\xd8BBB\xff\xd9')
    assert relay.jpeg_frames(buf) == [b'\xff\xd8AAA\xff\xd9', b'\xff\xd8BBB\xff\xd9']
    assert buf == b''


def test_holds_partial_frame_until_rest_arrives():
    buf = bytearray(b'\xff\xd8AAA')
    assert relay.jpeg_frames(buf) == []
    buf += b'BBB\xff\xd9'
    assert relay.jpeg_frames(buf) == [b'\xff\xd8AAABBB\xff\xd9']


def test_frame_split_across_marker_boundary():
    """`FF` 和 `D8` 分在两个 TCP 块里（真实 MJPEG 常见）。"""
    buf = bytearray()
    assert relay.jpeg_frames(buf) == []
    buf += b'\xff'
    assert relay.jpeg_frames(buf) == []
    buf += b'\xd8X\xff\xd9'
    assert relay.jpeg_frames(buf) == [b'\xff\xd8X\xff\xd9']


def test_drops_garbage_before_first_marker():
    buf = bytearray(b'----fwrframe\r\nContent-Type: image/jpeg\r\n\r\n')
    assert relay.jpeg_frames(buf) == []
    buf += b'\xff\xd8Q\xff\xd9'
    assert relay.jpeg_frames(buf) == [b'\xff\xd8Q\xff\xd9']


# ---------------------------------------------------------------- 控制消息

@pytest.mark.parametrize('msg,expected', [
    ({'cmd': 'play', 'ch': 0}, ('play', 0)),
    ({'cmd': 'play', 'ch': 1}, ('play', 1)),
    ({'cmd': 'play'}, ('play', 0)),          # 缺省主摄
    ({'cmd': 'stop', 'ch': 1}, ('stop', 1)),
    ({'cmd': 'ping'}, ('pong', -1)),
    ({'cmd': 'bogus'}, ('ignore', -1)),
    ({'cmd': 'play', 'ch': 9}, ('ignore', -1)),
    ({'cmd': 'play', 'ch': 'x'}, ('ignore', -1)),
])
def test_control_action(msg, expected):
    assert relay.control_action(msg) == expected


def test_unknown_command_never_starts_a_stream():
    """云端发来乱七八糟的东西时**不能**顺手开流（否则车端白烧 CPU）。"""
    for bad in ({}, {'cmd': ''}, {'cmd': None}, {'cmd': 'PLAY'}):
        assert relay.control_action(bad)[0] == 'ignore'


# ---------------------------------------------------------------- 地址 / 退避

def test_channel_url_uses_low_profile():
    assert relay.channel_url('http://127.0.0.1:8081', relay.CH_MAIN) == \
        'http://127.0.0.1:8081/video?cam=main&profile=low'
    assert 'cam=wide' in relay.channel_url('http://h:8081/', relay.CH_WIDE)


def test_channel_url_keeps_full_profile_when_asked():
    assert 'profile=full' in relay.channel_url('http://h:8081', 0, 'full')


def test_relay_url_carries_token_and_device():
    url = relay.relay_url('ws://jccontrol.xyz/ws/media/', 'car-001', 'tok')
    assert url == 'ws://jccontrol.xyz/ws/media/car-001?token=tok'


def test_backoff_grows_and_is_capped():
    delays = [relay.backoff_delay(i) for i in range(1, 8)]
    assert delays[0] < delays[3], '退避必须递增'
    assert relay.backoff_delay(30) <= 36, '上限 30 s（±20% 抖动）'


# ---------------------------------------------------------------- 静态回归

def test_relay_does_not_import_aiohttp_at_module_level():
    """车端环境里 aiohttp 由 venv 提供；模块级导入会让纯逻辑测试和
    `--help` 在没有 aiohttp 的机器上直接炸。"""
    text = (ROOT / 'tools' / 'media_relay.py').read_text(encoding='utf-8')
    head = text.split('class MediaRelay')[0]
    assert 'import aiohttp' not in head


def test_relay_only_keeps_latest_frame():
    """每通道只留最新一帧：排队会故意加延迟。"""
    text = (ROOT / 'tools' / 'media_relay.py').read_text(encoding='utf-8')
    assert 'self.latest[ch] = frame' in text
    assert 'asyncio.Queue' not in text.split('class MediaRelay')[1]


def test_unit_file_exists_and_enables_restart():
    unit = (ROOT / 'deploy' / 'systemd' / 'fwr-media-relay.service')
    assert unit.exists(), '缺少 systemd 单元'
    body = unit.read_text(encoding='utf-8')
    assert 'Restart=always' in body
    assert 'media_relay.py' in body
    assert 'HOME=/home/manbaout' in body, 'systemd 缺 HOME 会让节点起不来'
