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
    ({'cmd': 'play', 'ch': 0}, ('play', 0, 'low')),
    ({'cmd': 'play', 'ch': 1}, ('play', 1, 'low')),
    ({'cmd': 'play'}, ('play', 0, 'low')),   # 缺省主摄 / 高清档
    ({'cmd': 'stop', 'ch': 1}, ('stop', 1, 'low')),
    ({'cmd': 'play', 'ch': 0, 'profile': 'eco'}, ('play', 0, 'eco')),
    ({'cmd': 'play', 'ch': 1, 'profile': 'eco'}, ('play', 1, 'eco')),
    ({'cmd': 'ping'}, ('pong', -1, '')),
    ({'cmd': 'proxy', 'id': '1', 'path': '/boxes'}, ('proxy', -1, '')),
    ({'cmd': 'bogus'}, ('ignore', -1, '')),
    ({'cmd': 'play', 'ch': 9}, ('ignore', -1, '')),
    ({'cmd': 'play', 'ch': 'x'}, ('ignore', -1, '')),
    ({'cmd': 'play', 'ch': 0, 'profile': 'weird'}, ('ignore', -1, '')),
])
def test_control_action(msg, expected):
    assert relay.control_action(msg) == expected


def test_unknown_command_never_starts_a_stream():
    """云端发来乱七八糟的东西时**不能**顺手开流（否则车端白烧 CPU）。"""
    for bad in ({}, {'cmd': ''}, {'cmd': None}, {'cmd': 'PLAY'}):
        assert relay.control_action(bad)[0] == 'ignore'


@pytest.mark.parametrize('ch,profile,slot', [
    (0, 'low', 0), (1, 'low', 1), (0, 'eco', 2), (1, 'eco', 3),
])
def test_slot_roundtrip(ch, profile, slot):
    """槽位号 = 通道 + 档位偏移；反解必须还原（云端按槽位分发帧）。"""
    assert relay.slot_of(ch, profile) == slot
    assert relay.channel_of(slot) == (ch, profile)


def test_proxy_is_whitelisted():
    """云端只能让车端打白名单里的本机接口（别做成任意 URL 转发）。
    2026-11-26：跟随模式的 /boxes /select /follow 在 4G 上要经这里转发。"""
    text = (ROOT / 'tools' / 'media_relay.py').read_text(encoding='utf-8')
    assert 'PROXY_PATHS = (' in text
    for want in ("'/boxes'", "'/select'", "'/follow'", "'/followstate'"):
        assert want in text, f'白名单缺 {want}'
    assert 'not path.startswith(PROXY_PATHS)' in text, '必须拒绝白名单外的路径'
    assert 'async def _proxy(' in text


def test_slot_rejects_garbage():
    assert relay.channel_of(9) == (-1, '')


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
    assert 'self.latest[slot] = frame' in text
    assert 'asyncio.Queue' not in text.split('class MediaRelay')[1]


def test_unit_file_exists_and_enables_restart():
    unit = (ROOT / 'deploy' / 'systemd' / 'fwr-media-relay.service')
    assert unit.exists(), '缺少 systemd 单元'
    body = unit.read_text(encoding='utf-8')
    assert 'Restart=always' in body
    assert 'media_relay.py' in body
    assert 'HOME=/home/manbaout' in body, 'systemd 缺 HOME 会让节点起不来'


def test_follow_services_include_leg_tracker():
    """2026-11-26 用户报「人走到车后面画面追不上、雷达也没显示跟随」。

    真因：`leg_tracker` 从来没被拉起过 —— `/robot/lidar/persons` 没有发布者，
    `person_follower` 的轨迹表是空的，相机丢人后没有雷达可接管。
    现在它跟视觉/跟随节点一起按需启停。
    """
    text = (ROOT / 'tools' / 'robot_mjpeg_server.py').read_text(encoding='utf-8')
    assert "FOLLOW_UNITS = ('fwr-vision', 'fwr-follow', 'fwr-legtracker')" in text
    unit = (ROOT / 'deploy' / 'systemd' / 'fwr-legtracker.service').read_text(encoding='utf-8')
    assert 'python -m fwr_bringup.leg_tracker' in unit
    assert 'Restart=always' in unit
    # 腿跟踪只发观测、不发速度（AST 单测另外钉死），单元里也不许出现运动门
    assert 'motion_enabled' not in unit
    # main() 不许重复 shutdown（SIGTERM 时会 RCLError → systemd 记 failed）
    src = (ROOT / 'ros2_ws' / 'src' / 'fwr_bringup' / 'fwr_bringup' / 'leg_tracker.py').read_text(encoding='utf-8')
    assert 'ExternalShutdownException' in src
    assert 'if rclpy.ok():' in src
