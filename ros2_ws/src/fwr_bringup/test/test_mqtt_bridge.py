"""mqtt_bridge 纯函数单测（不连 MQTT）。"""
import json
import math

import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

from fwr_bringup.mqtt_bridge import (
    build_state_payload, clamp, parse_command, yaw_of,
)


def cmd(**kw):
    return json.dumps(kw).encode()


def test_clamp():
    assert clamp(0.5, 1.0) == 0.5
    assert clamp(-2.0, 1.0) == -1.0
    assert clamp(3.0, 1.0) == 1.0


def test_parse_valid_command():
    out = parse_command(cmd(seq=1, linear=0.12, angular=0.0, ttl_ms=300), 0.15, 0.6)
    assert out['linear'] == pytest.approx(0.12)
    assert out['angular'] == pytest.approx(0.0)
    assert out['ttl_s'] == pytest.approx(0.3)
    assert out['seq'] == 1


def test_parse_clamps_out_of_range():
    out = parse_command(cmd(seq=2, linear=9.9, angular=-9.9), 0.15, 0.6)
    assert out['linear'] == pytest.approx(0.15)
    assert out['angular'] == pytest.approx(-0.6)


def test_parse_rejects_garbage():
    assert parse_command(b'not json', 0.15, 0.6) is None
    assert parse_command(cmd(linear='x', angular=0), 0.15, 0.6) is None
    assert parse_command(cmd(linear=float('nan'), angular=0), 0.15, 0.6) is None
    assert parse_command(cmd(angular=0), 0.15, 0.6) is None      # 缺 linear


def test_parse_ttl_bounds():
    assert parse_command(cmd(linear=0, angular=0, ttl_ms=-5), 0.15, 0.6)['ttl_s'] == 0
    assert parse_command(cmd(linear=0, angular=0, ttl_ms=99999), 0.15, 0.6)['ttl_s'] == 2.0


def test_state_payload_with_sensors():
    bat = BatteryState()
    bat.voltage = 24.12
    bat.current = 3.4
    bat.percentage = 0.62
    payload = json.loads(build_state_payload(bat, (1.234, -5.678, 0.5), 'navigating', 0, True))
    assert payload['voltage'] == pytest.approx(24.12)
    assert payload['x'] == pytest.approx(1.234)
    assert payload['nav'] == 'navigating'
    assert payload['armed'] is True
    assert 'ts' in payload


def test_state_payload_without_sensors():
    payload = json.loads(build_state_payload(None, None, None, None, False))
    assert 'voltage' not in payload
    assert payload['nav'] == 'unknown'
    assert payload['armed'] is False


def test_yaw_of_identity():
    q = Odometry().pose.pose.orientation
    q.w = 1.0
    assert yaw_of(q) == pytest.approx(0.0)


def test_yaw_of_quarter_turn():
    q = Odometry().pose.pose.orientation
    q.z = math.sin(math.pi / 4)
    q.w = math.cos(math.pi / 4)
    assert yaw_of(q) == pytest.approx(math.pi / 2)
