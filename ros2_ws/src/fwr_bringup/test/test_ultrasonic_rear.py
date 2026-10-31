"""后向超声节点：协议解析与异常值处理的单元测试（不需要硬件）。"""
import math
import struct

import pytest

from fwr_bringup.ultrasonic_rear import (
    NO_DATA,
    crc16,
    decode_distances,
    range_from_raw,
)


def test_crc16_matches_spec_examples():
    """规格书 7 条示例帧的 CRC（交接文档里已与规格书核对过）。"""
    examples = [
        (bytes.fromhex('010301060004'), 'a5f4'),
        (bytes.fromhex('010301060001'), '65f7'),
        (bytes.fromhex('010300000001'), '840a'),
    ]
    for payload, expected in examples:
        assert crc16(payload).hex() == expected


def test_decode_four_distances():
    payload = bytes.fromhex('0103') + bytes((8,)) + bytes.fromhex('04db0371023e0131')
    resp = payload + crc16(payload)
    assert decode_distances(resp) == [1243, 881, 574, 305]


def test_decode_rejects_short_frame():
    assert decode_distances(b'') is None
    assert decode_distances(b'\x01\x03\x08\x00') is None


def test_decode_raises_on_modbus_exception():
    payload = bytes.fromhex('018302')
    resp = payload + crc16(payload)
    with pytest.raises(ValueError):
        decode_distances(resp)


def test_decode_returns_none_when_count_too_small():
    payload = bytes.fromhex('0103') + bytes((2,)) + bytes.fromhex('012c')
    resp = payload + crc16(payload)
    assert decode_distances(resp) is None


def test_anomaly_codes_are_not_distances():
    """0xFFFF/0xFFFE/0xFFFD 必须返回 None（绝不当 0 或空地）。"""
    for code in NO_DATA:
        assert range_from_raw(code, 0.05, 3.5) is None


def test_range_conversion_and_limits():
    assert range_from_raw(1000, 0.05, 3.5) == pytest.approx(1.0)
    assert range_from_raw(50, 0.05, 3.5) == pytest.approx(0.05)
    assert range_from_raw(3500, 0.05, 3.5) == pytest.approx(3.5)
    # 超量程 / 过近 → 无可靠读数
    assert range_from_raw(3600, 0.05, 3.5) is None
    assert range_from_raw(10, 0.05, 3.5) is None


def test_frames_default_is_four_rear_sensors():
    """默认 frame 命名符合交接文档要求。"""
    import pathlib
    src = pathlib.Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'ultrasonic_rear.py'
    text = src.read_text()
    for name in ('ultrasonic_rear_left_out', 'ultrasonic_rear_left_in',
                 'ultrasonic_rear_right_in', 'ultrasonic_rear_right_out'):
        assert name in text
