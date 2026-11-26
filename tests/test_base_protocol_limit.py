"""固件/ROS 单轮上限必须一致（2026-11-26 300 → 400 mm/s）。"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from fwr_bringup.base_protocol import MAX_WHEEL_MMPS, encode_wheel_speed


def test_ros_limit_is_400():
    assert MAX_WHEEL_MMPS == 400.0
    assert encode_wheel_speed(400, -400) == b'v 400.000 -400.000\n'


def test_over_limit_rejected():
    with pytest.raises(ValueError):
        encode_wheel_speed(400.5, 0)
    with pytest.raises(ValueError):
        encode_wheel_speed(0, -401)


def test_firmware_constant_matches_ros():
    """固件 `MAX_WHEEL_MMPS` 与 ROS 侧必须一字不差，否则超限帧在 ROS 先抛异常。"""
    header = Path('firmware/esp32s3_base_control/main/control.h').read_text(encoding='utf-8')
    m = re.search(r'#define\s+MAX_WHEEL_MMPS\s+([0-9.]+)f', header)
    assert m, 'control.h 里找不到 MAX_WHEEL_MMPS'
    assert float(m.group(1)) == MAX_WHEEL_MMPS
