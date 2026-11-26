"""手动速度的三道闸必须一致（2026-11-26：单轮上限 300→400 mm/s 后手动仍卡在 0.30）。

链条：页面摇杆 `MAX_LINEAR` → `mqtt_bridge.max_linear_mps` → 底盘 `manual_max_speed_mps`。
任何一道比其它小，摇杆推满也只到最小那个值 —— 现场表现就是"改了参数还是 0.3"。
"""

from __future__ import annotations

import re
from pathlib import Path

PAGE = Path('cloud/web/car.html')
UNIT = Path('deploy/systemd/fwr-mqtt-bridge.service')
BRIDGE = Path('ros2_ws/src/fwr_bringup/fwr_bringup/base_bridge.py')


def _page_limits():
    s = PAGE.read_text(encoding='utf-8')
    m = re.search(r'const MAX_LINEAR = ([0-9.]+), MAX_ANGULAR = ([0-9.]+);', s)
    assert m, 'car.html 里找不到 MAX_LINEAR'
    return float(m.group(1)), float(m.group(2))


def _unit_param(name: str) -> float:
    s = UNIT.read_text(encoding='utf-8')
    m = re.search(rf'-p {name}:=([0-9.]+)', s)
    assert m, f'单元里找不到 {name}'
    return float(m.group(1))


def _chassis_manual_cap() -> float:
    s = BRIDGE.read_text(encoding='utf-8')
    m = re.search(r"declare_parameter\('manual_max_speed_mps', ([0-9.]+)\)", s)
    assert m, 'base_bridge 里找不到 manual_max_speed_mps'
    return float(m.group(1))


def test_manual_speed_chain_matches():
    page_v, page_w = _page_limits()
    assert page_v == 0.40 and page_w == 0.74
    # 摇杆满量程不能超过桥的钳位，否则推满也只是桥的那个数
    assert page_v <= _unit_param('max_linear_mps') + 1e-9
    assert page_w <= _unit_param('max_angular_rps') + 1e-9
    # 也不能超过底盘 manual_velocity 的硬限（|ω| ≤ 0.75）
    assert page_v <= _chassis_manual_cap() + 1e-9
    assert page_w <= 0.75


def test_chassis_cap_is_400_mm_s():
    """manual_max_speed_mps 0.40 必须 ≤ 固件单轮上限 400 mm/s = 0.40 m/s。"""
    assert _chassis_manual_cap() == 0.40
