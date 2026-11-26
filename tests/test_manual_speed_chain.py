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
# 第四道闸在云端：ECS 上 jc-api 的 Environment=JC_MAX_LINEAR / JC_MAX_ANGULAR
CLOUD = Path('deploy/ecs/jc-api-speed.conf')


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


def _cloud_limits():
    s = CLOUD.read_text(encoding='utf-8')
    # ⚠️ 只认 `Environment=` 行：注释里写着历史值 0.30，正则别抓到它
    lv = re.search(r'^Environment=JC_MAX_LINEAR=([0-9.]+)', s, re.M)
    aw = re.search(r'^Environment=JC_MAX_ANGULAR=([0-9.]+)', s, re.M)
    assert lv and aw, '云端 drop-in 里缺 JC_MAX_LINEAR/JC_MAX_ANGULAR'
    return float(lv.group(1)), float(aw.group(1))


def test_manual_speed_chain_matches():
    page_v, page_w = _page_limits()
    assert page_v == 0.40 and page_w == 0.74
    # 云端 clamp 也不能比页面小（否则页面给 0.40、云端砍成 0.30 —— 2026-11-26 现场）
    cloud_v, cloud_w = _cloud_limits()
    assert page_v <= cloud_v + 1e-9
    assert page_w <= cloud_w + 1e-9
    # 摇杆满量程不能超过桥的钳位，否则推满也只是桥的那个数
    assert page_v <= _unit_param('max_linear_mps') + 1e-9
    assert page_w <= _unit_param('max_angular_rps') + 1e-9
    # 也不能超过底盘 manual_velocity 的硬限（|ω| ≤ 0.75）
    assert page_v <= _chassis_manual_cap() + 1e-9
    assert page_w <= 0.75


def test_chassis_cap_is_400_mm_s():
    """manual_max_speed_mps 0.40 必须 ≤ 固件单轮上限 400 mm/s = 0.40 m/s。"""
    assert _chassis_manual_cap() == 0.40


def test_map_joystick_keeps_its_own_limits():
    """建图模式摇杆**不许**跟着手动模式一起变快（用户 2026-11-26 明确要求）。

    手动和建图共用 `sendCmd()`，所以页面里必须保留两套常量：
    `MAX_LINEAR/MAX_ANGULAR`（手动 0.40/0.74）与 `MAP_MAX_LINEAR/MAP_MAX_ANGULAR`（建图 0.30/0.60）。
    """
    s = PAGE.read_text(encoding='utf-8')
    m = re.search(r'const MAP_MAX_LINEAR = ([0-9.]+), MAP_MAX_ANGULAR = ([0-9.]+);', s)
    assert m, '页面里找不到 MAP_MAX_LINEAR/MAP_MAX_ANGULAR'
    assert float(m.group(1)) == 0.30 and float(m.group(2)) == 0.60
    # 建图摇杆的每一处发送都必须带上建图上限（漏一处就会变成 0.40）
    assert s.count('sendCmd(mjx, mjy, MAP_MAX_LINEAR, MAP_MAX_ANGULAR)') >= 3
    assert 'sendCmd(mjx, mjy);' not in s
