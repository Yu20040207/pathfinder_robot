"""减速斜坡（2026-11-26 用户："停止时一下子停住，惯性太大"）。

链路真相：固件 `base_command()` 里 `if (left==0 && right==0) { base_stop(c); }`
→ **零指令直接把 PWM 归零**，150 mm/s² 的 setpoint 斜率根本不参与，所以停是"撞停"。
修法在 ROS 侧：`BaseBridgeCore._slew()` 把上一帧速度按 `decel_mm_s2` 收下来，
收完才发 `x`。导航（auto）通道 `decel_enabled=False`，保持"说停就停"。
"""

from __future__ import annotations

import pytest

from fwr_bringup.base_bridge_core import BaseBridgeCore, slew_wheel
from fwr_bringup.base_protocol import BaseState


def _state(count=0, fault=0, esp_us=1_000_000):
    return BaseState(esp_us, count, count, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     True, fault)


def _core(decel_mps2: float) -> BaseBridgeCore:
    core = BaseBridgeCore(decel_mm_s2=1000.0 * decel_mps2)
    core.connect(0.0)
    core.verified = True
    return core


def _run(core, t0, seconds, hold=1.0):
    """以 20 Hz 跑一段，返回 [(t, command)]。"""
    out = []
    step = 0.05
    n = int(round(seconds / step))
    for k in range(n + 1):
        out.append((t0 + k * step, core.step(None, t0 + k * step).command))
    return out


def _wheel(cmd: bytes):
    if cmd == b'x':
        return 0.0, 0.0
    parts = cmd.decode().strip().split()
    return float(parts[1]), float(parts[2])


# ---------------- 纯函数 ----------------

def test_slew_wheel_decel_is_budget_limited():
    assert slew_wheel(400.0, 0.0, 25.0) == 375.0
    assert slew_wheel(10.0, 0.0, 25.0) == 0.0      # 预算够 → 直接到 0
    assert slew_wheel(0.0, 0.0, 25.0) == 0.0


def test_slew_wheel_accel_not_limited():
    """加速不在这里限（固件自带 150 mm/s² 斜率），否则 0→0.4 要 2.7 s。"""
    assert slew_wheel(0.0, 400.0, 25.0) == 400.0
    assert slew_wheel(100.0, 400.0, 25.0) == 400.0


def test_slew_wheel_reversal_goes_through_zero():
    """换向必须先过零，绝不在一帧里翻符号（那等于让固件硬刹再反转）。"""
    assert slew_wheel(400.0, -200.0, 25.0) == 375.0     # 只是往零减
    assert slew_wheel(20.0, -200.0, 25.0) == 0.0        # 预算够就停在 0
    assert slew_wheel(0.0, -200.0, 25.0) == -200.0      # 从 0 才进入反方向


# ---------------- 核心 step 行为 ----------------

def test_zero_command_ramps_down_not_instant():
    core = _core(0.50)                       # 500 mm/s²
    # hold_s=0：指令立即过期，测的是纯减速斜坡（正常链路还有 0.2~1.0 s 的 hold）
    core.accept_cmd(0.30, 0.0, 0.0, hold_s=0.0)   # 单轮 300 mm/s
    assert _wheel(core.step(None, 0.0).command) == (300.0, 300.0)
    zero_at = None
    seen = []
    for k in range(1, 40):
        t = k * 0.05
        cmd = core.step(None, t).command      # 指令已过期 → 目标 0
        l, r = _wheel(cmd)
        if cmd != b'x':
            seen.append(l)
        if cmd == b'x':
            zero_at = t
            break
    assert seen, '必须有一串逐渐减速的帧，不能一帧到 x'
    assert seen == sorted(seen, reverse=True), f'必须单调下降：{seen}'
    # 300 mm/s ÷ 500 mm/s² = 0.6 s
    assert zero_at is not None and 0.5 <= zero_at <= 0.75, zero_at


def test_ramp_budget_matches_param():
    """0.40 m/s ÷ 0.50 m/s² = 0.8 s 停稳（用户可调 manual_decel_mps2）。"""
    core = _core(0.50)
    core.accept_cmd(0.40, 0.0, 0.0, hold_s=0.0)
    core.step(None, 0.0)
    t_zero = None
    for k in range(1, 60):
        t = k * 0.05
        if core.step(None, t).command == b'x':
            t_zero = t
            break
    assert t_zero is not None and abs(t_zero - 0.8) <= 0.1, t_zero


def test_decel_disabled_keeps_old_instant_stop():
    core = _core(0.0)                        # 0 = 关
    core.accept_cmd(0.40, 0.0, 0.0)
    assert _wheel(core.step(None, 0.0).command) == (400.0, 400.0)
    assert core.step(None, 1.5).command == b'x'   # 过期立刻停


def test_decel_enabled_flag_bypasses_ramp_for_auto():
    """导航通道（decel_enabled=False）必须立刻 'x'，不能被斜坡拖慢。"""
    core = _core(0.50)
    core.accept_cmd(0.40, 0.0, 0.0)
    core.step(None, 0.0)
    core.decel_enabled = False
    assert core.step(None, 0.05).command != b'x'      # 指令还新鲜 → 照常走
    assert core.step(None, 1.0).command == b'x'       # 过期就立刻停，不走斜坡


def test_halt_clears_ramp_so_next_stop_is_instant():
    core = _core(0.50)
    core.accept_cmd(0.40, 0.0, 0.0)
    core.step(None, 0.0)
    core.halt()
    assert core._ramp is None
    assert core.step(None, 0.05).command == b'x'


def test_ramp_never_exceeds_wheel_protocol_limit():
    from fwr_bringup.base_bridge_core import scale_velocity_to_wheel_limit
    core = _core(0.50)
    v, w = scale_velocity_to_wheel_limit(0.40, 0.3, core.track_m, max_wheel_mps=0.40)
    core.accept_cmd(v, w, 0.0)
    for k in range(40):
        cmd = core.step(None, k * 0.05).command
        if cmd == b'x':
            continue
        l, r = _wheel(cmd)
        assert abs(l) <= 400.0 + 1e-6 and abs(r) <= 400.0 + 1e-6


def test_fault_streams_x_immediately():
    core = _core(0.50)
    core.accept_cmd(0.40, 0.0, 0.0)
    core.step(None, 0.0)
    core.fault = 3
    assert core.step(None, 0.05).command == b'x'
