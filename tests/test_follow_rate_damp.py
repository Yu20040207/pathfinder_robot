"""对准时的"实测转速阻尼"（2026-11-26 用户反复报"对准我时左摆右摆"）。

为什么纯 PD 止不住：框中心变化率比真实转动**晚 1~2 帧**（相机 15 fps + 检测 12 Hz +
EMA），底盘占空比爬升（固件 `DUTY_RAMP_PER_S 0.5`）又让车在"收力"之后继续转
0.5~1.2 s。于是"看到对准了才收力"永远迟一步 → 过冲 → 反着修 → 来回摆。

`rate_damp()` 用 `/odom` 的实测角速度（**没有帧延迟**）做反向制动，
且**只在中线圈 `|err| < window` 里生效**（大误差照旧满速追，别把掉头变慢），
越靠近中线阻尼越强。
"""

from __future__ import annotations

from fwr_bringup.follow_core import rate_damp


def test_brakes_hard_when_centered_but_still_spinning():
    """误差几乎为 0，但车还在以 0.74 rad/s 转 → 指令要压到接近 0。"""
    out = rate_damp(0.74, 0.00, 0.74, kd=0.9, window=0.25)
    assert abs(out - 0.074) < 1e-9
    assert abs(out) < 0.1


def test_counters_spin_the_other_way():
    """指令是左转 0，车却在往右 0.60 → 必须给出左转的制动（反号）。"""
    out = rate_damp(0.0, 0.02, -0.60, kd=0.9, window=0.25)
    assert out > 0.4


def test_no_effect_outside_window():
    """大误差（还在掉头/大角度对准）不许被拖慢。"""
    assert rate_damp(0.74, 0.40, 0.74) == 0.74
    assert rate_damp(-0.74, -0.30, -0.74) == -0.74


def test_gain_ramps_with_distance_from_center():
    """越靠近中线阻尼越强：半程的衰减必须比中线处小。"""
    center = rate_damp(0.60, 0.02, 0.50)
    halfway = rate_damp(0.60, 0.125, 0.50)
    assert halfway > center          # 0.125 处 gain 0.5，收得少
    assert center < halfway < 0.60


def test_zero_kd_is_kill_switch():
    assert rate_damp(0.74, 0.0, 0.74, kd=0.0) == 0.74


def test_result_stays_inside_wmax():
    """反向制动也不许超过底盘硬限（|ω| ≤ 0.74，超了整帧被丢）。"""
    out = rate_damp(0.0, 0.01, -2.0, kd=0.9, window=0.25, wmax=0.74)
    assert abs(out) <= 0.74 + 1e-9
    out2 = rate_damp(0.0, -0.01, 2.0, kd=0.9, window=0.25, wmax=0.74)
    assert abs(out2) <= 0.74 + 1e-9


def test_sign_symmetry():
    a = rate_damp(0.30, 0.05, 0.20)
    b = rate_damp(-0.30, -0.05, -0.20)
    assert abs(a + b) < 1e-12
