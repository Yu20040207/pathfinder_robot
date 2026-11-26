"""重新认领目标后的"软对准"（2026-11-26 用户报：掉头重新看到人之后左右摆）。

现象：人从旁边绕到车后 → 车掉头 → 重新识别到人 → **对准时左摆右摆**。
两个成因，分别由这两处治：
1. `_ema['cx']` 还是掉头前那一侧的旧中心，新框在另一侧 → 平滑值爬好几帧，
   这期间误差方向是错的（已在 `person_follower` 里"重新认领就清 EMA"修掉）；
2. 刚认回来就在中线附近按满幅 ω 修，12 Hz 检测 + 83 ms EMA + `w_accel 3.0`
   每次都比目标多转一点 → `reacq_cap()` 把它限到 0.40（**只限"误差已经不大"时**，
   掉头那种大误差照旧满速追，不能变慢）。
"""

from __future__ import annotations

from fwr_bringup.follow_core import DEFAULTS, reacq_cap


def test_near_center_is_capped_after_reacquire():
    assert abs(reacq_cap(0.74, 0.10, True) - 0.40) < 1e-9
    assert abs(reacq_cap(-0.74, -0.10, True) + 0.40) < 1e-9


def test_far_off_target_keeps_full_speed():
    """误差还很大（掉头途中）不许收力，否则转得更慢。"""
    assert abs(reacq_cap(0.74, 0.48, True) - 0.74) < 1e-9
    assert abs(reacq_cap(-0.70, -0.46, True) + 0.70) < 1e-9


def test_below_cap_is_untouched():
    assert abs(reacq_cap(0.20, 0.05, True) - 0.20) < 1e-9
    assert reacq_cap(0.0, 0.0, True) == 0.0


def test_inactive_passes_through():
    """软对准窗口外（老行为）什么都不改。"""
    assert abs(reacq_cap(0.74, 0.05, False) - 0.74) < 1e-9


def test_custom_window():
    assert abs(reacq_cap(0.74, 0.05, True, soft_wmax=0.30) - 0.30) < 1e-9
    assert abs(reacq_cap(0.74, 0.30, True, soft_ex=0.25) - 0.74) < 1e-9


def test_soft_values_stay_inside_chassis_limits():
    """软对准的 0.40 必须小于硬限 0.74，且默认值真在 DEFAULTS 语义范围内。"""
    assert 0.0 < 0.40 < DEFAULTS['steer_wmax'] <= 0.74
