"""机械臂运动学：正解/逆解往返必须自洽，可达边界必须报对原因。

这些断言的价值在**边界**：握手能不能做，取决于"人的手在不在可达空间里"，
而不是"平均情况下在不在"。所以关键的那几条都钉死边界两侧各一个点。
"""

from __future__ import annotations

import numpy as np
import pytest

import math

from fwr_bringup.arm_kinematics import (ABOVE_REACH, ANGLE_LIMIT, BEHIND,
                                        BELOW_REACH, OK, TOO_CLOSE, TOO_FAR,
                                        ArmGeometry, DEFAULT_GEOMETRY,
                                        check_point, forward, horizontal_band_m,
                                        horizontal_reach_m, inverse, solve_hand,
                                        with_shoulder_height)

G = DEFAULT_GEOMETRY


def test_default_geometry_matches_user_numbers():
    """25 cm 臂 + 手最高 1.25 m 必须自洽（这是用户给的唯一硬数字）。"""
    assert G.reach_m == pytest.approx(0.22, abs=1e-9)
    assert G.max_hand_height_m == pytest.approx(1.25, abs=1e-9)
    assert (G.l_base + G.l_upper + G.l_fore + G.l_grip) == pytest.approx(0.25, abs=1e-9)
    # 车顶安装面 = 肩高 − 底座那段
    assert G.mount_height_m == pytest.approx(1.00, abs=1e-9)


@pytest.mark.parametrize("seed", range(8))
def test_inverse_then_forward_roundtrip(seed):
    """逆解出的关节角回代正解，位置必须一致（1 mm 内）。

    随机撒点是故意的：只测几个"好看的"角度会漏掉 q3 换分支时的符号错误。
    """
    rng = np.random.default_rng(seed)
    checked = 0
    for _ in range(200):
        q1 = rng.uniform(G.q1_min, G.q1_max)
        q2 = rng.uniform(G.q2_min, G.q2_max)
        q3 = rng.uniform(G.q3_min, G.q3_max)
        q4 = rng.uniform(G.q4_min, G.q4_max)
        tgt = forward((q1, q2, q3, q4))
        pitch = q2 + q3 + q4  # 该姿态下夹爪的绝对仰角
        joints, reason = inverse(tgt, pitch)
        if joints is None:
            continue  # 被结构限位挡掉是允许的，但不能是"全挡"
        got = forward(joints)
        assert got == pytest.approx(tgt, abs=1e-3), f"roundtrip drift {got} vs {tgt}"
        checked += 1
    assert checked > 20, "多数位姿被挡 = 几何值或限位写错了"


def test_reach_boundary_is_exact():
    """可达半径上正好一点必须可达，往外 1 cm 必须 TOO_FAR。"""
    hand = (G.reach_m, 0.0, G.shoulder_height_m)  # 肩同高、正前方、正好 0.22
    _j, reason = solve_hand(hand, gripper_pitch=0.0)
    assert reason == OK, f"边界点应可达，实际 {reason}"

    assert check_point((G.reach_m + 0.01, 0.0, G.shoulder_height_m)) == TOO_FAR


def test_too_close_is_reported_separately():
    """太近和太远必须是两个原因：前者要人退后，后者要车靠近。"""
    near = (0.005, 0.0, 1.02)  # 手几乎贴在臂根上，肘折不回来
    assert check_point(near) == TOO_CLOSE


def test_above_reach():
    """超过 1.25 m 够不着，这是人抬手高度的上限。"""
    assert check_point((0.0, 0.0, 1.30)) == ABOVE_REACH
    # 往下也有个底：安装面以下（车顶内部）不算臂的事
    assert check_point((0.10, 0.0, 0.70)) == BELOW_REACH  # 安装面(1.00)以下再走满一个臂长


def test_reachable_handshake_point():
    """最典型的握手位姿：正前方 8 cm、离地 1.15 m（站着手抬起来的高度）。"""
    assert check_point((0.08, 0.0, 1.15)) == OK


def test_behind_the_arm_is_behind_not_far():
    """人站在臂正后方时不是"够不着"，而是底座转不过去 —— 原因不能混。"""
    assert check_point((-0.15, 0.0, G.shoulder_height_m)) == BEHIND


def test_both_elbow_branches_agree_on_position():
    """肘上/肘下解出的位置必须相同，否则 IK 的符号写反了。"""
    # 这个点两个分支都在关节限位内（近处的点肘朝上时 q4 会超 100°，那是真实约束）
    tgt = (0.15, 0.0, 1.05)
    up, r_up = inverse(tgt, gripper_pitch=0.0, elbow_up=True)
    down, r_down = inverse(tgt, gripper_pitch=0.0, elbow_up=False)
    assert r_up == OK and r_down == OK
    assert forward(up) == pytest.approx(forward(down), abs=1e-9)
    assert up[2] * down[2] < 0  # q3 反号


def test_horizontal_reach_is_not_naive_sphere():
    """⚠️ 水平够程必须扣掉夹爪那 4 cm，不能直接用球半径。

    用错会**高估**可达范围，现场表现是"规划说够得着、实际差 3~4 cm"。
    """
    at_shoulder = horizontal_reach_m(height_m=G.shoulder_height_m)
    assert at_shoulder == pytest.approx(G.reach_m, abs=1e-9)  # 肩同高时刚好 0.22

    # 高出肩 0.10 m 时：腕只能到 sqrt(0.18²−0.10²)=0.1497，再横伸 4 cm
    higher = horizontal_reach_m(height_m=G.shoulder_height_m + 0.10)
    assert higher == pytest.approx(math.sqrt(0.18 ** 2 - 0.10 ** 2) + 0.04, abs=1e-9)
    assert higher < at_shoulder

    # 超过 0.18 m 就连腕都到不了
    assert horizontal_reach_m(height_m=G.shoulder_height_m + 0.19) == 0.0


def test_horizontal_band_matches_shoulder_math():
    """水平伸直时的高度带：肩 − 0.18 ~ 肩 + 0.18（夹爪仰角 0）。"""
    lo, hi = horizontal_band_m()
    assert lo == pytest.approx(G.shoulder_height_m - 0.18, abs=1e-9)
    assert hi == pytest.approx(G.shoulder_height_m + 0.18, abs=1e-9)


def test_shoulder_height_sweep_finds_handshake_band():
    """扫肩高，找出能覆盖"站着的人抬手 1.20 m"的那一段。

    回答"上层折叠高度够不够"的核心用例：肩低于 0.98 m 就碰不到 1.20 m。
    """
    good = []
    for h in np.arange(0.90, 1.101, 0.05):
        g = with_shoulder_height(float(h))
        if g.max_hand_height_m >= 1.20:
            good.append(round(float(h), 2))
    assert good, "没有任何肩高能碰到 1.20 m —— 折叠高度或臂长不够"
    assert all(h >= 0.98 - 1e-9 for h in good)


def test_min_wrist_dist_follows_elbow_limit():
    """肘限位收紧，最小腕距必须变大（不然"太近"这条永远不触发）。"""
    loose = ArmGeometry(q3_min=math.radians(-160), q3_max=math.radians(160))
    tight = ArmGeometry(q3_min=math.radians(-90), q3_max=math.radians(90))
    assert tight.min_wrist_dist_m > loose.min_wrist_dist_m
    assert tight.min_wrist_dist_m == pytest.approx(0.09 * math.sqrt(2), abs=1e-9)


def test_unreachable_reasons_are_distinct_strings():
    """原因字符串别撞车：上层要靠它决定"车靠近"还是"人退后"。"""
    reasons = {OK, TOO_FAR, TOO_CLOSE, ANGLE_LIMIT, ABOVE_REACH, BELOW_REACH, BEHIND}
    assert len(reasons) == 7
