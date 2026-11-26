"""P2 相机 + 雷达融合的纯逻辑测试（2026-11-24）。

守的是什么
----------
用户要求：「人绕到车侧/车后仍要跟」。相机视场只有 ~70°，人走到侧面就看不见了；
雷达 360° 能看见腿，但**分不清人和家具**。所以规矩是：

1. **雷达不许自己起跟** —— 只有"刚才被相机点名过"的同一条轨迹才能接管；
2. 接管只发生在**相机已经丢人**的时候，且最多 `lidar_hold_s`（5 s）；
3. 接管期间**绝不后退**（雷达只看得到腿，不知道身后是人是墙）；
4. 车头走廊有障碍照样停（安全优先级不变）。
"""

from __future__ import annotations

import math

import pytest

from fwr_bringup.follow_core import (DEFAULTS, cam_bearing, lidar_follow,
                                     pick_lidar_track)


def tr(tid: int, bearing_deg: float, r: float, cam: bool = True,
       static: bool = False, v: float = 0.3) -> dict:
    # 2026-11-26：只有"相机新鲜背书"的轨迹才允许认领（`cam` 有效期 1 s），
    # 默认给 True 模拟"相机正看着这个人"；测家具时显式传 cam=False。
    return {'id': tid, 'bearing': math.radians(bearing_deg), 'r': r,
            'cam': cam, 'cam_age_s': (0.0 if cam else None), 'static': static,
            'v': v}


# ---------------------------------------------------------------- 方位换算
def test_cam_bearing_center_is_zero():
    assert cam_bearing(0.5) == pytest.approx(0.0, abs=1e-9)


def test_cam_bearing_sign_matches_lidar():
    # 画面左边（cx<0.5）→ 方位角为正（车体左正，REP-103）
    assert cam_bearing(0.2) > 0.0
    assert cam_bearing(0.8) < 0.0
    # 70° 视场：画面边缘 ≈ ±35°
    assert math.degrees(cam_bearing(0.0)) == pytest.approx(35.0, abs=0.5)
    assert math.degrees(cam_bearing(1.0)) == pytest.approx(-35.0, abs=0.5)


# ---------------------------------------------------------------- 认领
def test_pick_track_prefers_sticky_id():
    cands = [tr(1, 8.0, 2.0), tr(2, 4.0, 2.1)]
    # 上次认领的是 1；1 的方位差更大，但也要黏住它
    assert pick_lidar_track(cands, math.radians(4.5), last_id=1)['id'] == 1


def test_pick_track_without_history_takes_closest():
    cands = [tr(1, 8.0, 2.0), tr(2, 4.0, 2.1)]
    assert pick_lidar_track(cands, math.radians(4.5), last_id=-1)['id'] == 2


def test_pick_track_rejects_far_bearing():
    # 方位差 30° > lidar_assoc_deg 25° → 不认领（宁可不跟，也不跟错人）
    assert pick_lidar_track([tr(1, 30.0, 2.0)], 0.0) is None


def test_pick_track_rejects_out_of_range():
    assert pick_lidar_track([tr(1, 0.0, 0.2)], 0.0) is None
    assert pick_lidar_track([tr(1, 0.0, 5.0)], 0.0) is None


def test_pick_track_empty_is_none():
    assert pick_lidar_track([], 0.0) is None
    assert pick_lidar_track(None, 0.0) is None


# ---------------------------------------------------------------- 控制律
BIG = math.inf


def test_lidar_follow_straight_ahead_moves():
    v, w, reason = lidar_follow(0.0, 2.2, BIG, BIG, BIG, BIG)
    assert reason == 'lidar_follow'
    assert v > 0.05 and abs(w) < 1e-9


def test_lidar_follow_turns_toward_target_left():
    # 目标在左（bearing>0）→ ω>0（左转）
    _v, w, _r = lidar_follow(math.radians(15.0), 2.2, BIG, BIG, BIG, BIG)
    assert w > 0.05


def test_lidar_follow_turns_toward_target_right():
    _v, w, _r = lidar_follow(math.radians(-15.0), 2.2, BIG, BIG, BIG, BIG)
    assert w < -0.05


def test_lidar_follow_faces_target_behind_without_moving():
    # 人在车后（180°）→ 原地转身，不前进（差速车不能侧移，也不能倒着追）
    v, w, reason = lidar_follow(math.radians(175.0), 2.0, BIG, BIG, BIG, BIG)
    assert v == 0.0
    assert reason == 'lidar_face'
    assert abs(w) > 0.2


def test_lidar_follow_stops_when_close_and_never_reverses():
    v, _w, reason = lidar_follow(0.0, 1.0, BIG, BIG, BIG, BIG)
    assert v == 0.0 and reason == 'lidar_close'
    # 迟滞：停在 1.2 m，但退到 1.55 m 才重新走
    v2, _w2, reason2 = lidar_follow(0.0, 1.4, BIG, BIG, BIG, BIG, close_hold=True)
    assert v2 == 0.0 and reason2 == 'lidar_close'
    v3, _w3, reason3 = lidar_follow(0.0, 1.6, BIG, BIG, BIG, BIG, close_hold=True)
    assert v3 > 0.0 and reason3 in ('lidar_follow', 'lidar_slow')


def test_lidar_follow_never_returns_negative_speed_anywhere():
    for deg in (-179, -90, -45, -10, 0, 10, 45, 90, 179):
        for r in (0.5, 1.0, 1.5, 2.0, 3.5, 8.0):
            v, _w, _reason = lidar_follow(math.radians(deg), r, BIG, BIG, BIG, BIG)
            assert v >= 0.0, (deg, r, v)


def test_lidar_follow_far_target_only_turns():
    v, _w, reason = lidar_follow(0.0, 4.0, BIG, BIG, BIG, BIG)
    assert v == 0.0 and reason == 'lidar_far'


def test_lidar_follow_obstacle_in_corridor_stops():
    # 车头走廊 0.20 m 有东西（< front_hard_m 0.35）→ 停，即便人在 2 m
    v, w, reason = lidar_follow(0.0, 2.0, 0.20, 0.20, BIG, BIG)
    assert v == 0.0 and w == 0.0 and reason == 'obstacle_stop'


def test_lidar_follow_slows_near_obstacle():
    v, _w, reason = lidar_follow(0.0, 2.6, 0.45, 0.50, BIG, BIG)
    assert 0.0 < v < DEFAULTS['lidar_vmax']
    assert reason == 'lidar_slow'


def test_lidar_follow_blocked_side_stops_turning():
    # 要左转但左边 0.10 m 就是墙 → 不转也不走
    v, w, reason = lidar_follow(math.radians(20.0), 2.2, BIG, BIG, 0.10, BIG)
    assert v == 0.0 and w == 0.0 and reason == 'side_blocked'


def test_lidar_follow_speed_cap_below_chassis_limit():
    # 单轮包络：|v| + |ω|·track/2 ≤ max_wheel_mps（0.38，2026-11-26 前是 0.30）
    p = DEFAULTS
    for deg in (-25, -10, 0, 10, 25):
        v, w, _r = lidar_follow(math.radians(deg), 2.2, BIG, BIG, BIG, BIG)
        assert abs(v) + abs(w) * p['track_m'] / 2.0 <= p['max_wheel_mps'] + 1e-9
        # 底盘硬限 0.40（manual_max_speed_mps）也必须满足
        assert abs(v) + abs(w) * p['track_m'] / 2.0 <= 0.40 + 1e-9


# ------------------------------------------------- 轨迹换号后按位置续认
# 2026-11-24 用户报"雷达跟着跟着突然断开"：实流里 leg_tracker 的 id 在
# 腿交叉 / 两腿合并时会重生，旧写法只认固定 id，一换号就丢目标。
from fwr_bringup.follow_core import reacquire_track  # noqa: E402


def _tr(tid: int, bearing_deg: float, r: float, static: bool = False,
        cam: bool = True) -> dict:
    # 2026-11-26：续认只认"相机新鲜背书"的轨迹（cam 1 s 有效）
    return {'id': tid, 'bearing': math.radians(bearing_deg), 'r': r,
            'static': static, 'cam': cam,
            'cam_age_s': (0.0 if cam else None), 'v': 0.4}


def test_reacquire_keeps_same_id_first():
    cands = [_tr(3, 2.0, 2.0), _tr(9, 1.0, 2.0)]
    # 原来那条还在 → 直接用它，不去比距离
    assert reacquire_track(cands, 0.0, 2.0, last_id=3)['id'] == 3


def test_reacquire_after_id_changed_takes_nearest():
    cands = [_tr(9, 3.0, 2.05), _tr(11, 18.0, 2.9)]
    assert reacquire_track(cands, 0.0, 2.0, last_id=3)['id'] == 9


def test_reacquire_rejects_static_furniture():
    # 椅子不动 → leg_tracker 标 static → 不许被续认
    cands = [_tr(5, 0.0, 2.0, static=True)]
    assert reacquire_track(cands, 0.0, 2.0, last_id=3) is None


def test_reacquire_rejects_big_bearing_jump():
    # 方位差 40° > lidar_reacq_deg 20° → 不认（宁可不跟，也不跟错）
    assert reacquire_track([_tr(9, 40.0, 2.0)], 0.0, 2.0) is None


def test_reacquire_rejects_big_range_jump():
    # 距离差 1.5 m > lidar_reacq_dr 0.90 m → 不认
    assert reacquire_track([_tr(9, 0.0, 3.5)], 0.0, 2.0) is None


def test_reacquire_without_history_is_none():
    assert reacquire_track([_tr(9, 0.0, 2.0)], None, 2.0) is None
    assert reacquire_track([_tr(9, 0.0, 2.0)], 0.0, None) is None


def test_reacquire_empty_list():
    assert reacquire_track([], 0.0, 2.0) is None


def test_lidar_hold_long_enough_for_occlusion():
    """续跟窗口至少 8 s：人从柱子/门后走过要能等回来（原 5 s 实测偏短）。"""
    assert DEFAULTS['lidar_hold_s'] >= 8.0


def test_reacquire_keeps_standing_person_same_id():
    """人停下来等车 → leg_tracker 标 static；同一条被点名的轨迹必须继续跟。"""
    cands = [_tr(7, 0.0, 1.8, static=True)]
    got = reacquire_track(cands, 0.0, 1.8, last_id=7)
    assert got is not None and got['id'] == 7


# ---------------- 2026-11-26：雷达接管"左右摆"（face 纯 P ×2 → PD + 滞回） ----------------

def test_lidar_face_gain_is_damped_not_bang_bang():
    """老写法 `lidar_kp*2.0*b` 一进 face 就顶满 0.74 rad/s，20 Hz 下必冲过头。

    现场（2026-11-26 用户报"左转右转对准幅度大，刷新重新跟随就稳"）：
    轨迹号 #34/#49/#95/#112 乱跳 + 方位 ±25~60°，face 支路就左右各甩一炮。
    """
    p = DEFAULTS
    v, w, reason = lidar_follow(0.60, 2.2, 9, 9, 9, 9)
    assert reason == 'lidar_face' and v == 0.0
    assert abs(w) <= p['lidar_face_wmax'] + 1e-9 < p['wmax']


def test_lidar_face_damping_reduces_turn_while_closing():
    """方位在收（rate<0）时阻尼项提前松力。"""
    p = DEFAULTS
    b = 0.30                     # 低于 face 门槛 → 走正常跟随支路
    _, w_free, _ = lidar_follow(b, 2.2, 9, 9, 9, 9)
    _, w_damp, _ = lidar_follow(b, 2.2, 9, 9, 9, 9, bearing_rate=-2.0)
    assert w_damp < w_free - 1e-6
    # 阻尼只许收力：反向 rate 最多抵消一半 P，不会把转向翻号
    _, w_open, _ = lidar_follow(b, 2.2, 9, 9, 9, 9, bearing_rate=+6.0)
    assert w_open <= p['lidar_kp'] * b * 1.5


def test_lidar_face_hysteresis():
    """滞回：进 face 用 0.50 rad，已在 face 里要掉到 0.38 以下才退出。"""
    p = DEFAULTS
    assert abs(p['lidar_face_exit_rad']) < abs(p['lidar_face_rad'])
    b = 0.42
    _, w_normal, r_normal = lidar_follow(b, 2.2, 9, 9, 9, 9)
    assert r_normal == 'lidar_follow'
    _, w_hold, r_hold = lidar_follow(b, 2.2, 9, 9, 9, 9, face_hold=True)
    assert r_hold == 'lidar_face'
    assert abs(w_hold) <= p['lidar_face_wmax'] + 1e-9
    # 掉到退出阈值以下 → 回到正常跟随
    _, _, r_out = lidar_follow(0.30, 2.2, 9, 9, 9, 9, face_hold=True)
    assert r_out != 'lidar_face'


def test_bearing_ema_smooths_and_reports_rate():
    from fwr_bringup.follow_core import bearing_ema
    first, rate0 = bearing_ema(None, 0.20, 0.05, 0.5)
    assert first == 0.20 and rate0 == 0.0
    second, rate1 = bearing_ema(0.20, 0.60, 0.05, 0.5)
    assert abs(second - 0.40) < 1e-9            # 只走一半
    assert rate1 > 0.0                           # 方位在变大


def test_flip_confirm_rejects_track_hopping():
    """轨迹换号 + 方位突变 → 先不信；连续一致 0.3 s 才认。"""
    from fwr_bringup.follow_core import flip_confirm
    ok, last_ok, flip = flip_confirm((0.10, 34), None, 0.90, 49, 10.00, 0.60, 0.30)
    assert ok is False and last_ok == (0.10, 34) and flip is not None
    ok, last_ok, flip = flip_confirm(last_ok, flip, 0.90, 49, 10.10, 0.60, 0.30)
    assert ok is False
    ok, last_ok, flip = flip_confirm(last_ok, flip, 0.90, 49, 10.35, 0.60, 0.30)
    assert ok is True and last_ok == (0.90, 49) and flip is None


def test_flip_confirm_allows_smooth_drift():
    """同一条轨迹慢慢转（方位连续变化）不能被当成突变挡掉。"""
    from fwr_bringup.follow_core import flip_confirm
    last, flip = (0.10, 34), None
    for k, b in enumerate((0.20, 0.32, 0.45, 0.58)):
        ok, last, flip = flip_confirm(last, flip, b, 34, 10.0 + 0.05 * k, 0.60, 0.30)
        assert ok is True
