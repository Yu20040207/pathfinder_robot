"""跟随控制律的单元测试（纯函数，不需要 ROS / 相机 / 雷达）。

2026-11-21 第二版：远近看**框高占比**、避障看**车身走廊**、转向增益加大。
"""

from __future__ import annotations

import math

import pytest

from fwr_bringup.follow_core import (DEFAULTS, OscGuard, decide, door_clear,
                                    door_metrics, door_swept_radius, limit_pair,
                                    passage_metrics, scan_metrics, steer)

BOX = {'x': 0.40, 'y': 0.30, 'w': 0.20, 'h': 0.55}   # 正中、中等大小
P = dict(DEFAULTS)


def _decide(person=BOX, lost=0.0, hard=9.0, slow=9.0, left=9.0, right=9.0,
            size_hold=False, **kw):
    return decide(person, lost, hard, slow, left, right, P,
                  size_hold=size_hold, **kw)


# ---------------------------------------------------------------- 远近
def test_no_person_stops():
    assert _decide(person=None)[:3] == (0.0, 0.0, 'no_target')


def test_small_box_full_speed():
    v, _, r, h = _decide(person={'x': 0.4, 'y': 0.3, 'w': 0.1, 'h': 0.50})
    assert v == pytest.approx(P['vmax']) and r == 'following' and h == 0.50


def test_big_box_stops():
    v, _, r, _ = _decide(person={'x': 0.4, 'y': 0.3, 'w': 0.3, 'h': 0.95})
    assert v == 0.0 and r == 'too_close'


def test_box_at_stop_threshold_stops():
    v, _, r, _ = _decide(person={'x': 0.4, 'y': 0.3, 'w': 0.3, 'h': 0.90})
    assert v == 0.0 and r == 'too_close'


def test_mid_box_slows():
    """框在 go_box_h(0.82) ~ stop_box_h(0.85) 之间 → 按比例慢走。

    2026-11-21 晚用户第三次实测：停距 0.75 → 0.85、起走 0.65 → 0.80
    （"框 80 的时候就可以跟随了，不用停住等我"），减速带只剩 0.82~0.85。
    """
    v, _, r, _ = _decide(person={'x': 0.4, 'y': 0.3, 'w': 0.2, 'h': 0.84})
    assert r == 'following' and 0.05 < v < P['vmax']
    # 框 ≤0.82 全速（最常见的跟随距离），不再爬行
    v2, _, r2, _ = _decide(person={'x': 0.4, 'y': 0.3, 'w': 0.2, 'h': 0.78})
    assert r2 == 'following' and v2 == pytest.approx(P['vmax'])


def test_stop_at_85_percent():
    """2026-11-21 晚用户第三次实测：框到 **85%** 才停，75% 时还要继续跟。"""
    v, _, r, _ = _decide(person={'x': 0.4, 'y': 0.3, 'w': 0.2, 'h': 0.86})
    assert v == 0.0 and r == 'too_close'
    v2, _, r2, _ = _decide(person={'x': 0.4, 'y': 0.3, 'w': 0.15, 'h': 0.80})
    assert r2 == 'following' and v2 > 0.2


def test_size_hold_keeps_stopped_until_resume():
    """迟滞：停过之后框缩到 83%（> resume 0.80）还不能走，防止阈值上下抖。"""
    v, _, r, _ = _decide(person={'x': 0.4, 'y': 0.3, 'w': 0.2, 'h': 0.83},
                         size_hold=True)
    assert v == 0.0 and r == 'too_close'
    # 节点在框缩到 80% 以下时把 size_hold 清掉，这里模拟清掉后的状态
    v2, _, r2, _ = _decide(person={'x': 0.4, 'y': 0.3, 'w': 0.2, 'h': 0.60},
                           size_hold=False)
    assert v2 > 0 and r2 == 'following'


def test_tiny_box_does_not_chase():
    v, w, r, _ = _decide(person={'x': 0.48, 'y': 0.3, 'w': 0.04, 'h': 0.20})
    assert (v, w, r) == (0.0, 0.0, 'too_far')


def test_far_box_keeps_rotation():
    v, w, r, _ = _decide(person={'x': 0.70, 'y': 0.3, 'w': 0.05, 'h': 0.20}, left=9.0, right=9.0)
    assert v == 0.0 and w < 0 and r == 'too_far'


# ---------------------------------------------------------------- 转向
def test_turn_gain_is_faster_than_v1():
    """偏右 15% → 角速度应明显大于第一版（2.2×0.12=0.26）。"""
    _, w, _, _ = _decide(person={'x': 0.65, 'y': 0.3, 'w': 0.20, 'h': 0.55})
    assert w < -0.35


def test_turn_clamped_at_wmax():
    _, w, _, _ = _decide(person={'x': 0.85, 'y': 0.3, 'w': 0.10, 'h': 0.55})
    assert w == pytest.approx(-P['wmax'])


def test_centered_no_turn():
    _, w, r, _ = _decide()
    assert w == 0.0 and r == 'following'


def test_turn_blocked_by_wall():
    # 注意：x=0.85 属于"框贴到画面边"→ 现在走 facing（原地转向追框），不再受侧向门限
    _, w, r, _ = _decide(person={'x': 0.62, 'y': 0.3, 'w': 0.10, 'h': 0.55}, right=0.20)
    assert (w, r) == (0.0, 'side_blocked')


# ---------------------------------------------------------------- 避障（走廊）
def test_corridor_obstacle_stops():
    v, w, r, _ = _decide(hard=0.30)
    assert (v, w, r) == (0.0, 0.0, 'obstacle_stop')


def test_corridor_obstacle_slows():
    v, _, r, _ = _decide(hard=0.50)
    assert r == 'obstacle_slow' and v == pytest.approx(P['vmax'] * 0.65)


def test_obstacle_beyond_window_free():
    v, _, r, _ = _decide(hard=0.90)
    assert r == 'following' and v == pytest.approx(P['vmax'])


# ---------------------------------------------------------------- 丢人
def test_lost_stops():
    assert _decide(lost=1.2)[:3] == (0.0, 0.0, 'target_lost')


def test_gone_aborts():
    assert _decide(lost=3.5)[:3] == (0.0, 0.0, 'target_gone')


# ---------------------------------------------------------------- 雷达几何
def _scan(points, n=360):
    """points = [(角度°, 距离)]，其余方向 9 m 空旷。"""
    rng = [30.0] * n   # 30 m 超量程 = 空旷
    inc = 360.0 / n
    for deg, r in points:
        idx = int(round((deg + 180.0) / inc)) % n
        rng[idx] = r
    return rng


def test_door_frame_not_in_corridor():
    """门框在左右两扇（±35°、0.5 m）→ 横向 0.29 m 在车身走廊外 → 不算障碍。"""
    ranges = _scan([(35, 0.50), (-35, 0.50)])
    hard, slow, left, right, _, _ = scan_metrics(ranges, math.radians(-180), math.radians(1), P)
    assert hard == math.inf
    assert left == pytest.approx(0.50) and right == pytest.approx(0.50)


def test_center_obstacle_in_corridor():
    ranges = _scan([(0, 0.30)])
    hard, _, _, _, _, _ = scan_metrics(ranges, math.radians(-180), math.radians(1), P)
    assert hard == pytest.approx(0.30)


def test_corridor_uses_lateral_distance():
    """正前方 0.6 m 处的点横向 0 m → 在走廊内；横向 0.4 m 的点（±34°）→ 不在。"""
    ranges = _scan([(0, 0.60), (34, 0.60), (-34, 0.60)])
    hard, slow, _, _, _, _ = scan_metrics(ranges, math.radians(-180), math.radians(1), P)
    assert hard == pytest.approx(0.60)     # 只有正前方那个点算
    # 减速区 ±0.30 m：34° 处横向 0.34 m，仍然在外
    assert slow == pytest.approx(0.60)


def test_side_sectors_exclude_front():
    ranges = _scan([(10, 0.40), (50, 0.80), (-50, 1.20)])
    _, _, left, right, _, _ = scan_metrics(ranges, math.radians(-180), math.radians(1), P)
    assert left == pytest.approx(0.80) and right == pytest.approx(1.20)


def test_invalid_ranges_ignored():
    ranges = _scan([(0, 0.0), (0, 99.0)])
    hard, _, _, _, _, _ = scan_metrics(ranges, math.radians(-180), math.radians(1), P)
    assert hard == math.inf


# ---------------------------------------------------------------- 第三版：侧向对准 / 后退
def test_faces_person_walking_past():
    """人从车旁边走过（框偏到画面右边缘）→ 原地转向，不前进。"""
    v, w, r, _ = _decide(person={'x': 0.80, 'y': 0.2, 'w': 0.15, 'h': 0.55})
    assert (v, r) == (0.0, 'facing')
    assert w < 0          # 人在画面右侧 → 右转（ω<0）

    v2, w2, r2, _ = _decide(person={'x': 0.02, 'y': 0.2, 'w': 0.15, 'h': 0.55})
    assert (v2, r2) == (0.0, 'facing')
    assert w2 > 0


def test_backs_up_when_face_to_face():
    """框 95%（人贴到车头）→ 后退，车尾空着就退。"""
    v, _, r, _ = _decide(person={'x': 0.4, 'y': 0.1, 'w': 0.2, 'h': 0.95},
                         back_hold=True, rear_hard=3.0, rear_slow=3.0)
    assert r == 'backing' and v < 0
    assert abs(v) <= P['back_vmax'] + 1e-9


def test_backward_slows_near_rear_obstacle():
    """车尾 0.40 m 有东西 → 还是退，但只退一点点。"""
    v, _, r, _ = _decide(person={'x': 0.4, 'y': 0.1, 'w': 0.2, 'h': 0.95},
                         back_hold=True, rear_hard=0.40, rear_slow=0.40)
    assert r == 'backing' and -P['back_vmax'] < v < 0


def test_rear_too_close_turns_in_place():
    """车尾 0.15 m 有东西 → 不许退，只原地转向（用户要求）。"""
    v, w, r, _ = _decide(person={'x': 0.52, 'y': 0.1, 'w': 0.1, 'h': 0.95},
                         back_hold=True, rear_hard=0.15, rear_slow=0.15)
    assert (v, r) == (0.0, 'rear_blocked')
    assert abs(w) > 0


def test_scan_metrics_rear_corridor():
    """车尾正后方 0.40 m 有东西 → rear_hard ≈ 0.40，车头仍是空的。"""
    ranges = _scan([(180, 0.40)])          # 扫描角 180° = 车尾正后方
    hard, _, _, _, rear_hard, rear_slow = scan_metrics(
        ranges, math.radians(-180), math.radians(1), P)
    assert hard == math.inf
    assert abs(rear_hard - 0.40) < 0.02
    assert abs(rear_slow - 0.40) < 0.02


# ------------------------------------------------ 第三版补：底盘硬限（2026-11-21 实机踩坑）
def test_wmax_below_base_bridge_hard_limit():
    """base_bridge 的 manual_velocity 硬限 |ω| ≤ 0.75，超了整帧被丢 → 车不动。"""
    assert DEFAULTS['wmax'] <= 0.70


def test_limit_pair_caps_omega():
    v, w = limit_pair(0.0, 0.9)
    assert abs(w) <= 0.74 and v == 0.0


def test_limit_pair_shrinks_v_to_fit_wheel_envelope():
    """v=0.25 + ω=0.7 时单轮 0.25+0.092=0.342 > 0.30 → 必须收线速度，不能整帧被丢。"""
    v, w = limit_pair(0.25, 0.70)
    assert v < 0.25
    wheel = abs(v) + abs(w) * DEFAULTS['track_m'] / 2.0
    assert wheel <= DEFAULTS['max_wheel_mps'] + 1e-9


def test_limit_pair_keeps_straight_speed():
    v, w = limit_pair(0.25, 0.0)
    assert v == pytest.approx(0.25) and w == 0.0


def test_limit_pair_reverse_ok():
    v, w = limit_pair(-0.12, 0.70)
    wheel = abs(v) + abs(w) * DEFAULTS['track_m'] / 2.0
    assert v < 0 and wheel <= DEFAULTS['max_wheel_mps'] + 1e-9


# ---------------------------------------------- 第四版：穿门框（2026-11-21）
def _scan_cart(points):
    """points=[(lon, lat)] → 360 点扫描（其余 30 m 空旷），供 passage_metrics 用。

    ⚠️ 别叫 `_scan`：文件上面那个 `_scan` 是 (角度, 距离) 的老helper，同名会覆盖它。
    """
    n = 360
    rng = [30.0] * n
    for lon, lat in points:
        deg = math.degrees(math.atan2(lat, lon))
        i = int(round((deg + 180.0))) % 360
        rng[i] = math.hypot(lon, lat)
    return rng


def _gate_scan(half_gap_m, lon_m=0.70):
    return _scan_cart([(lon_m, half_gap_m), (lon_m, -half_gap_m)])


def test_passage_metrics_finds_doorway():
    gap, center = passage_metrics(_gate_scan(0.45), -math.pi, 2 * math.pi / 360, P)
    assert gap == pytest.approx(0.90, abs=0.06)
    assert center == pytest.approx(0.0, abs=0.03)


def test_passage_metrics_offset_doorway_center():
    # 门洞中心偏 20 cm：左边界 0.70、右边界 -0.30（净宽 1.00，正好在门宽上限内）
    _, center = passage_metrics(_scan_cart([(0.7, 0.70), (0.7, -0.30)]),
                                -math.pi, 2 * math.pi / 360, P)
    assert center == pytest.approx(0.20, abs=0.05)


def test_passage_metrics_ignores_wide_hall():
    assert passage_metrics(_gate_scan(0.70), -math.pi, 2 * math.pi / 360, P) is None


def test_passage_metrics_ignores_too_narrow():
    assert passage_metrics(_gate_scan(0.26), -math.pi, 2 * math.pi / 360, P) is None


def test_passage_metrics_one_side_only_is_not_gate():
    assert passage_metrics(_scan_cart([(0.7, 0.45)]),
                           -math.pi, 2 * math.pi / 360, P) is None


def test_gate_aligns_before_entering():
    v, w, r, _ = _decide(person={'x': 0.44, 'y': 0.3, 'w': 0.12, 'h': 0.55},
                         gate_hold=True, gate_center=0.20)
    # 门洞中心偏左（y>0）→ 要**左转**（ω>0），符号和跟框那条相反
    assert v == 0.0 and r == 'gate_align' and w > 0


def test_gate_passes_slowly_when_aligned():
    v, w, r, _ = _decide(person={'x': 0.46, 'y': 0.3, 'w': 0.08, 'h': 0.55},
                         gate_hold=True, gate_center=0.02)
    assert r == 'gate_pass' and 0.0 < v <= P['gate_vmax']
    assert abs(w) <= P['gate_wmax']


def test_gate_ignores_person_off_center():
    """人走太快、框跑到画面边上时，门里不许跟着转（这就是卡门框的根因）。"""
    v, w, r, _ = _decide(person={'x': 0.70, 'y': 0.3, 'w': 0.14, 'h': 0.55},
                         gate_hold=True, gate_center=0.02)
    assert r == 'gate_pass' and abs(w) <= P['gate_wmax']


def test_gate_stops_when_person_blocks_door():
    v, _, r, _ = _decide(person={'x': 0.4, 'y': 0.3, 'w': 0.3, 'h': 0.90},
                         gate_hold=True, gate_center=0.0)
    assert v == 0.0 and r == 'too_close'


def test_gate_obeys_front_obstacle():
    v, w, r, _ = _decide(gate_hold=True, gate_center=0.0, hard=0.30)
    assert (v, w) == (0.0, 0.0) and r == 'obstacle_stop'


def test_align_first_when_obstacle_near():
    """车头 0.8 m 内有东西 → 先对准再走（门槛 30% 收紧到 12%），防切内角撞门框。"""
    person = {'x': 0.62, 'y': 0.3, 'w': 0.14, 'h': 0.55}   # err≈0.19
    _, _, r_far, _ = _decide(person=person, hard=5.0, slow=5.0)
    _, _, r_near, _ = _decide(person=person, hard=0.75, slow=0.75)
    assert r_far == 'following'
    assert r_near == 'facing'


# ---------------------------------------- 第五版：门内丢人 → 先出门再找人（2026-11-21）
def test_gate_exit_keeps_driving_when_person_lost():
    """人在门口先消失：门洞还在 → 车继续往外走，不停在门框中间。"""
    v, w, r, _ = _decide(person=None, lost=1.5, gate_hold=True, gate_center=0.02,
                         gate_exit=True)
    assert r == 'gate_exit' and 0.0 < v <= P['gate_vmax'] and abs(w) <= P['gate_wmax']


def test_gate_exit_aligns_first():
    v, w, r, _ = _decide(person=None, lost=1.5, gate_hold=True, gate_center=0.15,
                         gate_exit=True)
    assert v == 0.0 and r == 'gate_align' and w > 0


def test_gate_exit_respects_front_obstacle():
    v, w, r, _ = _decide(person=None, lost=1.5, hard=0.30, gate_hold=True,
                         gate_center=0.0, gate_exit=True)
    assert (v, w) == (0.0, 0.0) and r == 'obstacle_stop'


def test_search_turns_right_when_person_was_right():
    """人在画面右边走掉 → 出门后原地**右转**（ω<0）找他。"""
    v, w, r, _ = _decide(person=None, lost=1.5, search=True, lost_side=1.0)
    assert r == 'search' and v == 0.0 and w < 0


def test_search_turns_left_when_person_was_left():
    v, w, r, _ = _decide(person=None, lost=1.5, search=True, lost_side=-1.0)
    assert r == 'search' and v == 0.0 and w > 0


def test_search_without_side_does_not_spin():
    v, w, r, _ = _decide(person=None, lost=1.5, search=True, lost_side=0.0)
    assert r == 'search' and v == 0.0 and w == 0.0


def test_lost_without_gate_still_stops():
    """不在门口丢人 → 保持原行为：直接停，不自己乱走。"""
    v, w, r, _ = _decide(person=None, lost=1.5)
    assert (v, w, r) == (0.0, 0.0, 'no_target')


def test_gate_exit_keeps_driving_after_gate_signal_gone():
    """2026-11-22 用户报「出门口没完全出来就转向」的另一半：车快出去时前雷达
    已经看不到门洞（gate_hold 掉），旧逻辑直接变成 no_target **停在门框里**，
    门线永远退不到车尾后面 → 要么停死、要么超时后乱转。
    现在只要还在出门状态就继续沿门中心低速外走，直到 door_clear。
    """
    v, w, r, _ = _decide(person=None, lost=1.5, gate_exit=True, gate_hold=False,
                         gate_center=0.0)
    assert r == 'gate_exit' and v > 0.10
    # 车头走廊真被堵住时仍然停（不能为了出门撞上去）
    v, w, r, _ = _decide(person=None, lost=1.5, gate_exit=True, gate_hold=False,
                         hard=0.20)
    assert r == 'obstacle_stop' and v == 0.0


def test_person_back_cancels_exit():
    """人一回到画面里，立刻恢复正常跟随（不再走 gate_exit/search）。"""
    v, w, r, _ = _decide(person={'x': 0.42, 'y': 0.3, 'w': 0.15, 'h': 0.55},
                         lost=0.0, gate_hold=True, gate_center=0.02, gate_exit=True)
    assert r == 'gate_pass'


# -------------------- 底盘一致性：轮距 / 单轮包络（2026-11-21 实机踩坑后钉死）
def test_track_m_matches_chassis():
    """`track_m` 必须等于底盘真实轮距，否则 limit_pair 算小 → 整帧被丢 → 车不动。

    2026-11-21 实机：跟随里 `v=0.228 w=0.476` 被 base_bridge 判超限丢弃 11 次
    （页面显示"跟随中"车不动）。当时 follow_core 还写着换轮前的旧轮距 0.262，
    而 `BaseBridgeCore().track_m` 已是 **0.317**。
    """
    from fwr_bringup.base_bridge_core import BaseBridgeCore
    assert P['track_m'] == BaseBridgeCore().track_m


def test_limited_pair_always_accepted_by_chassis():
    """limit_pair 出来的任何一条指令，底盘都必须接受（否则就是"显示在转但不动"）。"""
    from fwr_bringup.base_bridge_core import BaseBridgeCore, manual_velocity
    track = BaseBridgeCore().track_m
    cap = 0.30                     # base_bridge 的 manual_max_speed_mps
    for v in (0.0, 0.12, 0.25, 0.29, -0.12):
        for w in (0.0, 0.1, 0.3, 0.45, 0.7, -0.7):
            lv, lw = limit_pair(v, w, P)
            assert manual_velocity(lv, lw, track, cap, True), (v, w, lv, lw)


# ------------------------------------------ 第六版：整车出门（前后雷达，2026-11-21）
def test_door_metrics_sees_jambs_beside_body():
    """门框点在车身两侧（纵向跨过车身）→ 门线还在身上，不算出去。"""
    front, rear, gap = door_metrics(
        _scan_cart([(0.30, 0.45), (-0.35, -0.45)]),
        -math.pi, 2 * math.pi / 360, P)
    assert front == pytest.approx(0.30, abs=0.05) and gap == pytest.approx(0.90, abs=0.08)
    assert not door_clear(front, P)


def test_door_metrics_front_lidar_past_but_body_inside():
    """用户报的场景：车头刚过门线（门框点都在车后 0.1 m），**车尾还在门里** →

    前雷达已经看不到门了，但门线还没退到车尾（-0.23-0.06=-0.29）后面 → 不能转身。
    """
    front, _, _ = door_metrics(_scan_cart([(-0.10, 0.45), (-0.12, -0.45)]),
                               -math.pi, 2 * math.pi / 360, P)
    assert front == pytest.approx(-0.10, abs=0.05)
    assert not door_clear(front, P)          # 还没整车出去


def test_door_metrics_cleared_when_line_behind_swept_circle():
    """门线退到"原地转身扫掠圆 + 10 cm"之外（约车后 0.415 m）→ 整车出去了。"""
    front, _, _ = door_metrics(_scan_cart([(-0.46, 0.45), (-0.48, -0.45)]),
                               -math.pi, 2 * math.pi / 360, P)
    assert door_clear(front, P)


def test_door_metrics_none_in_open_space():
    assert door_metrics(_scan_cart([(3.0, 0.45), (3.0, -0.45)]),
                        -math.pi, 2 * math.pi / 360, P) is None
    assert door_metrics(_scan_cart([(0.5, 0.45)]),
                        -math.pi, 2 * math.pi / 360, P) is None
    assert door_clear(None, P)          # 没有门线 → 视为已出去


def test_door_clear_uses_swept_radius_and_margin():
    """2026-11-22 用户第二次报"出门口没完全出来就转向"。

    旧判据只减车尾（-0.29 m），但原地转身时**车尾角点**扫到 -hypot(0.23, 0.215)
    = -0.315 m，比车尾还靠后 8.5 cm → 门线在 -0.30 时转身就蹭门框。
    现在必须退到 -(扫掠半径 + margin) 之后。
    """
    sweep = door_swept_radius(P)
    assert sweep == pytest.approx(math.hypot(0.23, 0.215), abs=1e-6)
    thresh = -(sweep + P['door_clear_margin_m'])
    assert thresh == pytest.approx(-0.415, abs=0.01)
    assert not door_clear(thresh + 0.01, P)
    assert door_clear(thresh - 0.01, P)
    # 旧阈值专门钉死：-0.29 / -0.30 这种"车尾刚过门线"的位置**绝不允许转身**
    assert not door_clear(-0.29, P)
    assert not door_clear(-0.30, P)
    assert not door_clear(-0.35, P)


# ------------------------------- 第七版：PD 转向 + 左右摆自动阻尼（2026-11-21 晚）
def test_steer_deadband_zero_output():
    """画面中间 ±6% 内不转（YOLO 框每帧抖 1~2%，死区太小就是来回摆）。"""
    for e in (-0.05, -0.01, 0.0, 0.03, 0.05):
        assert steer(e, 0.0, P) == 0.0


def test_steer_sign_and_gain():
    """人偏右 → 右转（ω<0）；偏左 → 左转（ω>0）。"""
    assert steer(0.20, 0.0, P) < 0.0
    assert steer(-0.20, 0.0, P) > 0.0


def test_steer_capped_while_following():
    """正常跟随 ω ≤ steer_wmax；偏到画面边上（要原地对准）才允许用满 wmax。"""
    assert abs(steer(0.28, 0.0, P, P['steer_wmax'])) <= P['steer_wmax'] + 1e-9
    assert abs(steer(0.45, 0.0, P, P['wmax'])) == pytest.approx(P['wmax'])


def test_steer_damping_only_reduces_never_reverses():
    """阻尼项只许"提前收力"，不许把转向拽反。

    踩过的坑：不夹的话 `-kd·err_dot` 在 err=+0.10、err_dot=-0.5 时输出 **+0.35**
    （车往反方向转），跟着直行的人照样左右摆。
    """
    base = steer(0.10, 0.0, P)
    damped = steer(0.10, -0.5, P)          # 偏差正在变小 → 该收力
    assert 0.0 < abs(damped) < abs(base)   # 变小，但符号不变
    assert math.copysign(1, damped) == math.copysign(1, base)
    # 偏差在变大 → 允许略微加点力，但不超过 P 项的 30%
    ahead = steer(0.10, +0.5, P)
    assert abs(ahead) <= 1.30 * abs(base) + 1e-9


def test_osc_guard_damps_when_oscillating_and_recovers():
    """左右摆 → 自动收增益到 35%；稳住后慢慢恢复。"""
    g, dt = OscGuard(), 0.05
    for i in range(120):                       # 8 Hz 正负交替
        w = 0.25 if (i // 5) % 2 == 0 else -0.25
        g.update(w, dt)
    assert g.gain < 0.60                       # 已经明显收力（下限 0.35，恢复期会略回一点）
    for _ in range(100):                       # 稳定直行 5 s
        g.update(0.25, dt)
    assert g.gain > 0.9                         # 基本恢复
    # 小幅抖动（< min_w）不算"摆"，不该误触发
    g2 = OscGuard()
    for i in range(120):
        g2.update(0.02 if i % 2 else -0.02, dt)
    assert g2.gain == pytest.approx(1.0)


# ---------------------------------------------------------------- 雷达接管：别再跟着"动的家具"（2026-11-26）

def test_pick_lidar_track_requires_camera_backing():
    """**只认相机新鲜背书的轨迹**，没背书的一律不认（不许"退而求其次"）。

    2026-11-26 现场图片：画面里只有货箱/船壳，提示却写着「雷达接管中，正朝你过去」
    —— 货箱曾被永久标成"相机背书"，于是被当成人一路跟过去。
    """
    from fwr_bringup.follow_core import pick_lidar_track
    # 推车正好在中线、人被挤到偏右：以前会选推车，现在直接 None（宁可不跟）
    cands = [
        {'id': 1, 'bearing': 0.00, 'r': 1.6, 'cam': False, 'v': 0.6},
        {'id': 2, 'bearing': 0.30, 'r': 1.8, 'cam': False, 'v': 0.4},
    ]
    assert pick_lidar_track(cands, bearing=0.0) is None
    # 加上真背书的人 → 选人
    cands.append({'id': 3, 'bearing': 0.12, 'r': 1.7, 'cam': True, 'cam_age_s': 0.1,
                  'static': False, 'v': 0.4})
    assert pick_lidar_track(cands, bearing=0.0)['id'] == 3
    # 背书过期（cam_age_s 超 2 s）也不认
    assert pick_lidar_track([{'id': 4, 'bearing': 0.0, 'r': 1.6, 'cam': False,
                              'cam_age_s': 5.0, 'static': False, 'v': 0.3}],
                            bearing=0.0) is None
    # 静止轨迹（家具）永远不认
    assert pick_lidar_track([{'id': 5, 'bearing': 0.0, 'r': 1.6, 'cam': True,
                              'cam_age_s': 0.0, 'static': True, 'v': 0.0}],
                            bearing=0.0) is None


def test_pick_lidar_track_rejects_fast_blob():
    """速度超过 `lidar_max_speed` 的轨迹一律不认（甩动的门/被拖的车）。"""
    from fwr_bringup.follow_core import pick_lidar_track
    cands = [{'id': 3, 'bearing': 0.02, 'r': 1.5, 'cam': True, 'v': 3.2}]
    assert pick_lidar_track(cands, bearing=0.0) is None


def test_reacquire_rejects_static_and_unbacked():
    """续认一律不许换成静止物或没背书的轨迹。

    ⚠️ 2026-11-26 的教训：放宽"背书过的静止轨迹"后，人车旁边那堆货箱/船壳
    （曾被误背书）被续认接手，车直接朝它开过去。宁可**原地转着找人**，
    也不许把目标换成一件家具。
    """
    from fwr_bringup.follow_core import reacquire_track
    last_b, last_r = 0.10, 1.60
    backed_static = [{'id': 9, 'bearing': 0.12, 'r': 1.62, 'cam': True,
                      'cam_age_s': 0.1, 'static': True, 'v': 0.0}]
    assert reacquire_track(backed_static, last_b, last_r) is None
    unbacked = [{'id': 10, 'bearing': 0.12, 'r': 1.62, 'cam': False,
                 'cam_age_s': None, 'static': False, 'v': 0.2}]
    assert reacquire_track(unbacked, last_b, last_r) is None
    # 正在走动、背书新鲜 → 可以续
    walking = [{'id': 11, 'bearing': 0.12, 'r': 1.62, 'cam': True,
                'cam_age_s': 0.2, 'static': False, 'v': 0.9}]
    assert reacquire_track(walking, last_b, last_r)['id'] == 11


def test_reacquire_gate_is_tight():
    """续认门限 **15°/0.60 m**：放宽到 28°/1.2 m 会认到旁边的东西（实测栽过）。"""
    from fwr_bringup.follow_core import reacquire_track
    near = [{'id': 11, 'bearing': 0.22, 'r': 1.95, 'cam': True,
             'cam_age_s': 0.1, 'static': False, 'v': 1.0}]     # 12.6° / 0.45 m
    assert reacquire_track(near, 0.10, 1.50)['id'] == 11
    far = [{'id': 12, 'bearing': 0.22, 'r': 2.60, 'cam': True,
            'cam_age_s': 0.1, 'static': False, 'v': 1.1}]      # 1.1 m → 太远
    assert reacquire_track(far, 0.10, 1.50) is None


def test_lidar_search_cmd_only_turns():
    """轨迹短暂丢失时**原地只转**（v 恒 0），并按 lidar_wmax 限幅。"""
    from fwr_bringup.follow_core import DEFAULTS, lidar_search_cmd
    v, w, reason = lidar_search_cmd(-0.40)
    assert v == 0.0 and reason == 'lidar_search'
    assert abs(w) <= DEFAULTS['lidar_wmax'] + 1e-9 and w < 0      # 目标在右 → 右转（负）
    assert lidar_search_cmd(DEFAULTS['lidar_bearing_deadband'] * 0.5)[:2] == (0.0, 0.0)
    assert lidar_search_cmd(None)[:2] == (0.0, 0.0)
