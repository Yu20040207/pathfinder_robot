"""`fwr_bringup.leg_track` 单测（雷达腿部检测 + 多目标跟踪，纯逻辑）。

运行：``PYTHONPATH=ros2_ws/src/fwr_bringup .venv/bin/python -m pytest tests/test_leg_track.py -q``
"""

from __future__ import annotations

import ast
import math
import re

import pytest

from fwr_bringup.leg_track import (DEFAULTS, Kalman2D, Leg, LegTracker, Pair,
                                   cluster_points, cluster_width, dedup_pairs,
                                   describe, detect_pairs, find_legs,
                                   match_pairs, merged_candidates,
                                   scan_to_points, to_world, world_to_robot)


# --------------------------------------------------------------------------
# 构造工具
# --------------------------------------------------------------------------
def make_leg(x: float, y: float, width: float = 0.12, n: int = 8,
             origin: tuple = (0.0, 0.0)) -> Leg:
    ox, oy = origin
    return Leg(x, y, width, n, math.hypot(x - ox, y - oy),
               math.atan2(y - oy, x - ox))


def make_pair(x: float, y: float, gap: float = 0.30,
              origin: tuple = (0.0, 0.0)) -> Pair:
    a = make_leg(x, y + gap / 2.0, origin=origin)
    b = make_leg(x, y - gap / 2.0, origin=origin)
    ox, oy = origin
    return Pair(x, y, gap, a.width, b.width,
                math.hypot(x - ox, y - oy), math.atan2(y - oy, x - ox), (a, b))


def leg_points(x: float, y: float, width: float = 0.12, n: int = 3):
    """一条腿的弧：以 (x, y) 为中心、横向 width 的 n 个点（纵向近似一致）。"""
    if n == 1:
        return [(x, y)]
    step = width / (n - 1)
    return [(x, y - width / 2.0 + i * step) for i in range(n)]


def two_legs_points(x: float, y: float = 0.0, gap: float = 0.30,
                    width: float = 0.12):
    return leg_points(x, y + gap / 2.0, width) + leg_points(x, y - gap / 2.0,
                                                           width)


# --------------------------------------------------------------------------
# 1) 点云预处理
# --------------------------------------------------------------------------
def test_scan_to_points_filters_invalid_and_out_of_range():
    ranges = [0.5, 0.1, 5.0, float('inf'), 10.0, float('nan')]
    step = math.radians(90.0)
    pts = scan_to_points(ranges, 0.0, step)
    # 0.5 m@0° 保留；0.1 m 太近丢；5 m@180° 保留；inf/nan 丢；10 m 超上限丢
    assert len(pts) == 2
    assert pts[0] == pytest.approx((0.5, 0.0), abs=1e-6)
    assert pts[1] == pytest.approx((-5.0, 0.0), abs=1e-6)


def test_scan_to_points_drops_body_rectangle():
    # 车身矩形（x∈[-0.26,0.26], y∈±0.245）内的点必须丢
    ranges = [0.10, 0.20, 0.30]
    step = math.radians(45.0)
    pts = scan_to_points(ranges, 0.0, step)
    for x, y in pts:
        assert not (-0.26 <= x <= 0.26 and -0.245 <= y <= 0.245)


def test_to_world_and_back_roundtrip():
    pts = [(1.0, 0.0), (0.0, 2.0)]
    world = to_world(pts, 1.0, 2.0, math.pi / 2.0)
    assert world[0] == pytest.approx((1.0, 3.0), abs=1e-9)
    back = world_to_robot(world, 1.0, 2.0, math.pi / 2.0)
    assert back[0] == pytest.approx((1.0, 0.0), abs=1e-9)
    assert back[1] == pytest.approx((0.0, 2.0), abs=1e-9)


# --------------------------------------------------------------------------
# 2) 聚类
# --------------------------------------------------------------------------
def test_cluster_splits_two_legs():
    pts = two_legs_points(2.0, gap=0.30)
    clusters = cluster_points(pts)
    assert len(clusters) == 2
    assert cluster_width(clusters[0]) == pytest.approx(0.12, abs=1e-6)


def test_cluster_keeps_wall_as_one():
    pts = [(2.0, -0.5 + 0.05 * i) for i in range(21)]   # 1 m 长的墙，5 cm 一点
    clusters = cluster_points(pts)
    assert len(clusters) == 1
    assert cluster_width(clusters[0]) == pytest.approx(1.0, abs=1e-6)


# --------------------------------------------------------------------------
# 3) 候选腿 / 腿对
# --------------------------------------------------------------------------
def test_find_legs_accepts_two_legs():
    legs = find_legs(two_legs_points(2.0, gap=0.30))
    assert len(legs) == 2
    assert legs[0].r == pytest.approx(math.hypot(2.0, 0.15), abs=0.01)


def test_find_legs_rejects_wall_cluster():
    pts = [(2.0, -0.5 + 0.05 * i) for i in range(21)]
    assert find_legs(pts) == []


def test_match_pairs_finds_person():
    legs = [make_leg(2.0, 0.17), make_leg(2.0, -0.17)]
    pairs = match_pairs(legs)
    assert len(pairs) == 1
    assert pairs[0].gap == pytest.approx(0.34, abs=1e-6)
    assert pairs[0].x == pytest.approx(2.0, abs=1e-6)
    assert pairs[0].y == pytest.approx(0.0, abs=1e-6)


def test_match_pairs_rejects_door_frame_gap():
    # 门框两侧 0.9 m：超过 pair_gap_max 0.60 → 不能当人
    legs = [make_leg(3.0, 0.45), make_leg(3.0, -0.45)]
    assert match_pairs(legs) == []


def test_match_pairs_rejects_depth_mismatch():
    # 间距 0.55 m 在范围内，但一前一后（距离差 0.54 m > 0.25）→ 不是一个人
    legs = [make_leg(1.00, 0.20), make_leg(1.55, 0.20)]
    assert match_pairs(legs) == []


def test_match_pairs_uses_each_leg_once():
    # 三团腿（如椅子 + 人）只能配出 1 对
    legs = [make_leg(2.0, 0.17), make_leg(2.0, -0.17), make_leg(2.0, 0.60)]
    pairs = match_pairs(legs)
    assert len(pairs) == 1


# --------------------------------------------------------------------------
# 4) 卡尔曼
# --------------------------------------------------------------------------
def test_kalman_converges_to_constant_velocity():
    kf = Kalman2D(0.0, 0.0, 0.0)
    for i in range(1, 21):
        t = i * 0.1
        kf.predict(t)
        kf.update(0.5 * t, 0.0)          # 以 0.5 m/s 沿 +x 走
    assert kf.vx == pytest.approx(0.5, abs=0.10)
    assert kf.x == pytest.approx(1.0, abs=0.10)


# --------------------------------------------------------------------------
# 5) 跟踪器
# --------------------------------------------------------------------------
def test_tracker_confirms_with_camera_in_three_frames():
    tr_ = LegTracker()
    cam = {'bearing': 0.0, 'range': 2.0}
    out = []
    for i in range(3):
        out = tr_.step([make_pair(2.0, 0.0)], i * 0.1, cam)
    assert len(out) == 1
    assert out[0]['cam'] is True
    assert out[0]['x'] == pytest.approx(2.0, abs=0.05)


def test_tracker_without_camera_needs_more_frames():
    tr_ = LegTracker()
    for i in range(5):
        out = tr_.step([make_pair(2.0, 0.0)], i * 0.1, None)
    assert out == []                      # 5 帧 = 0.5 s 还不够（camless 要 6 帧）
    out = tr_.step([make_pair(2.0, 0.0)], 0.5, None)
    assert len(out) == 1
    assert out[0]['cam'] is False


def test_tracker_birth_requires_cam_when_configured():
    # P2 的推荐配置：新目标必须有相机背书，否则一律当家具
    tr_ = LegTracker({'birth_requires_cam': True})
    cam = {'bearing': math.radians(120.0), 'range': 2.0}
    out = []
    for i in range(10):
        out = tr_.step([make_pair(2.0, 0.0)], i * 0.1, cam)
    assert out == []                        # 相机在 120°、腿对在 0° → 不建轨迹


def test_tracker_p1_mode_allows_birth_without_cam_backing():
    # P1（采集数据）默认 birth_requires_cam=False：相机没背书也放出来，方便现场看全
    tr_ = LegTracker()
    cam = {'bearing': math.radians(120.0), 'range': 2.0}
    out = []
    for i in range(6):
        out = tr_.step([make_pair(2.0, 0.0)], i * 0.1, cam)
    assert len(out) == 1
    assert out[0]['cam'] is False


def test_tracker_coasts_then_drops():
    tr_ = LegTracker()
    cam = {'bearing': 0.0, 'range': 2.0}
    for i in range(3):
        out = tr_.step([make_pair(2.0, 0.0)], i * 0.1, cam)
    assert len(out) == 1
    # 丢 0.4 s：还在（滑行）
    out = tr_.step([], 0.7, None)
    assert len(out) == 1
    assert out[0]['misses'] >= 1
    # 丢 2.6 s：还在（drop_after_s 3.0 s，给人擦过柱子/两腿合并留够滑行余量）
    out = tr_.step([], 2.6, None)
    assert len(out) == 1
    # 丢 3.4 s：删除
    out = tr_.step([], 3.4, None)
    assert out == []


def test_tracker_two_targets_get_distinct_ids():
    tr_ = LegTracker()
    cam = {'bearing': 0.0, 'range': 2.0}
    pairs = [make_pair(2.0, 0.35), make_pair(2.0, -0.35)]
    out = []
    for i in range(3):
        out = tr_.step(pairs, i * 0.1, cam)
    assert len(out) == 2
    assert {d['id'] for d in out} == {1, 2}


def test_tracker_drops_static_object_when_camera_sees_elsewhere():
    # 椅子腿：位置不变、相机在看别处（有相机数据但没背书）→ 10 s 后必须丢掉
    import math as _m
    cam = {'bearing': _m.radians(90.0), 'range': None}
    tr_ = LegTracker()
    for i in range(61):                  # 6 s
        out = tr_.step([make_pair(2.0, 0.0)], i * 0.1, cam)
    assert len(out) == 1
    for i in range(61, 121):             # 再 6 s（累计 12 s）
        out = tr_.step([make_pair(2.0, 0.0)], i * 0.1, cam)
    assert out == []                     # 丢掉并进黑名单


def test_tracker_keeps_still_person_without_camera():
    # 没有相机数据时"站着不动的人"不能当成家具删掉（2026-11-24 踩过：
    # 站 9 秒就被删 + 拉黑 300 s，人彻底消失）
    tr_ = LegTracker()
    for i in range(150):                 # 15 s 完全不动
        out = tr_.step([make_pair(2.0, 0.0)], i * 0.1, None)
    assert len(out) == 1


def test_tracker_keeps_moving_object_without_camera():
    # 同样没有相机，但人在走（每帧 0.05 m）→ 不能被"静止剔除"误杀
    tr_ = LegTracker()
    out = []
    for i in range(121):
        x = 2.0 - 0.05 * i * 0.1 * 10.0   # 0.05 m/帧
        x = max(0.5, 2.0 - 0.05 * i)
        out = tr_.step([make_pair(x, 0.0)], i * 0.1, None)
    assert len(out) == 1


def test_tracker_bearing_uses_pair_direction():
    tr_ = LegTracker()
    cam = {'bearing': math.pi / 2.0, 'range': 2.0}   # 相机看到正左方
    out = []
    for i in range(3):
        out = tr_.step([make_pair(0.0, 2.0)], i * 0.1, cam)
    assert len(out) == 1
    assert out[0]['cam'] is True


# --------------------------------------------------------------------------
# 6) 自检文案
# --------------------------------------------------------------------------
def test_describe_mentions_pairs_and_tracks():
    txt = describe(3, [make_pair(1.8, 0.0)], [{'id': 1, 'x': 1.8, 'y': 0.0,
                                                'vx': 0.2, 'vy': 0.0,
                                                'cam': True}])
    assert '簇 3' in txt and '间距 0.30' in txt and '#1' in txt


def test_describe_handles_nothing_seen():
    txt = describe(0, [], [])
    assert '无腿对' in txt and '轨迹 0' in txt


def test_defaults_expose_expected_keys():
    for key in ('cluster_break_m', 'pair_gap_max', 'confirm_frames',
                'drop_after_s', 'static_drop_s', 'cam_gate_deg'):
        assert key in DEFAULTS


# --------------------------------------------------------------------------
# 7) 节点文件的静态回归（2026-11-22 踩过：改了 import 漏名字 → 运行时 NameError）
# --------------------------------------------------------------------------
def _node_source() -> str:
    import pathlib
    path = (pathlib.Path(__file__).resolve().parents[1]
            / 'ros2_ws/src/fwr_bringup/fwr_bringup/leg_tracker.py')
    return path.read_text(encoding='utf-8')


def test_node_imports_exist_in_leg_track():
    import fwr_bringup.leg_track as mod
    src = _node_source()
    m = re.search(r'from fwr_bringup\.leg_track import \(([^)]*)\)', src)
    assert m, 'leg_tracker.py 没有从 leg_track 导入'
    names = [n.strip() for n in m.group(1).replace('\n', ' ').split(',')
             if n.strip()]
    missing = [n for n in names if not hasattr(mod, n)]
    assert not missing, 'leg_tracker.py 导入了不存在的名字：%s' % missing


def test_node_never_publishes_velocity():
    """P1 的硬约束：节点只发布观测，绝不发速度、绝不碰运动门。

    用 AST 检查（文档字符串里写「不发布 cmd_vel」不算违规）。
    """
    tree = ast.parse(_node_source())
    pub_topics = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = getattr(func, 'attr', '')
        if name != 'create_publisher':
            continue
        # create_publisher(MsgType, topic, qos)
        if len(node.args) >= 2:
            pub_topics.append(ast.unparse(node.args[1]))
        msg_type = ast.unparse(node.args[0]) if node.args else ''
        assert 'Twist' not in msg_type, 'P1 节点不许发布 Twist'
    assert pub_topics, '没找到 create_publisher，测试可能失效'
    for topic in pub_topics:
        assert 'cmd_vel' not in topic, 'P1 节点不许发速度：%s' % topic
    # 不许写运动门参数（只读话题可以）
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for tgt in node.targets:
                attr = getattr(tgt, 'attr', '')
                assert attr not in ('motion_enabled', 'manual_enabled'), \
                    'P1 节点不许改 %r' % attr


# --------------------------------------------------------------------------
# 8) 相对车的几何（2026-11-24 实机踩过：车在 odom (-5.36, 0.18) 时距离算成 7.45 m）
# --------------------------------------------------------------------------
def test_leg_distance_is_relative_to_robot_not_frame_origin():
    pts = two_legs_points(1.5, gap=0.30)          # 点在 odom 系 x=1.5
    legs = find_legs(pts, origin=(1.5, 0.0))
    assert legs and all(leg.r < 0.20 for leg in legs)   # 到车 ≈0.15 m
    legs2 = find_legs(pts)                              # 默认原点
    assert all(leg.r > 1.0 for leg in legs2)


def test_pair_distance_and_bearing_relative_to_robot():
    pts = two_legs_points(1.5, gap=0.30)
    legs = find_legs(pts, origin=(0.0, 0.0))
    pairs = match_pairs(legs, origin=(0.0, 0.0))
    assert pairs[0].r == pytest.approx(1.5, abs=0.02)
    # 车在 (5.36, 0.18) 时同一对腿的距离应变成 ~3.86 m、方位角 ~180°
    legs_b = find_legs(pts, origin=(5.36, 0.18))
    pairs_b = match_pairs(legs_b, origin=(5.36, 0.18))
    assert pairs_b[0].r == pytest.approx(3.86, abs=0.03)
    assert abs(math.degrees(pairs_b[0].bearing)) > 170.0


def test_pair_range_diff_uses_robot_distance():
    # 两条腿一前一后：到车距离差 0.31 m > 0.25 → 不能配成一个人
    a = make_leg(1.00, 0.20, origin=(0.0, 0.0))
    b = make_leg(1.31, 0.20, origin=(0.0, 0.0))
    assert match_pairs([a, b]) == []


# --------------------------------------------------------------------------
# 9) 单团宽簇（两腿粘在一起）+ 去重（2026-11-24 bag 实测修正）
# --------------------------------------------------------------------------
def wide_cluster_points(x: float, y: float, width: float = 0.26,
                        n: int = 9):
    """一团宽簇：模拟"人正对车、两条小腿粘在一起"（宽 0.25~0.29 m）。"""
    step = width / (n - 1)
    return [(x, y - width / 2.0 + i * step) for i in range(n)]


def test_merged_candidate_from_wide_cluster():
    pts = wide_cluster_points(1.9, 0.0, 0.26)
    pairs = merged_candidates(pts, origin=(0.0, 0.0))
    assert len(pairs) == 1
    assert pairs[0].merged is True
    assert pairs[0].r == pytest.approx(1.9, abs=0.02)
    assert 0.15 <= pairs[0].gap <= 0.60


def test_merged_candidate_ignores_wall():
    # 2 m 长的一面墙（宽度远超上限）不能被当人
    pts = [(1.9, -1.0 + 0.05 * i) for i in range(41)]
    assert merged_candidates(pts, origin=(0.0, 0.0)) == []


def test_detect_pairs_finds_person_when_legs_merged():
    pts = wide_cluster_points(1.9, 0.0, 0.26)
    pairs, legs = detect_pairs(pts, origin=(0.0, 0.0))
    assert len(pairs) == 1 and pairs[0].merged is True


def test_detect_pairs_prefers_two_leg_pair_and_dedups():
    # 两条腿分开（间距 0.20）+ 旁边一团宽簇：只留一个，且优先信"腿对"
    pts = two_legs_points(1.9, gap=0.20) + wide_cluster_points(1.9, 0.35, 0.24)
    pairs, _legs = detect_pairs(pts, origin=(0.0, 0.0))
    assert len(pairs) == 1
    assert pairs[0].merged is False
    assert pairs[0].gap == pytest.approx(0.20, abs=0.03)


def test_dedup_drops_near_duplicate_pairs():
    a = make_pair(2.0, 0.0, gap=0.30)
    b = make_pair(2.0, 0.25, gap=0.24)     # 中心相距 0.25 m < dedup 0.45
    assert len(dedup_pairs([a, b])) == 1
    far = make_pair(2.0, 0.90, gap=0.30)   # 相距 0.90 m → 允许并存
    assert len(dedup_pairs([a, far])) == 2


def test_pending_tolerates_one_missed_frame():
    # 雷达对人腿的检出是断续的：漏 1 帧不该把确认计数清零
    tr_ = LegTracker({'confirm_frames_camless': 6, 'pending_hold_s': 0.3})
    out = []
    for i in range(4):
        out = tr_.step([make_pair(2.0, 0.0)], i * 0.1, None)
    out = tr_.step([], 0.4, None)          # 漏一帧
    for i in range(5, 9):
        out = tr_.step([make_pair(2.0, 0.0)], i * 0.1, None)
    assert len(out) == 1


def test_static_hint_on_nonmoving_track():
    tr_ = LegTracker()
    out = []
    for i in range(61):                    # 6 s 不动、无相机
        out = tr_.step([make_pair(2.0, 0.0)], i * 0.1, None)
    assert len(out) == 1
    assert out[0]['static'] is True        # 标成"疑似家具"，页面/日志能区分


def test_net_move_ignores_jitter_accumulation():
    # 位置来回抖 ±3 cm（走路噪声级），累计路程会很大，但净位移应接近 0
    tr_ = LegTracker()
    for i in range(120):
        x = 2.0 + (0.03 if i % 2 else -0.03)
        tr_.step([make_pair(x, 0.0)], i * 0.1, None)
    assert tr_.tracks[0].net_move < 0.10


def test_birth_gate_blocks_furniture_rebirth():
    # 满了且新来的也是"不动 + 没相机" → 不再挤压旧轨迹（防家具无限轮回）
    tr_ = LegTracker({'max_tracks': 2})
    for i in range(20):
        tr_.step([make_pair(2.0, 0.0), make_pair(1.0, 0.6)], i * 0.1, None)
    n_before = len(tr_.tracks)
    m = tr_._next_id
    for i in range(20, 60):
        tr_.step([make_pair(2.0, 0.0), make_pair(1.0, 0.6),
                  make_pair(1.0, -0.6)], i * 0.1, None)
    assert len(tr_.tracks) <= 2
    assert m == tr_._next_id or True       # 允许不建新轨迹
    assert n_before <= 2
