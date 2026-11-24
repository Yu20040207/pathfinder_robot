"""`fwr_bringup.leg_track` 单测（雷达腿部检测 + 多目标跟踪，纯逻辑）。

运行：``PYTHONPATH=ros2_ws/src/fwr_bringup .venv/bin/python -m pytest tests/test_leg_track.py -q``
"""

from __future__ import annotations

import ast
import math
import re

import pytest

from fwr_bringup.leg_track import (DEFAULTS, Kalman2D, Leg, LegTracker, Pair,
                                   cluster_points, cluster_width, describe,
                                   find_legs, match_pairs, scan_to_points,
                                   to_world, world_to_robot)


# --------------------------------------------------------------------------
# 构造工具
# --------------------------------------------------------------------------
def make_leg(x: float, y: float, width: float = 0.12, n: int = 8) -> Leg:
    return Leg(x, y, width, n, math.hypot(x, y))


def make_pair(x: float, y: float, gap: float = 0.30) -> Pair:
    a = make_leg(x, y + gap / 2.0)
    b = make_leg(x, y - gap / 2.0)
    return Pair(x, y, gap, a.width, b.width, math.hypot(x, y), (a, b))


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
    # 丢 1.6 s：删除
    out = tr_.step([], 1.9, None)
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


def test_tracker_drops_static_object_without_camera():
    # 椅子腿：位置不变、相机也没看到 → 10 s 后必须丢掉
    tr_ = LegTracker()
    out = []
    for i in range(61):                  # 6 s
        out = tr_.step([make_pair(2.0, 0.0)], i * 0.1, None)
    assert len(out) == 1
    for i in range(61, 121):             # 再 6 s（累计 12 s）
        out = tr_.step([make_pair(2.0, 0.0)], i * 0.1, None)
    # 丢掉之后位置进黑名单，不会每 10 s 轮回一条新轨迹
    assert out == []


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
