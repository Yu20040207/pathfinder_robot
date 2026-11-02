"""掉头检测：路径起点切线 vs 前方 3 m 处切线的夹角。"""
import math

from fwr_bringup.turn_radius_boost import path_heading_change


class P:
    def __init__(self, x, y):
        self.pose = type('o', (), {'position': type('o', (), {'x': x, 'y': y})})()


def _path(pts):
    return [P(x, y) for x, y in pts]


def _straight():
    return _path([(i * 0.1, 0.0) for i in range(50)])


def _uturn():
    pts = [(i * 0.1, 0.0) for i in range(31)]
    pts += [(3.0, 0.1), (2.9, 0.2), (2.0, 0.2), (1.0, 0.2), (0.0, 0.2)]
    return _path(pts)


def _gentle():
    return _path([(4 * math.sin(t), 4 - 4 * math.cos(t))
                  for t in [i * 0.01 for i in range(200)]])


def test_straight_path_is_not_a_uturn():
    assert path_heading_change(_straight(), 3.0) < math.radians(20)


def test_gentle_curve_is_not_a_uturn():
    assert path_heading_change(_gentle(), 3.0) < math.radians(70)


def test_uturn_is_detected():
    assert path_heading_change(_uturn(), 3.0) >= math.radians(70)


def test_short_paths_return_none():
    assert path_heading_change([], 3.0) is None
    assert path_heading_change(_path([(0.0, 0.0)]), 3.0) is None


def test_find_turn_ranges_on_uturn_path():
    from fwr_bringup.turn_radius_boost import find_turn_ranges
    r = find_turn_ranges(_uturn(), 0.6, math.radians(70))
    assert len(r) >= 1, 'U 型掉头应至少检出 1 段'
    a, b = r[0]
    # 转点在 index 30；函数会向两侧各扩 2 个点，所以起点落在 20~30
    assert 20 <= a <= 30, f'掉头段起点应在转点附近，实际 {a}'


def test_find_turn_ranges_on_straight_path():
    from fwr_bringup.turn_radius_boost import find_turn_ranges
    assert find_turn_ranges(_straight(), 0.6, math.radians(70)) == []


def test_find_turn_ranges_on_gentle_curve():
    from fwr_bringup.turn_radius_boost import find_turn_ranges
    assert find_turn_ranges(_gentle(), 0.6, math.radians(70)) == []
