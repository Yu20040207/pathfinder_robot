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
