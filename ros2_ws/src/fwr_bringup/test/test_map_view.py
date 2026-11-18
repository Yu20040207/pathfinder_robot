"""map_view 降采样：块内优先级（占用 > 未知 > 空闲）与尺寸计算。"""
import pytest

from fwr_bringup.map_view_core import UNKNOWN, downsample


def test_factor_one_is_identity():
    data = [0, 100, -1, 0]
    out, w, h = downsample(data, 2, 2, 1)
    assert (w, h) == (2, 2)
    assert out == data


def test_occupied_wins_in_block():
    """块内只要有一个占用格，整块就是占用 —— 障碍不能被可视化抹掉。"""
    data = [0, 0, 0, 100]
    out, w, h = downsample(data, 2, 2, 2)
    assert (w, h) == (1, 1)
    assert out == [100]


def test_unknown_beats_free():
    """未知优先于空闲，避免把未扫到的地方画成空地。"""
    data = [0, -1, 0, 0]
    out, _, _ = downsample(data, 2, 2, 2)
    assert out == [-1]


def test_shape_with_odd_size():
    """尺寸非整数倍时向上取整，不丢边缘。"""
    out, w, h = downsample([0] * (5 * 3), 5, 3, 2)
    assert (w, h) == (3, 2)
    assert len(out) == 6


def test_all_free_stays_free():
    out, _, _ = downsample([0] * 16, 4, 4, 2)
    assert out == [0] * 4


def test_bad_length_raises():
    with pytest.raises(ValueError):
        downsample([0, 0], 3, 3, 2)
