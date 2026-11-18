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

def test_crop_centers_on_robot():
    """裁剪窗口以机器人格坐标为中心，尺寸 = 2*half+1。"""
    from fwr_bringup.map_view_core import crop_around
    data = list(range(10 * 10))
    out, w, h, dx, dy = crop_around(data, 10, 10, 5, 5, 2)
    assert (w, h) == (5, 5)
    assert (dx, dy) == (3, 3)
    assert out[0] == 3 * 10 + 3          # 左上角是原图 (3,3)


def test_crop_clamps_at_edges():
    """靠近边缘时窗口整体夹进边界内：尺寸保持 2*half+1，不会越界也不会缩小。"""
    from fwr_bringup.map_view_core import crop_around
    data = list(range(10 * 10))
    out, w, h, dx, dy = crop_around(data, 10, 10, 0, 0, 2)
    assert (dx, dy) == (0, 0)            # 夹到左上角
    assert (w, h) == (5, 5)              # 尺寸仍是 2*half+1
    assert len(out) == 25
    # 右下角同理
    out2, w2, h2, dx2, dy2 = crop_around(data, 10, 10, 9, 9, 2)
    assert (dx2, dy2) == (5, 5)
    assert (w2, h2) == (5, 5)


def test_crop_keeps_resolution_cells():
    """裁剪不改变分辨率语义：所有格子都来自原图，值不变。"""
    from fwr_bringup.map_view_core import crop_around
    data = [100 if i % 7 == 0 else 0 for i in range(20 * 20)]
    out, w, h, _, _ = crop_around(data, 20, 20, 10, 10, 4)
    assert all(v in (0, 100) for v in out)
    assert w == h == 9
