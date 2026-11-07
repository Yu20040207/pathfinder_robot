"""视点规划核心的单元测试（纯 numpy，不依赖 ROS）。"""
import numpy as np
import pytest

from fwr_bringup import viewpoint_core as vc


def make_map(rows):
    """把字符画变成 (data, width, height)。  '.'=自由  '#'=障碍  '?'=未知"""
    h = len(rows)
    w = len(rows[0])
    data = np.zeros(w * h, dtype=np.int16)
    for y, line in enumerate(rows):
        for x, ch in enumerate(line):
            data[y * w + x] = 0 if ch == '.' else (100 if ch == '#' else -1)
    return data, w, h


def test_visibility_stops_at_wall():
    rows = [
        "........",
        "........",
        "...###..",     # 视点下方有墙
        "........",
    ]
    data, w, h = make_map(rows)
    blocked = vc.blocked_mask(data, w, h)
    unknown = vc.unknown_mask(data, w, h)
    # fill_cells=0：严格射线（不填充），用来验证'射线被墙/未知挡住'
    seen = vc.visible_cells(blocked, unknown, (0, 2), sensor_cells=5, rays=72,
                            fill_cells=0)
    cells = {(int(i) // w, int(i) % w) for i in seen}
    assert (1, 2) in cells, '墙之前的格子应可见'
    # 墙在 y=2 的 x=3..5：正后方(x=4)被挡，两侧缺口(x=0/7)可见
    assert (3, 4) not in cells, '墙正后方不应可见（射线被挡）'
    assert (3, 0) in cells, '墙的缺口方向应当可见'


def test_visibility_stops_at_unknown():
    rows = [
        "........",
        "...??...",
        "........",
    ]
    data, w, h = make_map(rows)
    blocked = vc.blocked_mask(data, w, h)
    unknown = vc.unknown_mask(data, w, h)
    # 默认：未知只是"还没记录"，物理上不挡光 → 能看穿
    seen = vc.visible_cells(blocked, unknown, (2, 3), sensor_cells=5, rays=72,
                            fill_cells=0)
    cells = {(int(i) // w, int(i) % w) for i in seen}
    assert (0, 3) in cells, '未知区不挡光，应能看到后面'
    # 只有显式要求"只统计已记录区域"时才在未知处中断
    seen2 = vc.visible_cells(blocked, unknown, (2, 3), sensor_cells=5, rays=72,
                             fill_cells=0, stop_at_unknown=True)
    cells2 = {(int(i) // w, int(i) % w) for i in seen2}
    assert (0, 3) not in cells2, 'stop_at_unknown=True 时未知之后不算看见'


def test_radius_limited_selection_prefers_near():
    """半径内的小收益点应该胜过半径外的巨大收益点。"""
    w = h = 40
    drivable = np.ones((h, w), dtype=bool)
    dist = np.full((h, w), 0, dtype=np.int32)
    dist[:, 20:] = 200                      # 右半区"很远"
    covered = np.zeros(w * h, dtype=bool)
    near_vis = np.array([i for i in range(0, w * h)], dtype=np.int64)  # 假设看见很多
    far_vis = np.arange(0, w * h, dtype=np.int64)

    cands = [(5, 5), (5, 35)]               # 左=近处(低收益)，右=远处(高收益)
    visibility = [np.arange(20, dtype=np.int64), far_vis]   # 增益 20 vs 1600
    i = vc.pick_next_viewpoint(cands, visibility, covered, dist, radius_cells=50)
    assert i == 0, '半径外的点不应被选中（即使收益高得多）'

    # 半径放大后，远处高收益点按性价比应当胜出：
    #   近: 20/(0+1)=20   远: 1600/(200+1)≈8   → 仍然近的赢
    # 所以把远处拉近（dist 改小）才能体现"远处高收益胜出"
    dist2 = dist.copy(); dist2[:, 35] = 5
    i2 = vc.pick_next_viewpoint(cands, visibility, covered, dist2, radius_cells=500)
    assert i2 == 1, '路径接近且收益高时应当选远处那个' 


def test_pick_skips_visited_and_low_gain():
    w = h = 20
    drivable = np.ones((h, w), dtype=bool)
    dist = np.zeros((h, w), dtype=np.int32)
    covered = np.zeros(w * h, dtype=bool)
    covered[0:50] = True
    vis_small = np.array([0, 1, 2, 3], dtype=np.int64)          # 增益 0（都已覆盖）
    visibility = [vis_small, np.arange(60, 80, dtype=np.int64)]  # 增益 20
    cands = [(1, 1), (1, 2)]
    assert vc.pick_next_viewpoint(cands, visibility, covered, dist, 50) == 1
    assert vc.pick_next_viewpoint(cands, visibility, covered, dist, 50,
                                  visited=[(1, 2)]) is None


def test_frontier_and_expansion_target_inside_known():
    rows = [
        "#########",
        "#.......#",
        "#.......?",
        "#.......?",
        "#########",
    ]
    data, w, h = make_map(rows)
    free = vc.free_mask(data, w, h)
    unknown = vc.unknown_mask(data, w, h)
    drivable = vc.erode(free, 1)
    fronts = vc.frontier_cells(free, unknown)
    assert fronts, '应该找到前沿'
    dist = vc.bfs_distance(drivable, (2, 3))
    tgt = vc.expansion_target(fronts, drivable, dist, inset_cells=1)
    assert tgt is not None
    y, x = tgt
    assert drivable[y, x], '扩展目标必须在可行驶区'
    assert free[y, x], '扩展目标必须在已知自由区（不能打进未知）'


def test_bfs_distance_respects_walls():
    rows = [
        "...",
        ".#.",
        "...",
    ]
    data, w, h = make_map(rows)
    passable = vc.free_mask(data, w, h)
    d = vc.bfs_distance(passable, (0, 0))
    assert d[0, 0] == 0
    assert d[1, 2] > 0, '绕过障碍应当可达但更远'
    rows2 = ["...", "###", "..."]
    data2, w2, h2 = make_map(rows2)
    d2 = vc.bfs_distance(vc.free_mask(data2, w2, h2), (0, 0))
    assert d2[2, 1] == -1, '被完全隔断的区域不可达'


def test_coverage_ratio():
    free = np.array([True, True, True, False])
    covered = np.array([True, False, False, True])
    assert vc.coverage_ratio(covered, free) == pytest.approx(1 / 3)


def test_fill_leaks_only_a_few_cells_past_wall():
    """扇面填充会把已知区扩一点，但渗透量必须 <= fill_cells（否则会误判"墙后看过"）。"""
    rows = [
        "........",
        "........",
        "...###..",
        "........",
        "........",
    ]
    data, w, h = make_map(rows)
    blocked = vc.blocked_mask(data, w, h)
    unknown = vc.unknown_mask(data, w, h)
    fill = 2
    seen = vc.visible_cells(blocked, unknown, (0, 2), sensor_cells=5, rays=72,
                            fill_cells=fill)
    cells = {(int(i) // w, int(i) % w) for i in seen}
    behind = [y for y, x in cells if 3 <= x <= 5 and y > 2]
    assert not behind or max(behind) - 2 <= fill, (
        '填充渗透了 %d 格，超过允许的 %d' % (max(behind) - 2 if behind else 0, fill))


def test_heading_penalty_prefers_forward_candidate():
    """航向连续性：同等收益时不应为身后的点立刻掉头。"""
    w = h = 20
    dist = np.zeros((h, w), dtype=np.int32)
    covered = np.zeros(w * h, dtype=bool)
    vis = np.arange(100, dtype=np.int64)
    cands = [(5, 12), (5, 2)]          # east / west of robot at (5,7)
    visibility = [vis, vis.copy()]
    i_forward = vc.pick_next_viewpoint(
        cands, visibility, covered, dist, radius_cells=50,
        heading=0.0, origin=(5, 7), turn_weight=0.5)
    i_plain = vc.pick_next_viewpoint(
        cands, visibility, covered, dist, radius_cells=50)
    assert i_forward == 0, '朝向 0° 时应优先同一方向的候选'
    assert i_plain is not None


def test_frontier_clusters_group_adjacent_cells():
    """相邻前沿格必须合成一簇，否则回退会逐格换目标。"""
    w = h = 12
    free = np.zeros((h, w), dtype=bool)
    unknown = np.zeros((h, w), dtype=bool)
    for y, x in [(3, 3), (3, 4), (4, 4), (8, 8)]:
        free[y, x] = True
        unknown[y, x + 1] = True
    fronts = vc.frontier_cells(free, unknown)
    clusters = vc.frontier_clusters(fronts, (h, w))
    sizes = sorted(len(c) for c in clusters)
    assert sizes == [1, 3], f'unexpected clusters: {clusters}'
