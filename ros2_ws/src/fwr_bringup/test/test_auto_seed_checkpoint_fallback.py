"""播种的 checkpoint 回退与"两条路径必须同一套换算"。

2026-11-18 现场：checkpoint 细化出残差 0.039 m 的位姿，但车体位姿落在
地图障碍格里（跨墙退化匹配）。旧逻辑在 checkpoint 分数够好时直接返回，
后续检查拒绝却不回退全图搜索 → 永远播不了种 → 无 map→odom →
auto_activate_nav 不激活 Nav2 → 整栈 unconfigured。
"""
from pathlib import Path

SRC = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'auto_initial_pose.py').read_text()


def test_checkpoint_fallbacks_to_global_search():
    assert 'def _match_or_global' in SRC
    assert 'force_global=True' in SRC, '必须在 checkpoint 位姿不合法时回退全图'
    assert '_used_checkpoint' in SRC, '需要知道这次用的是不是 checkpoint'


def test_both_seeding_paths_use_same_conversion():
    """服务路径和定时自动路径都必须走 _to_base_pose（TF 换算）。

    曾经只改了一处，定时路径仍用写死的 `yaw - pi / -0.21 m`，
    换成 /scan_merged 后会把车位姿多转 180°、多偏 21 cm。
    """
    assert SRC.count('_match_or_global()') == 2, '两条播种路径都要用同一个匹配函数'
    # 定时路径里不允许再出现写死公式
    i = SRC.index('def _attempt_seed') if 'def _attempt_seed' in SRC else 0
    body = SRC[i:] if i else SRC
    assert 'base_pose_from_laser(\n            x, y, yaw' not in SRC


def test_sanity_check_still_rejects_wall_pose():
    """护栏不能被回退逻辑放松：不合法位姿仍然拒绝播种。"""
    assert 'not-free-or-unknown' in SRC
    assert '拒绝播种' in SRC


def test_clearance_mask_blocks_wall_positions():
    """掩码必须把"只有一格空闲的墙缝"排掉，只留车真能站的位置。"""
    import numpy as np
    from fwr_bringup.auto_seed_core import clearance_mask

    # 10x10 全空闲，只在 (5,5) 放一格障碍
    grid = np.zeros((10, 10), dtype=np.int16)
    grid[5, 5] = 100
    mask = clearance_mask(grid, 0.05, clearance_m=0.10)
    assert not mask[5, 5], '障碍格本身不能停'
    assert not mask[4, 4], '障碍 0.10 m 内的格子不能停'
    assert mask[0, 0] is np.bool_(False) or not mask[0, 0], '地图边缘不能停'
    assert mask[2, 2], '远离障碍的格子应可停'


def test_search_only_returns_standable_pose():
    """接入掩码后，全局搜索给出的位姿必须落在合法格上。"""
    import numpy as np
    from fwr_bringup.auto_seed_core import clearance_mask, find_best_pose

    # 一间 5x5 房间（墙一圈），中间空
    size, res = 60, 0.05
    grid = np.zeros((size, size), dtype=np.int16)
    grid[0:3, :] = 100
    grid[-3:, :] = 100
    grid[:, 0:3] = 100
    grid[:, -3:] = 100
    occ = grid >= 65
    mask = clearance_mask(grid, res, 0.10)
    # 模拟"机器人站在小房间中间、扫描点落在四周墙上"：半径 0.8 m 的一圈点
    pts = np.array([[0.8 * np.cos(a), 0.8 * np.sin(a)] for a in np.linspace(0, 6.28, 40)])
    bounds = (0.0, size * res, 0.0, size * res)
    score, x, y, yaw = find_best_pose(occ, res, 0.0, 0.0, bounds, pts,
                                      coarse_step=0.2, valid_mask=mask)
    ix, iy = int(x / res), int(y / res)
    assert 0 <= ix < size and 0 <= iy < size
    assert score < 9.0, f'没找到任何候选（哨兵值 {score}）'
    assert mask[iy, ix], f'搜索给出了停不进去的位姿 ({x:.2f},{y:.2f})'
