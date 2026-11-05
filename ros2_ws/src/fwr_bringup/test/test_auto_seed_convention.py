"""坐标约定回归：OccupancyGrid 的行 0 就是 origin_y，绝不能翻转。

2026-11-05：auto_seed_core.score_pose 曾用 `height-1-(y-origin)/res`（图片行序），
把地图上下镜像 → 自动播种把车放到镜像位置，实测每次启动落在相距 7~13 m 的点上；
而正确约定下的全局最佳匹配与用户手动摆的位置只差 5 cm。
"""
import math

import numpy as np

from fwr_bringup.auto_seed_core import distance_field, score_pose


def test_score_pose_uses_occupancygrid_row_order():
    H, W, res = 40, 40, 0.1
    ox, oy = 0.0, 0.0
    occ = np.zeros((H, W), dtype=bool)
    occ[20, :] = True                       # 墙在 y = 2.0 m（第 20 行）
    dist = distance_field(occ, res)
    # 车在 (2.0, 1.0)，朝 +y（yaw=90°）：车体坐标 x=前方 → 世界 +y。
    # 前方约 1 m 处就是 y=2.0 的墙，所以扫描点应写成 (前进距离, 0)。
    pts = np.array([[0.90, 0.0], [1.00, 0.0], [0.95, -0.2], [0.95, 0.2]])
    good = score_pose(dist, res, ox, oy, pts, 2.0, 1.0, math.pi / 2)
    mirrored = score_pose(dist, res, ox, oy, pts, 2.0, 3.0, math.pi / 2)
    # 扫描点距离墙 0-0.1 m，平均残差约 0.1 m
    assert good < 0.15, f'正确位姿应该贴着墙，实际 {good:.3f}'
    assert mirrored > good + 0.5, f'镜像位姿必须明显更差，实际 {mirrored:.3f}'


def test_find_best_pose_is_fast_enough_to_run_live():
    """全局搜索必须能在实机上跑完（原来逐位姿循环要几小时 → 一键重新定位永不返回）。

    这里造一张 20x20 m 的地图，要求 5 秒内出结果，并且结果接近真值。
    """
    import time
    import numpy as np
    from fwr_bringup.auto_seed_core import find_best_pose

    res = 0.05
    H = W = int(20.0 / res)
    occ = np.zeros((H, W), dtype=bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    occ[int(8.0 / res):int(8.2 / res), int(4.0 / res):int(12.0 / res)] = True
    ox, oy = -5.0, -5.0
    # 加几个不对称特征，否则长方形房间镜像/平移都能匹配，测不出位置
    for (bx, by, bw, bh) in [(1.0, 1.0, 0.6, 0.6), (6.5, 2.0, 0.4, 0.8),
                             (3.0, 6.5, 1.0, 0.4), (9.0, 9.0, 0.5, 0.5)]:
        occ[int((by - oy) / res):int((by + bh - oy) / res),
            int((bx - ox) / res):int((bx + bw - ox) / res)] = True
    # 真值位姿与它的扫描（沿 72 个方向投射到最近障碍）
    truth = (3.0, 3.0, 0.4)
    pts = []
    for ang in np.radians(np.arange(-180, 180, 5)):
        for r in np.arange(0.2, 6.0, 0.05):
            x = truth[0] + r * math.cos(truth[2] + ang)
            y = truth[1] + r * math.sin(truth[2] + ang)
            ix = int((x - ox) / res)
            iy = int((y - oy) / res)
            if not (0 <= ix < W and 0 <= iy < H) or occ[iy, ix]:
                pts.append((r * math.cos(ang), r * math.sin(ang)))
                break
    pts = np.array(pts)
    bounds = (ox, ox + W * res, oy, oy + H * res)
    t0 = time.time()
    score, x, y, yaw = find_best_pose(occ, res, ox, oy, bounds, pts)
    dt = time.time() - t0
    assert dt < 5.0, f'全局搜索太慢：{dt:.1f} s（原来要几小时）'
    assert score < 0.25, f'搜索质量太差：{score:.3f}'
    assert math.hypot(x - truth[0], y - truth[1]) < 0.6, (x, y, truth)


def test_map_edit_fix_preserves_unknown_and_applies_edits():
    """Map Edit 面板保存会丢未知区；修正脚本必须只应用真实编辑。"""
    import sys
    import numpy as np

    from pathlib import Path
    here = Path(__file__).resolve()
    repo_root = here.parents[4]              # .../Four-wheel-robot
    sys.path.insert(0, str(repo_root / 'tools'))
    try:
        from fix_map_unknown import apply_edits
    except ImportError:
        import pytest
        pytest.skip('tools 目录不可导入')

    base = np.full((6, 6), 205, dtype=np.uint8)   # 全未知
    base[2:4, 2:4] = 254                          # 一小块自由
    base[2, 3] = 0                                # 一堵墙
    # 面板保存的版本：未知全部变 254，墙上画了新墙，原墙被擦掉
    edited = np.full((6, 6), 254, dtype=np.uint8)
    edited[2, 3] = 254                            # 擦掉原墙
    edited[4, 4] = 0                              # 新画一堵墙
    out, drew, erased = apply_edits(base, edited)
    assert out[2, 3] == 254, '被擦的墙要变白'
    assert out[4, 4] == 0, '新画的墙要变黑'
    assert (out == 205).sum() == base.shape[0] * base.shape[1] - 5 - 1 + 1, (
        '除编辑点外未知区必须原样保留')
    assert drew == 1 and erased == 1
