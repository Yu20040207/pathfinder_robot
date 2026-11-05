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
