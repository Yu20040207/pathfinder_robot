"""车尾近障判定的纯逻辑（不依赖 ROS，便于单测）。

2026-11-18：车尾从"四路超声"升级为"后雷达"。雷达在水平面测距，
比超声（±1~3 cm 噪声、受地面/斜面反射影响）更可靠，而且能看到整条车尾弧线。
"""

from __future__ import annotations

import math
from typing import Sequence


def sector_min_range(ranges: Sequence[float],
                     angle_min: float,
                     angle_increment: float,
                     center_deg: float = 180.0,
                     half_deg: float = 35.0,
                     range_min: float = 0.05,
                     range_max: float = 12.0) -> float:
    """取"正后方 ±half_deg"扇区内的最近有效距离；没有有效点时返回 inf。

    Parameters
    ----------
    ranges
        LaserScan.ranges（遮罩区是 +inf，会被跳过）。
    center_deg
        扇区中心（雷达自身角度）。默认 180° = 正后方（后雷达 yaw=0 时与 base 一致）。
    half_deg
        半张角。默认 ±35°（70° 视野），足够覆盖车尾宽度又不会扫到侧墙。

    Returns
    -------
    float
        最近有效距离（米）；扇区内没有有效回波时返回 ``math.inf``。
    """
    if not ranges or angle_increment == 0.0:
        return math.inf
    lo = center_deg - half_deg
    hi = center_deg + half_deg
    best = math.inf
    for i, r in enumerate(ranges):
        if not (math.isfinite(r) and range_min <= r <= range_max):
            continue
        a = math.degrees(angle_min + i * angle_increment)
        # 扇区可能跨越 ±180 边界（默认中心 180°±35 → [145°, 215°]）。
        # 所以不规整 a，而是把 a 平移 ±360 后再比，避免 181° 被写成 -179° 漏判。
        if (lo <= a <= hi) or (lo <= a + 360.0 <= hi) or (lo <= a - 360.0 <= hi):
            if r < best:
                best = r
    return best
