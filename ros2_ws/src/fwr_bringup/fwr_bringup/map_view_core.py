"""给 RViz 看的降采样地图（可视化用，不影响 slam 与代价地图）。

为什么要单独一路
----------------
slam_toolbox 用 Reliable + Transient Local 发布 /map，一帧 149 KB。
RViz 在 WiFi 上做可靠消费者时，一旦丢包就会触发 FastDDS 反复重传，
实测把机器人发送量从 310 KB/s 放大到 7000 KB/s，遥控指令被挤在后面 → 操作卡顿。

本模块把地图按整数倍降采样（取块内最大值：任一格占用则整块占用，障碍不会被抹掉），
再由 map_view 节点以 Best Effort 低频发布，RViz 只看这一路。
"""

from __future__ import annotations

from typing import Sequence

# Level 取值见 nav_msgs/OccupancyGrid：-1 未知、0 空闲、100 占用
UNKNOWN = -1
OCCUPIED = 100


def downsample(data: Sequence[int], width: int, height: int, factor: int):
    """按 factor 降采样，块内取"最保守"的值。

    优先级：占用 > 未知 > 空闲。
    —— 占用优先保证障碍不会被可视化抹掉；未知优先于空闲，避免把没扫到的地方画成空地。

    Returns
    -------
    (new_data, new_width, new_height)
    """
    if factor <= 1:
        return list(data), width, height
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError('data 长度与 width*height 不一致')
    nw, nh = (width + factor - 1) // factor, (height + factor - 1) // factor
    out = [UNKNOWN] * (nw * nh)
    for ny in range(nh):
        for nx in range(nw):
            best = UNKNOWN
            rank = 0
            for dy in range(factor):
                y = ny * factor + dy
                if y >= height:
                    break
                row = y * width
                for dx in range(factor):
                    x = nx * factor + dx
                    if x >= width:
                        break
                    v = data[row + x]
                    r = 3 if v >= 50 else (2 if v < 0 else 1)
                    if r > rank:
                        rank, best = r, v
                        if rank == 3:      # 已经是最高优先级，提前结束
                            break
            out[ny * nw + nx] = best
    return out, nw, nh
