"""把两路已遮罩的 LaserScan 合并成一路 360° 扫描（给 slam_toolbox 用）。

为什么需要这个节点
------------------
- Cartographer 原生支持多路雷达（``num_laser_scans``），但**主控一直在用 slam_toolbox**，
  而 slam_toolbox 2.6.10 只订阅**一个** ``scan_topic``。
- 两路各自只保留 220°（后方 ±70° 被遮罩成 +inf），并集正好 360°。

和 ira_laser_tools 的区别
------------------------
ira 那类合并器对每路只做**一次** TF 变换，再用同一个 stamp 发布；本节点用 tf2 一次
取好两路的静态外参后按点变换，并且发布时用**较新那一路的时间戳**。
车身 0.2 m/s、两路同频 10 Hz 时，相位差最坏 100 ms → 2 cm，可接受。

合并规则
--------
每个 0.5° 角度格（720 格）取**最近**的有效回波；没有回波的格子填 ``+inf``
（ROS 里"无回波"的标准表示，slam_toolbox / Cartographer 都会跳过的值）。
**绝不填 0** —— 0 会被当成"这里有 0 米障碍"。
"""

from __future__ import annotations

import math
from typing import Iterable, Sequence

# 角度格数：720 格 = 0.5°/格，与 RPLIDAR C1 的原始分辨率一致
BINS = 720


def _norm_angle(a: float) -> float:
    """把角度规整到 [-pi, pi)。"""
    return math.atan2(math.sin(a), math.cos(a))


def merge_into_bins(scans: Sequence[tuple[Sequence[float], float, float, float, float]],
                    bins: int = BINS,
                    range_min: float = 0.05,
                    range_max: float = 12.0) -> list[float]:
    """把多路扫描按角度合并成一圈。

    Parameters
    ----------
    scans
        每项为 ``(ranges, angle_min, angle_increment, yaw, tx, ty)``… 实际用 6 元组：
        ``(ranges, angle_min, angle_increment, yaw, tx, ty)``。
        ``(yaw, tx, ty)`` 是该路在**目标坐标系**下的平面外参（由调用方用 tf2 取好）。
    bins
        角度格数（默认 720）。
    range_min, range_max
        有效量程，超出或非有限的值忽略。

    Returns
    -------
    list[float]
        长度 ``bins`` 的列表，索引 0 对应 ``-pi``，顺时针增加；
        空格填 ``math.inf``。
    """
    step = 2.0 * math.pi / bins
    out = [math.inf] * bins
    for item in scans:
        ranges, angle_min, angle_increment, yaw, tx, ty = item
        if angle_increment == 0:
            continue
        cy, sy = math.cos(yaw), math.sin(yaw)
        for index, r in enumerate(ranges):
            if not math.isfinite(r) or not (range_min <= r <= range_max):
                continue
            local = angle_min + index * angle_increment
            x = r * math.cos(local)
            y = r * math.sin(local)
            # 该路局部坐标 → 目标坐标系（绕 Z 转 yaw 再平移）
            gx = cy * x - sy * y + tx
            gy = sy * x + cy * y + ty
            ang = _norm_angle(math.atan2(gy, gx))
            dist = math.hypot(gx, gy)
            if not (range_min <= dist <= range_max):
                continue
            slot = int(round((ang + math.pi) / step)) % bins
            if dist < out[slot]:          # 同一格取最近，保守
                out[slot] = dist
    return out


def filled_ratio(merged: Iterable[float]) -> float:
    """有效格占比，用于健康检查/单测。"""
    values = list(merged)
    if not values:
        return 0.0
    return sum(1 for v in values if math.isfinite(v)) / len(values)
