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


def choose_sources(fresh, published_full, require_both=True, fallback_single=True,
                   waited_s=0.0, wait_first_frame_s=20.0):
    """决定这一帧用哪几路扫描，以及当前处于什么模式。

    2026-11-18 现场事故：后雷达 USB 掉线 → 合并节点按 ``require_both`` 停止发布
    → AMCL 收不到扫描就不再更新、不再发 map→odom → 代价地图拿不到车位置
    → 规划器报 "Starting point in lethal space" → 行为树进 Spin 恢复
    → 车在原地一直左转直到目标失败。

    ``require_both`` 的**本意只适用于启动阶段**（第一帧必须是完整 360°，否则
    slam_toolbox 可能只吃第一帧，地图永久缺半圈）。一旦已经成功发出过完整帧，
    运行中掉一路应该**降级继续**（单雷达本来就能导航），而不是把定位一起停掉。

    Returns
    -------
    (sources, mode)
        ``sources`` 为空表示这一帧不发布；``mode`` ∈
        'full' / 'degraded' / 'wait-first-frame' / 'wait-both' / 'none'
    """
    fresh = list(fresh)
    if len(fresh) >= 2:
        return fresh[:2], 'full'
    if not fresh:
        return [], 'none'
    if not published_full and require_both:
        # 等完整首帧，但**不能无限等**：万一开机时就少一路（线没插好），
        # 原来会永远不发布 → 定位冻结 → 整车瘫痪。超时后接受单路降级。
        if waited_s < wait_first_frame_s:
            return [], 'wait-first-frame'
        return (fresh[:1], 'degraded') if fallback_single else ([], 'wait-both')
    if fallback_single:
        return fresh[:1], 'degraded'
    return [], 'wait-both'
