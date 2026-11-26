#!/usr/bin/env python3
"""握手可达性体检：这只 25 cm 臂到底能碰到哪些位置。

用法::

    python3 tools/arm_reach_check.py
    python3 tools/arm_reach_check.py --shoulder 1.05 --distances 0.10 0.20 0.30

不需要 ROS、不需要硬件。拿到实物后把 ``--l-upper/--l-fore/--l-grip/--shoulder``
换成实测值，结论会跟着变。

输出的三张表分别回答三个问题：

1. **高度带**：夹爪水平伸出去时，能覆盖哪一段离地高度
2. **距离 × 高度**：哪些 (车距, 手高) 组合能握到，够不着的原因是什么
3. **肩高扫描**：上层折叠机构要展开到多高才够得到站着的人
"""

from __future__ import annotations

import argparse
import sys

sys.path.insert(0, "ros2_ws/src/fwr_bringup")

from fwr_bringup.arm_kinematics import (  # noqa: E402
    ANGLE_LIMIT, BEHIND, BELOW_REACH, OK, TOO_CLOSE, TOO_FAR, ABOVE_REACH,
    ArmGeometry, check_point, horizontal_reach_m, solve_hand)

# 人抬手高度的现实区间（离地，米）
HAND_LOW = 0.95
HAND_HIGH = 1.25

REASON_CN = {
    OK: "✓ 能握",
    TOO_FAR: "远",
    TOO_CLOSE: "近",
    ANGLE_LIMIT: "限位",
    ABOVE_REACH: "太高",
    BELOW_REACH: "太低",
    BEHIND: "背后",
}


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="握手可达性体检")
    p.add_argument("--shoulder", type=float, default=1.03,
                   help="肩关节离地高度（米），默认 1.03")
    p.add_argument("--l-upper", type=float, default=0.090, help="上臂段长（米）")
    p.add_argument("--l-fore", type=float, default=0.090, help="前臂段长（米）")
    p.add_argument("--l-grip", type=float, default=0.040, help="腕到夹爪中心（米）")
    p.add_argument("--distances", type=float, nargs="*",
                   default=[0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35],
                   help="水平距离采样点（米，从臂基座量起）")
    p.add_argument("--heights", type=float, nargs="*",
                   default=[0.90, 0.95, 1.00, 1.05, 1.10, 1.15, 1.20, 1.25],
                   help="离地高度采样点（米）")
    return p.parse_args(argv)


def geometry_of(args) -> ArmGeometry:
    return ArmGeometry(shoulder_height_m=args.shoulder,
                       l_upper=args.l_upper,
                       l_fore=args.l_fore,
                       l_grip=args.l_grip)


def print_summary(g: ArmGeometry) -> None:
    print(f"臂基座安装面离地 : {g.mount_height_m * 100:6.1f} cm")
    print(f"肩关节离地       : {g.shoulder_height_m * 100:6.1f} cm")
    print(f"可达半径         : {g.reach_m * 100:6.1f} cm "
          f"(上臂 {g.l_upper*100:.0f} + 前臂 {g.l_fore*100:.0f} + 夹爪 {g.l_grip*100:.0f})")
    print(f"举手最高能碰到   : {g.max_hand_height_m * 100:6.1f} cm")


def print_band(g: ArmGeometry) -> None:
    print("\n[1] 夹爪水平伸出时的可达高度带")
    lo = g.shoulder_height_m - (g.l_upper + g.l_fore)
    hi = g.shoulder_height_m + (g.l_upper + g.l_fore)
    print(f"    肩同高时能到 {lo*100:.0f} ~ {hi*100:.0f} cm（夹爪横伸 4 cm 不计入高度）")
    print(f"    人抬手的现实区间 {HAND_LOW*100:.0f} ~ {HAND_HIGH*100:.0f} cm")
    overlap_lo = max(lo, HAND_LOW)
    overlap_hi = min(hi, HAND_HIGH)
    if overlap_hi > overlap_lo:
        print(f"    → 重叠 {overlap_lo*100:.0f} ~ {overlap_hi*100:.0f} cm  "
              f"({(overlap_hi-overlap_lo)/(HAND_HIGH-HAND_LOW)*100:.0f}% 的抬手区间)")
    else:
        print("    → ⚠️ 完全不重叠，站着的人握不到")


def print_grid(g: ArmGeometry, distances, heights) -> None:
    print("\n[2] 距离 × 高度 可达矩阵（水平距离从臂基座量起）")
    header = "  高度↓ / 距离→ " + " ".join(f"{d*100:5.0f}" for d in distances)
    print(header)
    for h in sorted(heights, reverse=True):
        row = [f"  {h*100:5.0f} cm  "]
        for d in distances:
            reason = check_point((d, 0.0, h), 0.0, g)
            row.append(f"{REASON_CN.get(reason, reason):>5}")
        print(" ".join(row))
    print("  (单位 cm；'远'=车再靠近，'近'=人退后，'限位'=换肘向/改腕角)")


def print_stop_distance(g: ArmGeometry, heights) -> None:
    print("\n[3] 车必须停在多近（按臂基座到手的水平距离算）")
    for h in heights:
        reach = horizontal_reach_m(h, g)
        if reach <= 0:
            print(f"    手高 {h*100:5.0f} cm : 够不到任何水平距离")
            continue
        print(f"    手高 {h*100:5.0f} cm : 最远 {reach*100:5.1f} cm "
              f"→ 车要停到 {reach*100:.0f} cm 以内")


def print_shoulder_sweep(args, heights) -> None:
    print("\n[4] 肩高扫描：上层折叠机构要展开到多高才够到站着的人")
    print("    肩高    最高可碰    1.20 m 处水平够程")
    for h in [0.90, 0.95, 1.00, 1.03, 1.05, 1.10, 1.15]:
        g = geometry_of(argparse.Namespace(**{**vars(args), "shoulder": h}))
        reach120 = horizontal_reach_m(1.20, g)
        mark = "  ← 默认" if abs(h - args.shoulder) < 1e-6 else ""
        ok = "✓" if g.max_hand_height_m >= 1.20 else "✗"
        print(f"    {h*100:5.0f} cm {ok}  {g.max_hand_height_m*100:6.1f} cm "
              f"   {reach120*100:5.1f} cm{mark}")


def print_verdict(g: ArmGeometry) -> None:
    print("\n[结论]")
    if g.max_hand_height_m < HAND_HIGH:
        print(f"    ⚠️ 最高只到 {g.max_hand_height_m*100:.0f} cm，"
              f"够不到站着手抬到 {HAND_HIGH*100:.0f} cm —— 折叠机构要再高 "
              f"{(HAND_HIGH-g.max_hand_height_m)*100:.1f} cm")
    else:
        print(f"    ✓ 高度覆盖：最高 {g.max_hand_height_m*100:.0f} cm ≥ "
              f"人抬手 {HAND_HIGH*100:.0f} cm")

    reach120 = horizontal_reach_m(1.20, g)
    if reach120 <= 0:
        print(f"    ⚠️ 1.20 m 高度上水平够程为 0，只能握更低的点")
    else:
        print(f"    ✓ 1.20 m 高度上能伸 {reach120*100:.0f} cm，"
              f"车要停到该距离内（前保险 0.35 m 需要放宽或让臂基座前移）")


def main(argv=None) -> int:
    args = parse_args(argv)
    g = geometry_of(args)
    print("=" * 58)
    print("握手可达性体检 —— 六舵机臂 / 地面坐标系 / 单位 cm")
    print("=" * 58)
    print_summary(g)
    print_band(g)
    print_grid(g, args.distances, args.heights)
    print_stop_distance(g, args.heights)
    print_shoulder_sweep(args, args.heights)
    print_verdict(g)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
