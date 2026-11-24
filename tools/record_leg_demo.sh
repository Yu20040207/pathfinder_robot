#!/usr/bin/env bash
# 录制「雷达腿跟踪」现场标定数据（2026-11-24 P1）
#
# 用法（在车上，先由人确认可以启动/录制）：
#   bash tools/record_leg_demo.sh [输出目录]
#
# 录进来的内容：
#   /scan_merged                 360° 合并扫描（算法的唯一输入）
#   /odom                        车体位姿（把点变换到 odom 系）
#   /robot/lidar/persons         本节点输出（轨迹 id/位置/速度）
#   /robot/vision/persons        相机身份（用于"相机背书"门控）
#   /follow/state               跟随状态（P2 时对照）
#   /tf /tf_static               各 frame 关系
#   /robot/camera/color/image_raw/compressed   视频（可选，默认不录以免占空间）
#
# 走完 4 个场景（定点站立 / 直线走近 / 绕车一圈 / 反例：门框·桌腿·椅子·垃圾桶）
# 后 Ctrl-C，把整个目录拷回来；之后可以 `ros2 bag play` 反复回放调参，不用再走第二遍。
set -euo pipefail

OUT=${1:-"$HOME/leg_demo_$(date +%Y%m%d_%H%M%S)"}
WITH_IMAGE=${WITH_IMAGE:-0}

mkdir -p "$OUT"
echo "================================================================"
echo "录制目录：$OUT"
echo "建议场景（每个 1~2 分钟）："
echo "  ① 定点站立：车头正前 1 m / 2 m / 3 m；左前 45°、正左、正后 180°"
echo "  ② 直线走近：3 m → 1 m 再退回"
echo "  ③ 绕车一圈：前 → 右侧 → 后 → 左侧 → 回前"
echo "  ④ 反例：从门框穿过 / 从桌腿旁走过 / 椅子静止放 20 s / 走过垃圾桶"
echo "================================================================"

TOPICS=(/scan_merged /odom /robot/lidar/persons /robot/vision/persons
        /follow/state /tf /tf_static)
if [ "$WITH_IMAGE" = "1" ]; then
  TOPICS+=(/robot/camera/color/image_raw/compressed)
  echo "（包含摄像头视频，磁盘消耗约 40 MB/分钟）"
fi

echo "开始录制：${TOPICS[*]}"
ros2 bag record -o "$OUT/bag" "${TOPICS[@]}"
