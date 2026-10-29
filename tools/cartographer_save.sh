#!/usr/bin/env bash
# Cartographer 保存地图：结束轨迹 → 写 pbstream → 转成 pgm/yaml。
# 用法：bash tools/cartographer_save.sh [输出名]
set -eo pipefail
NAME="${1:-carto_$(date +%m%d_%H%M)}"
DIR="$HOME/.carto_maps"
mkdir -p "$DIR"
echo "1) 结束轨迹..."
ros2 service call /finish_trajectory \
  cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
echo "2) 写出 pbstream（Cartographer 原生存档，可用于续建/定位）..."
ros2 service call /write_state \
  cartographer_ros_msgs/srv/WriteState "{filename: '$DIR/$NAME.pbstream'}"
echo "3) 转成 pgm + yaml..."
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename "$DIR/$NAME.pbstream" \
  -map_filestem "$DIR/$NAME" -resolution 0.05
echo
echo "已保存："
ls -1 "$DIR/$NAME"* 2>/dev/null
echo
echo "注意：finish_trajectory 之后本次建图结束。要继续建图需重启建图栈。"
