#!/usr/bin/env bash
# 在已有 pbstream 上继续扩建地图（不用重扫）。
#
# 用法：
#   bash tools/continue_mapping.sh ~/.carto_maps/xxx.pbstream          # 手动键盘
#   bash tools/continue_mapping.sh ~/.carto_maps/xxx.pbstream auto     # 沿墙自动
#
# 前提：把机器人放在**已建图区域内**（Cartographer 会用已有子图定位）。
# 结束后用 bash tools/cartographer_save.sh <新名字> 存档（含合并后的 pbstream）。
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PB="${1:?用法: continue_mapping.sh <pbstream> [auto|manual]}"
MODE="${2:-manual}"
[ -f "$PB" ] || { echo "找不到 pbstream: $PB" >&2; exit 1; }
PB="$(readlink -f "$PB")"

set +u
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fwr/fastdds_udp_only.xml}"

SHARE="$ROOT/ros2_ws/install/fwr_bringup/share/fwr_bringup"
LOG=/tmp/fwr_map/continue.log
mkdir -p /tmp/fwr_map

echo "== 检查串口设备 =="
missing=""
ls /dev/serial/by-id/ 2>/dev/null | grep -q Espressif || missing="$missing ESP32"
ls /dev/serial/by-id/ 2>/dev/null | grep -q DM-Tech || missing="$missing IMU"
ls /dev/serial/by-id/ 2>/dev/null | grep -q CP2102N || missing="$missing 雷达"
[ -z "$missing" ] || { echo "缺少设备:$missing" >&2; exit 1; }
echo "  三个设备都在 ✅"

echo "== 启动底盘 + 雷达 + IMU + EKF =="
nohup ros2 launch fwr_bringup real_mapping.launch.py start_slam:=false \
  > "$LOG.base" 2>&1 &
sleep 12

echo "== 启动 Cartographer（载入已有地图）=="
echo "  $PB"
nohup ros2 run cartographer_ros cartographer_node \
  -configuration_directory "$SHARE/config" \
  -configuration_basename fwr_2d.lua \
  -load_state_filename "$PB" \
  --ros-args -r scan:=/scan_mapping -r odom:=/odom -r imu:=/imu/data_raw \
  > "$LOG" 2>&1 &
sleep 6
nohup ros2 run cartographer_ros cartographer_occupancy_grid_node \
  -resolution 0.05 -publish_period_sec 1.0 > "${LOG}.grid" 2>&1 &
sleep 6

echo "== 验证 =="
for t in /scan_mapping /map; do
  printf '  %-16s ' "$t"
  timeout 6 ros2 topic hz "$t" 2>&1 | head -1
done

if [ "$MODE" = "auto" ]; then
  echo "== 沿墙自动扩建 =="
  nohup ros2 run fwr_bringup wall_follower --ros-args \
    -p side:=right -p speed_mps:=0.12 > "${LOG}.wall" 2>&1 &
  echo "  wall_follower 已启动（日志 ${LOG}.wall）"
else
  echo "== 手动模式：用键盘开车 =="
  echo "  bash tools/kb.sh"
fi

cat <<EOF

继续建图中。结束时：
  bash tools/cartographer_save.sh 扩建_$(date +%m%d_%H%M)

停止全部：
  bash tools/start_mapping.sh stop

EOF
