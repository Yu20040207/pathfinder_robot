#!/usr/bin/env bash
# 复现录好的路线（含出门/进门）
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${1:?用法: replay_route.sh <路线名>}"
[ -f "$HOME/.fwr_routes/$NAME.yaml" ] || { echo "找不到 ~/.fwr_routes/$NAME.yaml" >&2; exit 1; }
set +u
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fwr/fastdds_udp_only.xml}"
bash "$ROOT/tools/start_mapping.sh" --engine carto >/tmp/fwr_map/replay.log 2>&1 || true
sleep 12
ros2 run fwr_bringup route_node --ros-args -p mode:=follow -p name:="$NAME"
