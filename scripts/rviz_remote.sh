#!/usr/bin/env bash
# Show the robot's RViz on this workstation (the robot itself runs headless).
#
#   ./scripts/rviz_remote.sh                 # default localization layout
#   ./scripts/rviz_remote.sh -d other.rviz   # custom layout
#
# Prerequisites:
#   * the workstation and the robot share a LAN and the same ROS_DOMAIN_ID
#     (default 0) -- no ROS_DOMAIN_ID is exported anywhere in this project;
#   * the robot runs scripts/start_navigation_headless.sh.
#
# The RViz instance subscribes to the robot's /map, /scan, /tf and /plan over
# DDS, so the two-point panel works exactly as it does on the robot.
set -eo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslog}"
mkdir -p "$ROS_LOG_DIR"

CONFIG="$ROOT/ros2_ws/install/fwr_bringup/share/fwr_bringup/config/real_localization.rviz"
if [[ "${1:-}" == "-d" && -n "${2:-}" ]]; then
  CONFIG="$2"
  shift 2
fi

if [[ "${FWR_SKIP_CHECK:-0}" != "1" ]]; then
  echo "检查主控话题（ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}）..."
  if ! timeout 10 ros2 topic list 2>/dev/null | grep -qx /scan; then
    echo "警告：10 秒内没看到 /scan —— 可能不在同一网段/域，或主控导航未启动。" >&2
    echo "      仍然尝试启动 RViz；可用 FWR_SKIP_CHECK=1 跳过本检查。" >&2
  else
    echo "已看到 /scan，启动 RViz"
  fi
fi

exec rviz2 -d "$CONFIG" "$@"
