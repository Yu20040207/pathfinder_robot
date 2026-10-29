#!/usr/bin/env bash
# Requires a separately started real_navigation.launch.py. Physical motion is explicit.
set -euo pipefail
if [[ $# -ne 4 || $1 != --trial-area-confirmed ]]; then
  echo '用法: start_auto_mapping.sh --trial-area-confirmed <实测最大长度mm> <实测最大宽度mm> <实测停车距离mm>' >&2
  exit 2
fi
LENGTH_MM=$2
WIDTH_MM=$3
STOP_MM=$4
ROOT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/ros2_ws/install/setup.bash"
OUTPUT_DIR=${FWR_MAP_OUTPUT_DIR:-$HOME/robot_maps}
STARTED=0
CLEANED=0
cleanup() {
  if [[ $CLEANED -eq 1 ]]; then return; fi
  CLEANED=1
  set +e
  timeout 5 ros2 service call /explore/stop std_srvs/srv/Trigger '{}' >/dev/null 2>&1
  timeout 5 ros2 param set /base_bridge motion_enabled false >/dev/null 2>&1
  if [[ $STARTED -eq 1 ]]; then
    mkdir -p "$OUTPUT_DIR"
    MAP_PATH="$OUTPUT_DIR/auto_$(date +%Y%m%d_%H%M%S)"
    timeout 20 ros2 run nav2_map_server map_saver_cli -f "$MAP_PATH"
    echo "试验地图: $MAP_PATH.yaml"
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# A previous run must never arm this run. Close the gate before any preflight.
ros2 param set /base_bridge motion_enabled false >/dev/null
if ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q 'unconfigured\|inactive'; then
  timeout 40 ros2 service call /lifecycle_manager_navigation/manage_nodes \
    nav2_msgs/srv/ManageLifecycleNodes '{command: 0}' >/dev/null
fi
ros2 run fwr_bringup auto_mapping_preflight \
  --length-mm "$LENGTH_MM" --width-mm "$WIDTH_MM" --stop-mm "$STOP_MM"
# Recheck immediately before opening the gate.
PARAM_RESULT=$(ros2 param set /base_bridge motion_enabled true)
if ! grep -q 'Set parameter successful' <<< "$PARAM_RESULT"; then
  echo "无法开启运动门: $PARAM_RESULT" >&2
  exit 1
fi
ARM_RESULT=$(timeout 8 ros2 service call /base/arm_auto std_srvs/srv/Trigger '{}')
if ! grep -q 'success: true' <<< "$ARM_RESULT"; then
  echo "无法授权自动运动: $ARM_RESULT" >&2
  exit 1
fi
# Allow the explorer to receive the bridge's new armed state before start.
sleep 0.3
START_RESULT=$(timeout 8 ros2 service call /explore/start std_srvs/srv/Trigger '{}')
if ! grep -q 'success: true' <<< "$START_RESULT"; then
  echo "探索启动失败: $START_RESULT" >&2
  exit 1
fi
STARTED=1
echo '自动建图已显式启动；保持现场看护，Ctrl+C 停止并保存地图。'
# The explorer also has its own 120 s maximum trial timer.
sleep 125
