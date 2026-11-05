#!/usr/bin/env bash
# One command to switch from any old ROS stack to saved-map navigation.
#
#   ./scripts/run_navigation.sh
#   ./scripts/run_navigation.sh --map /abs/path/to/real_map.yaml
#   ./scripts/run_navigation.sh --no-stop      # old stack already stopped
#   ./scripts/run_navigation.sh --no-rviz
#   ./scripts/run_navigation.sh --print-map    # show resolved map, do not start
#
# Without --map it reuses the map from the previous run (state file:
# ~/.config/fwr/last_navigation_map), falling back to the newest
# maps/real_map_auto_*.yaml.
set -eo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
STATE_FILE="${FWR_NAV_LAST_MAP:-$HOME/.config/fwr/last_navigation_map}"
MAP=""
STOP=1
RVIZ=true
PRINT_MAP=0

usage() {
  sed -n '2,9p' "$0"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) MAP="${2:-}"; shift 2 ;;
    --no-stop) STOP=0; shift ;;
    --no-rviz) RVIZ=false; shift ;;
    --print-map) PRINT_MAP=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done

# ROS setup.bash uses unset variables; keep -u off while sourcing it.
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u

export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fwr/fastdds_udp_only.xml}"
export DISPLAY="${DISPLAY:-:1}"

if [[ -n "$MAP" ]]; then
  if [[ ! -f "$MAP" ]]; then
    echo "地图不存在: $MAP" >&2
    exit 1
  fi
  MAP=$(realpath -- "$MAP")
  mkdir -p "$(dirname "$STATE_FILE")"
  printf '%s\n' "$MAP" > "$STATE_FILE"
  echo "已记录本次地图: $MAP"
elif [[ -s "$STATE_FILE" ]]; then
  MAP=$(head -n 1 "$STATE_FILE")
  if [[ ! -f "$MAP" ]]; then
    echo "上次地图已不存在: $MAP；回退到最新 real_map_auto_*.yaml" >&2
    MAP=""
  fi
fi

if [[ -z "$MAP" ]]; then
  MAP=$(ls -t "$ROOT"/ros2_ws/src/fwr_bringup/maps/real_map_auto_*.yaml 2>/dev/null | head -1 || true)
  if [[ -n "$MAP" ]]; then
    MAP=$(realpath -- "$MAP")
    mkdir -p "$(dirname "$STATE_FILE")"
    printf '%s\n' "$MAP" > "$STATE_FILE"
  fi
fi

if [[ -z "$MAP" || ! -f "$MAP" ]]; then
  echo "找不到存档地图；请用 --map 指定 .yaml，或先保存 real_map_auto_*.yaml。" >&2
  exit 1
fi

if [[ "$PRINT_MAP" -eq 1 ]]; then
  echo "$MAP"
  exit 0
fi

if [[ "$STOP" -eq 1 ]]; then
  echo "== 关闭旧栈运动门并停止建图 =="
  # Best-effort: stop the gate-opening mapping supervisor/planner and close
  # both motion paths before killing processes.  If /base_bridge is absent
  # these commands simply time out and are ignored.
  timeout 5 ros2 param set /base_bridge motion_enabled false >/dev/null 2>&1 || true
  timeout 5 ros2 param set /base_bridge manual_enabled false >/dev/null 2>&1 || true
  timeout 5 ros2 service call /explore/stop std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  timeout 5 ros2 topic pub --once /mapping/run std_msgs/msg/Bool "{data: false}" \
    >/dev/null 2>&1 || true
  sleep 1

  echo "== 停止旧 ROS 栈 =="
  bash "$ROOT/tools/start_mapping.sh" stop
  sleep 8
  # Prefer the process check: `ros2 node list` may still serve a cached
  # /base_bridge from the daemon for a few seconds after the process dies.
  if pgrep -x base_bridge >/dev/null 2>&1; then
    echo "旧栈仍有 base_bridge 进程；请检查进程后再运行，避免串口/AMCL/SLAM 冲突。" >&2
    exit 1
  fi
fi

echo "== 启动导航 =="
echo "  地图: $MAP"
echo "  RViz: $RVIZ"
echo "  DDS : $FASTRTPS_DEFAULT_PROFILES_FILE"
echo "  运动门保持关闭；在 RViz 面板点“开始导航”时由面板自行开门。"
echo

exec ros2 launch fwr_bringup run_navigation.launch.py \
  "map:=$MAP" \
  "start_rviz:=$RVIZ"
