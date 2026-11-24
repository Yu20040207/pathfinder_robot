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
START_LIDAR=true

usage() {
  sed -n '2,9p' "$0"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) MAP="${2:-}"; shift 2 ;;
    --no-stop) STOP=0; shift ;;
    --no-rviz) RVIZ=false; shift ;;
    --print-map) PRINT_MAP=1; shift ;;
    # 2026-11-12：雷达交给外部看门狗独占（launch 与看门狗同时开串口会让
    # sllidar_node 死锁在 futex 上，/scan 每 ~15 s 断一次）。
    --no-lidar) START_LIDAR=false; shift ;;
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

# 2026-11-24：**没有旧栈时整套清理都是空转** —— 4 个 ros2 CLI 每个都要新起
# 一个 DDS 客户端（没节点就等超时，实测合计 ~12 s），再加 sleep 8 + sleep 2。
# 开机第一次启动时这里白白多花 ~28 s（65 s 的启动里有 48 s 是收尾）。
# 先看一眼有没有旧栈，有才清场。
if [[ "$STOP" -eq 1 ]] && ! pgrep -f "[f]wr_bringup/base_bridge" >/dev/null 2>&1; then
  echo "== 没有旧栈在跑，跳过清理（省约 28 s）=="
elif [[ "$STOP" -eq 1 ]]; then
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
  # 2026-11-24：原来固定 sleep 8 —— 实测通常 1~3 s 就退干净了。
  # 改成轮询（最多 8 s），快的时候立刻往下走。
  for _ in $(seq 1 16); do
    pgrep -f "[f]wr_bringup/base_bridge|[s]lam_toolbox|[a]mcl_node" >/dev/null 2>&1 \
      || break
    sleep 0.5
  done
  # 2026-11-12：原来用 `pgrep -x base_bridge` 判断，但进程名其实是 python3
  # （comm 不等于 base_bridge），这条检查从来没生效 —— 结果残留的 base_bridge
  # 会占着 ESP32 串口，新起的实例看起来"一切正常"却写不出任何指令
  # （现象：面板发 0.2、[manual] target=(200,200)，但固件 setpoint 恒 0）。
  pkill -f "[f]wr_bringup/base_bridge" 2>/dev/null || true
  for _ in $(seq 1 8); do
    pgrep -f "[f]wr_bringup/base_bridge" >/dev/null 2>&1 || break
    sleep 0.25
  done
  if pgrep -f "[f]wr_bringup/base_bridge" >/dev/null 2>&1; then
    echo "旧栈仍有 base_bridge 进程；请检查进程后再运行，避免串口/AMCL/SLAM 冲突。" >&2
    exit 1
  fi
fi

# 2026-11-24：串口检查从"清理分支"里提出来 —— 跳过清理时也要确认 ESP32 没被
# 幽灵句柄占住（否则新实例看起来正常却写不出指令：面板发 0.2、固件 setpoint 恒 0）。
if [[ "$STOP" -eq 1 ]]; then
  ESP_PORT=$(ls /dev/serial/by-id/ 2>/dev/null | grep -i "Espressif" | head -1)
  if [ -n "$ESP_PORT" ] && [ -e "/dev/serial/by-id/$ESP_PORT" ]; then
    if fuser -s "/dev/serial/by-id/$ESP_PORT" 2>/dev/null; then
      echo "ESP32 串口仍被占用（可能是残留进程）：$(fuser /dev/serial/by-id/$ESP_PORT 2>&1)" >&2
      exit 1
    fi
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
  "start_rviz:=$RVIZ" \
  "start_lidar:=$START_LIDAR"
