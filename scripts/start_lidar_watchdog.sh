#!/usr/bin/env bash
# 后台启动雷达看门狗（/scan 断流自动重启 + 断流/恢复时清空代价地图）。
#
#   ./scripts/start_lidar_watchdog.sh              # 默认：stale 2s、冷却 15s、断流相关清空
#   ./scripts/start_lidar_watchdog.sh --clear-period 60   # 额外每 60s 清一次
#
# 日志：/tmp/fwr_runs/lidar_watchdog.log（可用 FWR_LOG_DIR 覆盖）
set -eo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${FWR_LOG_DIR:-/tmp/fwr_runs}"
LOG="$LOG_DIR/lidar_watchdog.log"
mkdir -p "$LOG_DIR"

pkill -f "tools/lidar_watchdog.py" 2>/dev/null || true
sleep 1
: > "$LOG"

cd "$ROOT"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fwr/fastdds_udp_only.xml}"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source ros2_ws/install/setup.bash

setsid --fork "$ROOT/.venv/bin/python" "$ROOT/tools/lidar_watchdog.py" "$@" \
  >> "$LOG" 2>&1 < /dev/null
sleep 2
if pgrep -f "tools/lidar_watchdog.py" > /dev/null; then
  echo "看门狗已启动（pid $(pgrep -f 'tools/lidar_watchdog.py' | head -1)）"
else
  echo "看门狗启动失败，检查 $LOG" >&2
  exit 1
fi
echo "  日志: $LOG"
