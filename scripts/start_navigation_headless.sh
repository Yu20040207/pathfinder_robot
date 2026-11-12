#!/usr/bin/env bash
# Start saved-map navigation on the robot WITHOUT RViz.
#
# Use this when the main controller has no monitor: every GUI task (RViz, the
# two-point navigation panel) runs on the workstation through
# scripts/rviz_remote.sh, while this script only brings up the ROS stack.
#
#   ./scripts/start_navigation_headless.sh                  # last used map
#   ./scripts/start_navigation_headless.sh --map /abs/x.yaml
#   ./scripts/start_navigation_headless.sh --no-stop        # old stack is down
#
# Environment:
#   FWR_LOG_DIR   log directory (default /tmp/fwr_runs)
#
# The script detaches the launch with setsid+nohup so an SSH disconnect does
# not kill navigation. Follow the log with: tail -f /tmp/fwr_runs/nav.log
set -eo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${FWR_LOG_DIR:-/tmp/fwr_runs}"
LOG="$LOG_DIR/nav.log"

if [[ ! -x "$ROOT/scripts/run_navigation.sh" ]]; then
  echo "找不到可执行的 scripts/run_navigation.sh" >&2
  exit 1
fi
mkdir -p "$LOG_DIR"

# start_lidar:=false：雷达交给 systemd 看门狗独占。
# 2026-11-12 实测：launch 和看门狗同时开串口 → sllidar_node 死锁在 futex 上，
# /scan 每 15 秒断一次 → 底盘撤授权 → 导航原地转/中止。
setsid nohup "$ROOT/scripts/run_navigation.sh" --no-rviz --no-lidar "$@" \
  > "$LOG" 2>&1 < /dev/null &
pid=$!
disown || true

echo "导航已后台启动（进程 $pid，无 RViz）"
echo "  日志: $LOG"
echo "  跟踪: tail -f $LOG"
echo "  显示: 在开发机运行 ./scripts/rviz_remote.sh"
