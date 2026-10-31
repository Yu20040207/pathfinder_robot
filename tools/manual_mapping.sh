#!/usr/bin/env bash
# 人工驾驶建图 + 记录路线（learning from demonstration）
#
# 用法：
#   bash tools/manual_mapping.sh 出门左转        # 开始：建图栈 + 录制
#   （另开终端用键盘开：bash tools/kb.sh）
#   结束时 Ctrl+C 这个脚本 → 自动保存路线
#
# 之后复现：
#   bash tools/replay_route.sh 出门左转
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${1:?用法: manual_mapping.sh <路线名>}"
set +u
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fwr/fastdds_udp_only.xml}"
mkdir -p /tmp/fwr_map ~/.fwr_routes

cleanup() {
  echo
  echo "== 保存路线并停止 =="
  python3 - "$ROOT" <<'PY'
import os, signal, subprocess, sys, time
out = subprocess.run(['ps', '-e', '-o', 'pid=,cmd='], capture_output=True, text=True).stdout
for line in out.splitlines():
    pid, _, cmd = line.strip().partition(' ')
    try:
        pid = int(pid)
    except ValueError:
        continue
    if pid == os.getpid():
        continue
    if 'route_node' in cmd and 'mode:=record' in cmd:
        os.kill(pid, signal.SIGINT)      # 让它走 finally 保存
        time.sleep(2)
PY
  bash "$ROOT/tools/start_mapping.sh" stop >/dev/null 2>&1 || true
}
trap cleanup EXIT

bash "$ROOT/tools/start_mapping.sh" --engine carto >/tmp/fwr_map/manual.log 2>&1 || true
sleep 12
ros2 run fwr_bringup route_node --ros-args -p mode:=record -p name:="$NAME" \
  >/tmp/fwr_map/route.log 2>&1 &
echo "== 录制中（路线名 $NAME）=="
echo "  另开终端开车：bash tools/kb.sh"
echo "  结束：Ctrl+C（会自动保存路线）"
wait
