#!/usr/bin/env bash
# 手动沿边建图（一条命令搞定）
#
#   · 启动 Cartographer 建图
#   · 同步记录你开的路线（每 0.30 m 一个路点 → ~/.fwr_routes/<名字>.yaml）
#   · Ctrl+C 结束时自动：结束轨迹 → 存 pbstream → 存 pgm/yaml → 存路线
#
# 用法：
#   bash tools/manual_mapping.sh              # 名字 = manual_MMDD_HHMM
#   bash tools/manual_mapping.sh 出门左转      # 自己起名字
#
# 另开一个终端开车： bash tools/kb.sh
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${1:-manual_$(date +%m%d_%H%M)}"
set +u
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fwr/fastdds_udp_only.xml}"
mkdir -p /tmp/fwr_map ~/.fwr_routes ~/.carto_maps

cleanup() {
  echo
  echo "== 结束：保存路线 / 轨迹 / 地图 =="
  # 1) 让录制节点正常退出（它的 finally 会写 YAML）
  python3 - <<'PY'
import os, signal, subprocess, time
out = subprocess.run(['ps', '-e', '-o', 'pid=,cmd='], capture_output=True, text=True).stdout
for line in out.splitlines():
    pid, _, cmd = line.strip().partition(' ')
    try:
        pid = int(pid)
    except ValueError:
        continue
    if 'route_node' in cmd and 'mode:=record' in cmd and pid != os.getpid():
        try:
            os.kill(pid, signal.SIGINT)
        except OSError:
            pass
time.sleep(2)
PY
  # 2) Cartographer 存 pbstream + 栅格图
  bash "$ROOT/tools/cartographer_save.sh" "$NAME" 2>&1 | tail -4 || true
  # 3) 停掉其余节点
  bash "$ROOT/tools/start_mapping.sh" stop >/dev/null 2>&1 || true
  echo
  echo "存档："
  echo "  地图   ~/.carto_maps/$NAME.pbstream / .pgm / .yaml"
  echo "  路线   ~/.fwr_routes/$NAME.yaml"
  echo
  echo "复现这条路线： bash tools/replay_route.sh $NAME"
}
trap cleanup EXIT

echo "== 启动建图（Cartographer）=="
nohup ros2 launch fwr_bringup real_mapping_cartographer.launch.py \
  > /tmp/fwr_map/manual.log 2>&1 &
sleep 12

echo "== 启动路线记录（$NAME）=="
nohup ros2 run fwr_bringup route_node --ros-args \
  -p mode:=record -p name:="$NAME" > /tmp/fwr_map/route.log 2>&1 &
sleep 3

cat <<TIP

==================== 开始手动建图 ====================
另开一个终端开车：
    bash tools/kb.sh

提示：
  · 沿着墙边走（离墙 0.5~0.8 m 最稳）
  · 出门、进门都正常开，走完一圈回到起点
  · 走完按 Ctrl+C（这个终端）→ 自动保存地图 + 路线
=====================================================

TIP
wait
