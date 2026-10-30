#!/usr/bin/env bash
# 一键开始建图（Cartographer 或 slam_toolbox）+ RViz + 漂移监视。
#
# 用法：
#   bash tools/start_mapping.sh            # 用 Cartographer（推荐）
#   bash tools/start_mapping.sh --slam     # 用 slam_toolbox
#   bash tools/start_mapping.sh stop       # 停止建图相关进程
#
# 建图结束用 tools/cartographer_save.sh 存档（会同时存 pbstream，可续建）。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENGINE="carto"
ACTION="start"
for arg in "$@"; do
  case "$arg" in
    --slam) ENGINE="slam" ;;
    --carto) ENGINE="carto" ;;
    stop|--stop) ACTION="stop" ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "未知参数: $arg（--help 查看用法）" >&2; exit 2 ;;
  esac
done

# ---------- 环境 ----------
# ROS 的 setup.bash 会引用未定义变量，source 期间必须放开 -u
set +u
if ! command -v ros2 >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [ -z "${FWR_WS_SOURCED:-}" ] && [ -f "$ROOT/ros2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$ROOT/ros2_ws/install/setup.bash"
  export FWR_WS_SOURCED=1
fi
set -u
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fwr/fastdds_udp_only.xml}"
export DISPLAY="${DISPLAY:-:0}"

LOG_DIR=/tmp/fwr_map
mkdir -p "$LOG_DIR"

if [ -t 1 ]; then
  OK=$'\033[32m'; WARN=$'\033[33m'; ERR=$'\033[31m'; HDR=$'\033[36m'; OFF=$'\033[0m'
else
  OK=""; WARN=""; ERR=""; HDR=""; OFF=""
fi
ok()   { printf '%s  ✓ %s%s\n' "$OK" "$*" "$OFF"; }
warn() { printf '%s  ! %s%s\n' "$WARN" "$*" "$OFF"; }
die()  { printf '%s  ✗ %s%s\n' "$ERR" "$*" "$OFF" >&2; exit 1; }
step() { printf '\n%s== %s ==%s\n' "$HDR" "$*" "$OFF"; }

# 停止所有与本项目相关的进程（不匹配自己的命令行）
stop_all() {
  python3 - "$@" <<'PY'
import os, signal, subprocess, time
PATTERNS = ('ros2 launch fwr_bringup', '/opt/ros/humble/lib/', '/install/fwr_',
            'sllidar', 'dm_imu', 'rviz2', 'map_health', 'map_keep_on',
            'cartographer', 'chain_watchdog')
out = subprocess.run(['ps', '-e', '-o', 'pid=,cmd='], capture_output=True, text=True).stdout
killed = []
for line in out.splitlines():
    pid, _, cmd = line.strip().partition(' ')
    try:
        pid = int(pid)
    except ValueError:
        continue
    if pid == os.getpid():
        continue
    if cmd.startswith('bash -c') or 'ps -e -o pid=' in cmd:
        continue
    if any(p in cmd for p in PATTERNS):
        try:
            os.kill(pid, signal.SIGKILL)
            killed.append(pid)
        except OSError:
            pass
print(f'已停止 {len(killed)} 个进程')
time.sleep(5)
PY
}

wait_topic() {   # wait_topic <topic> <秒>
  local topic="$1" deadline=$((SECONDS + ${2:-20}))
  while [ $SECONDS -lt $deadline ]; do
    if timeout 3 ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      local rate
      rate=$(timeout 4 ros2 topic hz "$topic" 2>&1 | head -1 || true)
      case "$rate" in
        *"average rate"*) printf '%s' "$rate"; return 0 ;;
      esac
    fi
    sleep 1
  done
  return 1
}

if [ "$ACTION" = "stop" ]; then
  step "停止建图相关进程"
  stop_all
  ok "已停止"
  exit 0
fi

# ---------- 前置检查 ----------
step "检查硬件"
missing=""
ls /dev/serial/by-id/ 2>/dev/null | grep -q 'Espressif' || missing="$missing ESP32"
ls /dev/serial/by-id/ 2>/dev/null | grep -q 'DM-Tech_DM-IMU' || missing="$missing IMU"
ls /dev/serial/by-id/ 2>/dev/null | grep -q 'CP2102N' || missing="$missing 雷达"
if [ -n "$missing" ]; then
  die "缺少串口设备:$missing
  处理：拔插对应的 USB 线，再用 ls /dev/serial/by-id/ 确认三个设备都在。"
fi
ok "ESP32 / IMU / 雷达 都在"

step "清理旧进程"
stop_all
rm -f /dev/shm/fastrtps_* 2>/dev/null || true

# ---------- 启动 ----------
step "启动建图栈（$([ "$ENGINE" = carto ] && echo Cartographer || echo slam_toolbox)）"
if [ "$ENGINE" = "carto" ]; then
  LAUNCH=real_mapping_cartographer.launch.py
else
  LAUNCH=real_mapping.launch.py
fi
: > "$LOG_DIR/$ENGINE.log"
nohup ros2 launch fwr_bringup "$LAUNCH" >> "$LOG_DIR/$ENGINE.log" 2>&1 &
echo $! > "$LOG_DIR/launch.pid"
printf '  等待栈启动'
for _ in $(seq 1 40); do
  printf '.'
  if ros2 node list 2>/dev/null | grep -q "$([ "$ENGINE" = carto ] && echo cartographer_node || echo slam_toolbox)"; then
    break
  fi
  sleep 1
done
printf '\n'

# ---------- 验证 ----------
step "验证数据"
scan_rate=$(wait_topic /scan 25) || die "/scan 没数据（雷达没起来？看 $LOG_DIR/$ENGINE.log）"
ok "/scan  $scan_rate"
map_rate=$(wait_topic /map 25) || die "/map 没数据（SLAM 没起来？看 $LOG_DIR/$ENGINE.log）"
ok "/map   $map_rate"
if wait_topic /wheel/odom 10 >/dev/null; then
  ok "/wheel/odom 正常"
else
  warn "/wheel/odom 没数据（底盘串口？）"
fi

# ---------- RViz + 监视 ----------
step "打开 RViz 与漂移监视"
nohup rviz2 -d "$ROOT/ros2_ws/src/fwr_bringup/config/real_mapping.rviz" \
  > "$LOG_DIR/rviz.log" 2>&1 &
echo $! > "$LOG_DIR/rviz.pid"
sleep 4
if ps -p "$(cat "$LOG_DIR/rviz.pid")" >/dev/null 2>&1; then
  ok "RViz 已打开（Fixed Frame 应为 map）"
else
  warn "RViz 启动失败，看 $LOG_DIR/rviz.log"
fi
nohup .venv/bin/python "$ROOT/tools/map_health.py" > "$LOG_DIR/health.log" 2>&1 &
echo $! > "$LOG_DIR/health.pid"
ok "漂移监视 → $LOG_DIR/health.log"

cat <<EOF

${OK}建图已就绪${OFF}

  键盘遥控 : bash tools/kb.sh
  检查质量 : .venv/bin/python tools/map_align_check.py     （每扫完一个区域跑一次）
  漂移监视 : tail -f $LOG_DIR/health.log
  存档     : bash tools/cartographer_save.sh 我的图名
  停止     : bash tools/start_mapping.sh stop

  扫图要点：
    · 速度 0.10 m/s，点按
    · 墙角停 2 秒
    · 贴家具/门框走，增加特征
    · 最后一定回到起点闭环

EOF
