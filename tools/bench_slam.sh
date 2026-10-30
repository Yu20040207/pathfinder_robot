#!/usr/bin/env bash
# 用录好的 bag 离线回放，跑指定 SLAM 方案并保存地图，然后打分。
# 同一个 bag → 完全相同的输入，结果才有可比性。
#
# 用法：
#   bash tools/bench_slam.sh <bag目录> slam          # slam_toolbox（当前参数）
#   bash tools/bench_slam.sh <bag目录> slam_lowfov   # slam_toolbox（低视场参数：0.25/0.25）
#   bash tools/bench_slam.sh <bag目录> carto         # Cartographer（当前参数）
#   bash tools/bench_slam.sh <bag目录> all           # 依次跑全部
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${1:?用法: bench_slam.sh <bag目录> <slam|slam_lowfov|carto|all>}"
ENGINE="${2:?缺少方案名}"
[ -d "$BAG" ] || { echo "bag 不存在: $BAG" >&2; exit 1; }

set +u
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fwr/fastdds_udp_only.xml}"

SHARE="$ROOT/ros2_ws/install/fwr_bringup/share/fwr_bringup"
RESULT_ROOT="$HOME/fwr_maps/bench"
mkdir -p "$RESULT_ROOT"

stop_all() {
  python3 - <<'PY'
import os, signal, subprocess, time
PAT = ('slam_toolbox', 'cartographer', 'occupancy_grid_node', '/install/fwr_',
       'ros2 launch fwr_bringup', 'sllidar', 'dm_imu', 'rviz2', 'map_health',
       'base_bridge', 'ekf_filter', 'colorado')
out = subprocess.run(['ps', '-e', '-o', 'pid=,cmd='], capture_output=True, text=True).stdout
k = []
for line in out.splitlines():
    pid, _, cmd = line.strip().partition(' ')
    try:
        pid = int(pid)
    except ValueError:
        continue
    if pid == os.getpid() or cmd.startswith('bash -c') or 'ps -e -o pid=' in cmd:
        continue
    if any(p in cmd for p in PAT):
        try:
            os.kill(pid, signal.SIGKILL)
            k.append(pid)
        except OSError:
            pass
if k:
    print(f'  停止 {len(k)} 个旧进程')
    time.sleep(4)
PY
}

lowfov_params() {
  # 按 slam_toolbox 维护者建议的低视场参数生成一份临时配置
  local src="$SHARE/config/real_slam.yaml"
  local dst="$RESULT_ROOT/real_slam_lowfov.yaml"
  sed -e 's/^\( *minimum_travel_distance:\).*/\1 0.25/' \
      -e 's/^\( *minimum_travel_heading:\).*/\1 0.25/' \
      -e 's/^\( *loop_search_maximum_distance:\).*/\1 5.0/' "$src" > "$dst"
  echo "$dst"
}

run_engine() {   # run_engine <engine> <outdir>
  local eng="$1" out="$2"
  mkdir -p "$out"
  stop_all
  rm -f /dev/shm/fastrtps_* 2>/dev/null || true

  case "$eng" in
    slam|slam_lowfov)
      local params="$SHARE/config/real_slam.yaml"
      [ "$eng" = "slam_lowfov" ] && params="$(lowfov_params)"
      echo "  启动 slam_toolbox（$params）"
      nohup ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
        --params-file "$params" -p use_sim_time:=true \
        > "$out/slam.log" 2>&1 &
      ;;
    carto)
      echo "  启动 Cartographer"
      # 注意：cartographer 自己的 -configuration_* 必须放在 --ros-args 之前，
      # 否则会被当成未知 ROS 参数直接 abort（2026-10-30 实测）。
      nohup ros2 run cartographer_ros cartographer_node \
        -configuration_directory "$SHARE/config" \
        -configuration_basename fwr_2d.lua \
        --ros-args -p use_sim_time:=true \
        -r scan:=/scan_mapping -r odom:=/odom -r imu:=/imu/data_raw \
        > "$out/carto.log" 2>&1 &
      sleep 2
      nohup ros2 run cartographer_ros cartographer_occupancy_grid_node \
        -resolution 0.05 -publish_period_sec 1.0 \
        --ros-args -p use_sim_time:=true >> "$out/carto.log" 2>&1 &
      ;;
    *)
      echo "未知方案: $eng（可选 slam / slam_lowfov / carto）" >&2
      return 1
      ;;
  esac
  sleep 5

  echo "  回放 bag（--clock）…"
  ros2 bag play "$BAG" --clock --rate 1.0 > "$out/play.log" 2>&1 || true
  echo "  回放结束，等待优化收敛…"
  sleep 8

  echo "  保存地图"
  # 注意：bag 放完后 /clock 停止，use_sim_time 的节点时间会冻结 → map_saver 会卡住。
  # 它只是读 latched 的 /map 写文件，用系统时钟即可。
  timeout 40 ros2 run nav2_map_server map_saver_cli -f "$out/map" \
    --ros-args -p save_map_timeout:=15.0 > "$out/save.log" 2>&1 || true

  stop_all
  if [ -f "$out/map.yaml" ]; then
    echo "  ✅ 已保存 $out/map.yaml"
  else
    echo "  ❌ 没有生成地图，见 $out/*.log"
  fi
}

echo "== bag: $BAG =="
if [ "$ENGINE" = "all" ]; then
  for e in slam slam_lowfov carto; do
    echo
    echo "-------- 方案 $e --------"
    run_engine "$e" "$RESULT_ROOT/${e}_$(date +%m%d_%H%M%S)"
  done
else
  run_engine "$ENGINE" "$RESULT_ROOT/${ENGINE}_$(date +%m%d_%H%M%S)"
fi

echo
echo "== 打分（所有结果）=="
mapfiles=$(find "$RESULT_ROOT" -name 'map.yaml' -newer "$BAG" 2>/dev/null | sort)
if [ -n "$mapfiles" ]; then
  .venv/bin/python "$ROOT/tools/map_quality.py" $mapfiles || true
else
  echo "  没有找到结果地图"
fi
