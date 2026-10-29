#!/usr/bin/env bash
# 播种式扩建建图：
#   1) 起 slam（加载旧位姿图，默认 mapping 模式）
#   2) 切 localization 模式 → 用导航同款匹配 + /initialpose 播种
#   3) 切回 mapping 模式 → 继续扩建
set -eo pipefail
PROJ=/home/manbaout/Company_Project/Four-wheel-robot
PG_BASE=${1:?posegraph base name}
CFG_BASE=${2:?slam params base file}
REF_YAML=${3:-}
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.config/fwr/fastdds_udp_only.xml
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$PROJ/ros2_ws/install/setup.bash"
cd "$PROJ"

RUNTIME=/tmp/fwr_slam_runtime.yaml
sed "s#^    map_file_name: .*#    map_file_name: $PG_BASE#" "$CFG_BASE" > "$RUNTIME"

ros2 launch fwr_bringup real_mapping.launch.py \
  slam_executable:=map_and_localization_slam_toolbox_node \
  slam_params:="$RUNTIME" > /tmp/fwr_seed_sensors.log 2>&1 &
STACK_PID=$!

for _ in $(seq 1 60); do
  if timeout 3 ros2 service list 2>/dev/null | grep -q "slam_toolbox/set_localization_mode"; then break; fi
  sleep 1
done
sleep 6

echo "→ 切换到 localization 模式"
ros2 service call /slam_toolbox/set_localization_mode std_srvs/srv/SetBool "{data: true}" >/tmp/fwr_toggle_on.log 2>&1 || true
sleep 3

echo "→ 计算并播种位姿"
if .venv/bin/python tools/seed_slam_runtime.py ${REF_YAML:+--reference "$REF_YAML"}; then
  echo "→ 播种成功，切回 mapping 模式继续扩建"
  ros2 service call /slam_toolbox/set_localization_mode std_srvs/srv/SetBool "{data: false}" >/tmp/fwr_toggle_off.log 2>&1 || true
else
  echo "⚠ 播种失败，仍处于 localization 模式；请人工用 2D Pose Estimate 后切回 mapping" >&2
fi
wait $STACK_PID
