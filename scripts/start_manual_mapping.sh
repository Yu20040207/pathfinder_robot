#!/usr/bin/env bash
# One-command manual mapping: stop old stack -> SLAM Toolbox mapping + RViz
# -> verified keyboard teleop in this terminal.
#
# Usage:
#   ./scripts/start_manual_mapping.sh
#
# The keyboard window must stay focused.  W/S = forward/back, A/D = turn,
# Space = stop, Q = quit.  The script opens the software motion gates through
# start_keyboard_mapping.sh; keep the robot supervised and 12 V reachable.
set -eo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  sed -n '2,10p' "$0"
}

case "${1:-}" in
  -h|--help) usage; exit 0 ;;
  "") ;;
  *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
esac

if [[ ! -t 0 ]]; then
  echo "请在交互式终端运行：键盘输入必须进入这个终端。" >&2
  exit 1
fi

cat <<'EOF'
== 手动建图 ==
将启动：SLAM Toolbox 建图 + real_mapping.rviz + 键盘遥控。
注意：键盘脚本会打开 motion_enabled/manual_enabled；请确认场地空旷、
有人看护、可立即断开 12 V。W/S 前进后退，A/D 原地转，空格停机，Q 退出。
EOF

# RViz 的「保存地图」按钮会把栅格图和位姿图存到这里；start_mapping.sh
# 启动的 launch 会继承该环境变量。
export FWR_MAP_DIR="${FWR_MAP_DIR:-$ROOT/ros2_ws/src/fwr_bringup/maps}"

# 复用已验证的建图入口：--slam 选择 slam_toolbox，默认 Cartographer 不适用
# 于当前手动建图流程；该脚本会先停掉旧 ROS 栈并等待 /scan、/map。
bash "$ROOT/tools/start_mapping.sh" --slam

# start_mapping.sh 已把 SLAM + RViz 放到后台；当前终端留给键盘。
# start_keyboard_mapping.sh 会检查 /base_bridge、/sllidar_node、/dm_imu_l1、
# /slam_toolbox、/base/fault，然后才打开运动门。
set +e
bash "$ROOT/scripts/start_keyboard_mapping.sh"
rc=$?
set -e

stem="$ROOT/ros2_ws/src/fwr_bringup/maps/real_map_manual_$(date +%m%d_%H%M)"
cat <<EOF

== 键盘控制已退出（exit=$rc）==
建图栈仍在后台运行。保存本局地图：

  source /opt/ros/humble/setup.bash
  source $ROOT/ros2_ws/install/setup.bash
  ros2 run nav2_map_server map_saver_cli -f $stem
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \\
    "{filename: '$stem'}"

停止建图栈：

  bash $ROOT/tools/start_mapping.sh stop
EOF

exit "$rc"
