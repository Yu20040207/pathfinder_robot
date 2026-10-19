#!/usr/bin/env bash
# Start supervised keyboard driving after the real mapping stack is running.
set -eo pipefail

PROJECT=/home/manbaout/Company_Project/Four-wheel-robot
source /opt/ros/humble/setup.bash
source "$PROJECT/ros2_ws/install/setup.bash"
set -u
if [[ ! -t 0 ]]; then
  echo '请在交互式终端运行此脚本，键盘输入必须进入该终端。' >&2
  exit 1
fi

nodes=$(timeout 5 ros2 node list)
for node in /base_bridge /sllidar_node /dm_imu_l1 /slam_toolbox; do
  if ! grep -Fxq "$node" <<< "$nodes"; then
    echo "缺少节点 $node。先运行实机建图 launch，见 docs/hardware/keyboard-mapping.md。" >&2
    exit 1
  fi
done
if grep -Fxq /fwr_keyboard_teleop <<< "$nodes"; then
  echo '已有键盘控制节点在运行；先退出旧窗口。' >&2
  exit 1
fi

fault=$(timeout 5 ros2 topic echo /base/fault --once)
if ! grep -Eq '^data: 0$' <<< "$fault"; then
  echo "底盘故障未清零：$fault" >&2
  exit 1
fi

cleanup() {
  ros2 param set /base_bridge motion_enabled false >/dev/null 2>&1 || true
  ros2 param set /base_bridge manual_enabled false >/dev/null 2>&1 || true
  echo '键盘控制已退出，软件运动门已关闭。'
}
trap cleanup EXIT

ros2 param set /base_bridge manual_enabled true >/dev/null
ros2 param set /base_bridge motion_enabled true >/dev/null

ready=false
for _ in 1 2 3 4 5; do
  if timeout 4 ros2 topic echo /base/ready --once | grep -Eq '^data: true$'; then
    ready=true
    break
  fi
  sleep 0.2
done
if [[ "$ready" != true ]]; then
  echo '底盘尚未就绪；已取消键盘控制。检查雷达、IMU、串口和 /base/fault。' >&2
  exit 1
fi

echo '键盘已就绪。保持当前终端焦点；W/S 前后，A/D 左右，空格停，Q 退出。'
ros2 run fwr_bringup keyboard_teleop
