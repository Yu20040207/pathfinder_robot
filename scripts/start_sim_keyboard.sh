#!/usr/bin/env bash
# Keyboard control for the isolated Gazebo corridor; never opens real hardware.
set -eo pipefail

PROJECT=/home/manbaout/Company_Project/Four-wheel-robot
export ROS_DOMAIN_ID=166
source /opt/ros/humble/setup.bash
source "$PROJECT/ros2_ws/install/setup.bash"
set -u

if [[ ! -t 0 ]]; then
  echo '请在接收按键的交互式终端运行。' >&2
  exit 1
fi

nodes=$(timeout 5 ros2 node list)
if grep -Fxq /base_bridge <<< "$nodes"; then
  echo '仿真域发现实车底盘桥接，拒绝启动。' >&2
  exit 1
fi
for node in /corridor_bridge /slam_toolbox; do
  if ! grep -Fxq "$node" <<< "$nodes"; then
    echo "缺少仿真节点 $node；先启动 corridor_sim.launch.py。" >&2
    exit 1
  fi
done
if grep -Fxq /fwr_keyboard_teleop <<< "$nodes"; then
  echo '已有键盘控制节点在运行；先退出旧窗口。' >&2
  exit 1
fi

if [[ -z "${DISPLAY:-}" ]] || ! command -v xset >/dev/null ||
   ! command -v gsettings >/dev/null; then
  echo '需要 X11、xset 和 GNOME gsettings 保证长按指令连续。' >&2
  exit 1
fi
original_repeat=$(xset q | awk '/auto repeat delay:/ {print $4, $7; exit}')
keyboard_schema=org.gnome.desktop.peripherals.keyboard
original_gnome_delay=$(gsettings get "$keyboard_schema" delay | awk '{print $2}')
original_gnome_interval=$(gsettings get "$keyboard_schema" repeat-interval | awk '{print $2}')
original_gnome_repeat=$(gsettings get "$keyboard_schema" repeat)
if [[ ! "$original_repeat" =~ ^[0-9]+\ [0-9]+$ ||
      ! "$original_gnome_delay" =~ ^[0-9]+$ ||
      ! "$original_gnome_interval" =~ ^[0-9]+$ ||
      ! "$original_gnome_repeat" =~ ^(true|false)$ ]]; then
  echo '无法读取键盘重复设置。' >&2
  exit 1
fi

cleanup() {
  timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
  gsettings set "$keyboard_schema" delay "$original_gnome_delay" >/dev/null 2>&1 || true
  gsettings set "$keyboard_schema" repeat-interval "$original_gnome_interval" >/dev/null 2>&1 || true
  gsettings set "$keyboard_schema" repeat "$original_gnome_repeat" >/dev/null 2>&1 || true
  xset r rate $original_repeat >/dev/null 2>&1 || true
  echo '仿真键盘控制已退出。'
}
trap cleanup EXIT

gsettings set "$keyboard_schema" repeat true
gsettings set "$keyboard_schema" delay 120
gsettings set "$keyboard_schema" repeat-interval 30
xset r rate 120 30

echo '仿真键盘已就绪：W/S 前后，A/D 左右，空格停，Q 退出。'
ros2 run fwr_bringup keyboard_teleop --ros-args -r /cmd_vel_manual:=/cmd_vel
