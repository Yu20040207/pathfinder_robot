#!/usr/bin/env bash
# 一键开工：开门 + 起键盘遥控（建图用）。
set -eo pipefail
ros2 param set /base_bridge motion_enabled true >/dev/null
ros2 param set /base_bridge manual_enabled true >/dev/null
echo "运动门已开（motion_enabled=true, manual_enabled=true）"
exec ros2 run fwr_bringup keyboard_teleop --ros-args \
     -p forward_speed_mps:=0.15 -p turn_rate_rad_s:=0.4
