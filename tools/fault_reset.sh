#!/usr/bin/env bash
# 一键恢复：失速(fault 1)/堵转后，清故障 + 重新开门。
# 用法：bash tools/fault_reset.sh   然后重开键盘节点。
set -eo pipefail
echo "1) 关闭运动门..."
ros2 param set /base_bridge motion_enabled false  >/dev/null
ros2 param set /base_bridge manual_enabled false  >/dev/null
sleep 0.5
echo "2) 清故障..."
ros2 service call /base/clear_fault std_srvs/srv/Trigger | tail -1
sleep 0.5
echo "3) 重新开门..."
ros2 param set /base_bridge motion_enabled true   >/dev/null
ros2 param set /base_bridge manual_enabled true   >/dev/null
sleep 1
echo -n "4) 当前故障码: "
ros2 topic echo /base/fault --once 2>/dev/null | tr -d '\n'
echo
echo "完成。重开键盘节点即可继续："
echo "  ros2 run fwr_bringup keyboard_teleop --ros-args -p forward_speed_mps:=0.15 -p turn_rate_rad_s:=0.4"
