#!/usr/bin/env bash
# 一键开工：开门 + 起键盘遥控（建图用）。
# 可以在任何新终端直接运行，脚本会自己 source ROS 与工作区。
set -eo pipefail

# 找到仓库根目录（脚本在 tools/ 下）
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 环境：没装过就 source，已配过则不重复
if ! command -v ros2 >/dev/null 2>&1; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
if [ -z "${FWR_WS_SOURCED:-}" ] && [ -f "$ROOT/ros2_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$ROOT/ros2_ws/install/setup.bash"
    export FWR_WS_SOURCED=1
fi
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fwr/fastdds_udp_only.xml}"

ros2 param set /base_bridge motion_enabled true >/dev/null
ros2 param set /base_bridge manual_enabled true >/dev/null
echo "运动门已开（motion_enabled=true, manual_enabled=true）"

# 建图用 0.10 m/s（慢一点地图更稳）；转弯 0.4 rad/s
exec ros2 run fwr_bringup keyboard_teleop --ros-args \
     -p forward_speed_mps:=0.10 -p turn_rate_rad_s:=0.4
