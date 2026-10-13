#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
preview_dir="$project_root/docs/hardware/preview"
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${FWR_PREVIEW_DOMAIN_ID:-196}"
# Software rendering keeps this small preview compatible with more desktops.
# Set LIBGL_ALWAYS_SOFTWARE=0 to use the normal graphics driver.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

python3 "$project_root/scripts/generate_chassis_preview_mesh.py"
ros2 run robot_state_publisher robot_state_publisher "$preview_dir/fwr_chassis_preview.urdf" &
publisher_pid=$!
python3 "$project_root/scripts/animate_chassis_preview.py" &
animation_pid=$!
trap 'kill "$publisher_pid" "$animation_pid" 2>/dev/null || true' EXIT
rviz2 -d "$preview_dir/fwr_chassis_preview.rviz"
