#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="$project_root/ascam_ros2_ws"
if [[ ! -d "$workspace_root/src/ascamera" ]]; then
  echo "Missing vendor workspace: $workspace_root/src/ascamera" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
workspace_id="$(printf '%s' "$workspace_root" | sha256sum | cut -c1-12)"
build_base="$workspace_root/build_$workspace_id"
install_base="$workspace_root/install_$workspace_id"
orb_slam2_root="$workspace_root/third_party/ORB_SLAM2"
export ORB_SLAM2_ROOT_DIR="$orb_slam2_root"
export LD_LIBRARY_PATH="$orb_slam2_root/lib:$orb_slam2_root/Thirdparty/DBoW2/lib:$orb_slam2_root/Thirdparty/g2o/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

cd "$workspace_root"
colcon build --build-base "$build_base" --install-base "$install_base" \
  --symlink-install --cmake-args \
  -DCROSS_COMPILE=x86_64-linux-gnu \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPython_EXECUTABLE=/usr/bin/python3
echo "Build complete. Source $install_base/setup.bash"
