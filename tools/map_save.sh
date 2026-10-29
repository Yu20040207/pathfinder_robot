#!/usr/bin/env bash
# 手动存一份完整检查点（建议先停车/冻结建图再执行）。
set -eo pipefail
DIR=/tmp/fwr_ckpt
mkdir -p "$DIR"
STAMP=$(date +%H%M%S)
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: '$DIR/manual_$STAMP'}"
echo "已保存到 $DIR/manual_$STAMP.posegraph"
