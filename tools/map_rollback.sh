#!/usr/bin/env bash
# 回退到某个滚动检查点（用 slam_toolbox 的 deserialize_map，不重启节点）。
#   tools/map_rollback.sh          列出可用的检查点
#   tools/map_rollback.sh 2        回退到 slot2
set -eo pipefail
DIR=/tmp/fwr_ckpt
if [ -z "${1:-}" ]; then
  echo "可用检查点（按时间从新到旧）："
  ls -lt "$DIR"/slot*.posegraph 2>/dev/null | awk '{print "  " $NF, $6, $7, $8}' || echo "  （还没有检查点）"
  echo
  echo "用法： tools/map_rollback.sh <slot编号，例如 2>"
  exit 0
fi
SLOT="$1"
FILE="$DIR/slot${SLOT}"
if [ ! -f "${FILE}.posegraph" ] && [ ! -f "${FILE}.data" ]; then
  echo "找不到检查点 $FILE" >&2
  exit 1
fi
echo "回退到 $FILE ..."
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph \
  "{filename: '$FILE', match_type: 1}"
