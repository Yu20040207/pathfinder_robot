#!/usr/bin/env bash
# 建图守护：每 8 秒自动存一个滚动检查点（保留最近 4 个），
# 需要"回退"时把它们当时间机器用。用法：tools/map_watch.sh [间隔秒]
set -eo pipefail
INTERVAL="${1:-8}"
DIR=/tmp/fwr_ckpt
mkdir -p "$DIR"
echo "[map_watch] 每 ${INTERVAL}s 存一个检查点到 $DIR（保留 4 个）"
i=0
while true; do
  slot=$(( i % 4 ))
  ros2 service call /slam_toolbox/serialize_map \
      slam_toolbox/srv/SerializePoseGraph "{filename: '$DIR/slot${slot}'}" >/dev/null 2>&1 || true
  i=$(( i + 1 ))
  sleep "$INTERVAL"
done
