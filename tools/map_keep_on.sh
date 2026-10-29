#!/usr/bin/env bash
# 滚动检查点：仅在【车静止】时保存，且间隔较长，避免与 slam 改图并发冲突。
# （曾用 8s 高频保存，导致 slam_toolbox 崩溃：Mapper FATAL ERROR - unable to get
#   pointer in probability search!）
set -eo pipefail
DIR=/tmp/fwr_ckpt
mkdir -p "$DIR"
INTERVAL="${1:-60}"
echo "[checkpoint] 每 ${INTERVAL}s 检查一次，仅在车静止时保存；目录 $DIR"
last_x=""; last_y=""; i=0
while true; do
  sleep "$INTERVAL"
  pos=$(timeout 5 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null | tr '\n' ' ' || true)
  x=$(echo "$pos" | grep -o 'x: *[-0-9.]*' | head -1 | awk '{print $2}')
  y=$(echo "$pos" | grep -o 'y: *[-0-9.]*' | head -1 | awk '{print $2}')
  [ -z "$x" ] && continue
  if [ "$x" = "$last_x" ] && [ "$y" = "$last_y" ]; then
    timeout 30 ros2 service call /slam_toolbox/serialize_map \
      slam_toolbox/srv/SerializePoseGraph "{filename: '$DIR/slot$((i % 4))'}" >/dev/null 2>&1 || true
    i=$((i + 1))
    echo "[checkpoint] 已存 slot$(( (i-1) % 4 ))  车静止于 ($x,$y)"
  fi
  last_x="$x"; last_y="$y"
done
