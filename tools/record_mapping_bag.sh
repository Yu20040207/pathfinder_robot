#!/usr/bin/env bash
# 录一段建图数据（用于离线回放对比不同 SLAM 方案）。
# 只录 SLAM 需要的输入，回放时不需要连接任何硬件。
#
# 用法：
#   bash tools/record_mapping_bag.sh               # 录制，Ctrl+C 结束
#   bash tools/record_mapping_bag.sh 我的一趟       # 指定名字
#
# 录完会打印 bag 路径；接着用 tools/bench_slam.sh 回放。
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${1:-map_$(date +%m%d_%H%M)}"
OUT="$HOME/fwr_bags/$NAME"
mkdir -p "$(dirname "$OUT")"

set +u
if ! command -v ros2 >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [ -z "${FWR_WS_SOURCED:-}" ] && [ -f "$ROOT/ros2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$ROOT/ros2_ws/install/setup.bash"
  export FWR_WS_SOURCED=1
fi
set -u
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fwr/fastdds_udp_only.xml}"

TOPICS=(/scan_mapping /scan /odom /wheel/odom /tf /tf_static /imu/data_raw)

echo "== 检查话题 =="
missing=""
for t in "${TOPICS[@]}"; do
  if ! ros2 topic list 2>/dev/null | grep -qx "$t"; then
    missing="$missing $t"
  fi
done
if [ -n "$missing" ]; then
  echo "  缺少话题:$missing" >&2
  echo "  先启动建图栈：bash tools/start_mapping.sh" >&2
  exit 1
fi
echo "  全部在线 ✅"

echo
echo "== 开始录制 =="
echo "  输出: $OUT"
echo "  话题: ${TOPICS[*]}"
echo "  按 Ctrl+C 结束录制"
echo
exec ros2 bag record -o "$OUT" "${TOPICS[@]}"
