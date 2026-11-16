#!/usr/bin/env bash
# 遥控就绪：等 /base_bridge 起来后打开运动门，供手机/云端遥控使用。
#
# 为什么需要它：导航栈每次重启都会把 base_bridge 的 motion_enabled /
# manual_enabled / manual_allow_mixed 重置为默认 False，遥控会突然失效。
#
# 用法：
#   ./scripts/arm_remote.sh                  # 一次性开门（等 base_bridge 最多 120s）
#   ./scripts/arm_remote.sh --watch          # 守护模式：栈重启后自动再开门
#   FWR_MANUAL_MAX=0.30 ./scripts/arm_remote.sh
#
# ⚠️ 安全：开门后，任何能连上 MQTT（当前 1883 匿名）的客户端都能驱动这台车。
#    仅在可信网络/已加 MQTT 认证的环境使用；无人看管时请先关门。
set -eo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SPEED="${FWR_MANUAL_MAX:-0.30}"

set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslog}"

waited() { echo "[arm_remote] $(date +%H:%M:%S) $*"; }

node_alive() {
    timeout 6 ros2 param get /base_bridge motion_enabled >/dev/null 2>&1
}

wait_node() {
    for _ in $(seq 1 30); do
        if node_alive; then return 0; fi
        sleep 2
    done
    return 1
}

arm() {
    timeout 6 ros2 param set /base_bridge motion_enabled true >/dev/null 2>&1 || return 1
    timeout 6 ros2 param set /base_bridge manual_enabled true >/dev/null 2>&1 || return 1
    timeout 6 ros2 param set /base_bridge manual_allow_mixed true >/dev/null 2>&1 || return 1
    timeout 6 ros2 param set /base_bridge manual_max_speed_mps "$SPEED" >/dev/null 2>&1 || return 1
    return 0
}

armed_state() {
    timeout 6 ros2 param get /base_bridge manual_enabled 2>/dev/null | tail -1
}

if ! wait_node; then
    waited "60 秒内没等到 /base_bridge，放弃"
    exit 1
fi
if arm; then
    waited "运动门已打开（manual_max_speed_mps=${SPEED}）"
else
    waited "开门失败（参数服务无响应）"
    exit 1
fi

if [[ "${1:-}" == "--watch" ]]; then
    waited "守护模式：每 5 秒检查一次，导航栈重启后自动再开门"
    while true; do
        sleep 5
        if ! node_alive; then
            wait_node || continue
        fi
        if [[ "$(armed_state)" != *True* ]]; then
            arm && waited "检测到门被重置，已重新打开"
        fi
    done
fi
