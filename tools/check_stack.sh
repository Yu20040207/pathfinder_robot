#!/usr/bin/env bash
# 启动自检：确认"车能动"所需的每一环都在位。
# 用法: bash tools/check_stack.sh [日志文件]
# 2026-11-04 加：之前多次出现"节点崩了但界面看起来正常，车就是不动"。
LOG="${1:-}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fwr/fastdds_udp_only.xml}"
HERE="$(cd "$(dirname "$0")" && pwd)"
# ROS 的 setup.bash 里有未定义变量，不能开 set -u
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HERE/../ros2_ws/install/setup.bash" 2>/dev/null
fail=0
say() { printf '  %-34s %s\n' "$1" "$2"; }
bad() { fail=$((fail+1)); }

echo "=== ① 进程 ==="
for pat in 'sllidar_ros2/[l]ib' 'bt_[n]avigator' 'fwr_bringup/[b]ase_bridge' 'fwr_bringup/[f]rontier_explorer' 'rviz2'; do
  n=$(pgrep -c -f "$pat" 2>/dev/null || echo 0)
  [ "$n" -ge 1 ] && say "$pat" "✅ $n" || { say "$pat" "❌ 缺失"; bad; }
done

echo "=== ② 节点在 ROS 图里 ==="
for node in /bt_navigator /controller_server /planner_server /base_bridge /frontier_explorer; do
  if timeout 8 ros2 node list 2>/dev/null | grep -qx "$node"; then say "$node" "✅"; else say "$node" "❌ 不在图里"; bad; fi
done

echo "=== ③ 服务 ==="
for srv in /explore/start_fixed_goal /explore/start_two_point_goal /explore/start_multi_point_goal; do
  timeout 8 ros2 service list 2>/dev/null | grep -qx "$srv" && say "$srv" "✅" || { say "$srv" "❌"; bad; }
done

echo "=== ③.5 碰撞监控链路（"开始导航没反应"的隐藏元凶）==="
# 2026-11-18 现场：collision_monitor_forward 的 configure 超时
# （"failed to send response to change_state (timeout)"）→ 节点停在 inactive，
# /cmd_vel_auto_forward_filtered 一帧都不发 → 选择器判 no-command →
# /cmd_vel_auto_safe 恒 0 → 面板显示"已开始"但车一动不动。
_lc=$(timeout 10 ros2 lifecycle get /collision_monitor_forward 2>/dev/null | head -1)
case "$_lc" in
  active*) say "collision_monitor_forward" "✅ $_lc";;
  *)       say "collision_monitor_forward" "❌ ${_lc:-查询超时}（应为 active）"; bad;;
esac
# 有发布者还不够，必须真的在发数据
if timeout 8 ros2 topic echo --once /cmd_vel_auto_forward_filtered >/dev/null 2>&1; then
  say "/cmd_vel_auto_forward_filtered" "✅ 有输出"
else
  say "/cmd_vel_auto_forward_filtered" "❌ 无输出（车不会动）"
  say "" "修复: ros2 lifecycle set /collision_monitor_forward deactivate && sleep 2 && ros2 lifecycle set /collision_monitor_forward activate"
  say "" "或常驻看门狗: .venv/bin/python tools/chain_watchdog.py"
  bad
fi

echo "=== ④ 冻结参数是否被意外改动 ==="
chk() {
  # 参数查询偶尔会超时（DDS 首次发现慢），重试一次再判定，避免误报
  v=$(timeout 10 ros2 param get "$1" "$2" 2>/dev/null | tail -1)
  if [ -z "$v" ]; then v=$(timeout 10 ros2 param get "$1" "$2" 2>/dev/null | tail -1); fi
  if [ -z "$v" ]; then say "$2" "⚠️ 查询超时（未知，非失败）"; return; fi
  case "$v" in
    *"$3"*) say "$2" "✅ $v";;
    *)      say "$2" "❌ 期望 $3，实际 $v"; bad;;
  esac
}
chk /controller_server FollowPath.lookahead_dist 0.55
chk /controller_server FollowPath.rotate_to_heading_angular_vel 0.55
chk /controller_server FollowPath.use_cost_regulated_linear_velocity_scaling False
chk /base_bridge wheel_speed_scale 0.66

echo "=== ⑤ 传感器 ==="
hz=$(timeout 6 ros2 topic hz /scan --window 10 2>/dev/null | grep -oE 'average rate: [0-9.]+' | head -1)
[ -n "$hz" ] && say "/scan" "✅ $hz" || { say "/scan" "❌ 无数据"; bad; }

echo "=== ⑥ 崩溃记录 ==="
if [ -n "$LOG" ] && [ -f "$LOG" ]; then
  deaths=$(grep -ac 'process has died' "$LOG" 2>/dev/null || true)
  deaths=${deaths:-0}
  [ "$deaths" -eq 0 ] && say "launch 日志" "✅ 0 次崩溃" || { say "launch 日志" "❌ $deaths 次崩溃"; grep -a 'process has died' "$LOG" | tail -3; bad; }
else
  say "launch 日志" "（未提供，跳过）"
fi

echo
if [ "$fail" -eq 0 ]; then echo "✅ 全部通过，可以开导航"; exit 0
else echo "❌ 有 $fail 项异常 —— 先修再开"; exit 1; fi
