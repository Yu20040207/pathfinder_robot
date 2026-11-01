#!/usr/bin/env bash
# 只起 RViz 来编辑地图：先停掉导航栈（否则 map_server 会覆盖你的修改），
# 再把 RViz 单独拉起来。此时 /map 只有地图编辑器一个发布者，改动才保得住。
set +u   # ROS 的 setup.bash 会引用未定义变量，不能开 nounset
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export DISPLAY="${DISPLAY:-:1}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fwr/fastdds_udp_only.xml}"

echo "== 1) 停掉导航栈（RViz 也会被关掉，稍后单独起）=="
bash "$REPO/tools/start_mapping.sh" stop || true
sleep 5

echo "== 2) 单独启动 RViz（只有地图编辑器在发 /map）=="
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$REPO/ros2_ws/install/setup.bash"
cd "$REPO"
setsid nohup rviz2 -d "$REPO/ros2_ws/src/fwr_bringup/config/real_localization.rviz" \
  > /tmp/fwr_runs/rviz_edit.log 2>&1 < /dev/null &
disown
sleep 6

cat <<'TIP'

== 现在在 RViz 里操作 ==
  1. Map Edit 面板 →「选择本地地图文件」→ 选
     ros2_ws/src/fwr_bringup/maps/real_map_manual_1101.yaml
     （此时 map_server 已停，新加的「选择当前地图」按钮读不到参数，用这个按钮）
  2. 顶部工具栏选中 Map Eraser Tool
  3. 画/擦改图
  4. 点「保存到yaml文件」保存

== 改完恢复导航 ==
  source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
  ros2 launch fwr_bringup real_navigation.launch.py use_saved_map:=true \
    map:=$PWD/ros2_ws/src/fwr_bringup/maps/real_map_manual_1101.yaml
TIP
