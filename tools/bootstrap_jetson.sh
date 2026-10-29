#!/usr/bin/env bash
# bootstrap_jetson.sh — 一键初始化 Jetson 上的 ROS 2 Humble 工作区
#
# 用法（在 Jetson 上执行）：
#   bash tools/bootstrap_jetson.sh                 # 交互式，逐步确认
#   bash tools/bootstrap_jetson.sh --yes           # 无人值守
#   bash tools/bootstrap_jetson.sh --skip-build    # 只装环境，不编译
#   bash tools/bootstrap_jetson.sh --branch main   # 指定分支
#   bash tools/bootstrap_jetson.sh --dir ~/robot   # 指定工作区目录
#
# 脚本是幂等的：重复执行不会重复安装或重复追加 .bashrc。
set -euo pipefail

REPO_URL="${REPO_URL:-https://github.com/Yu20040207/pathfinder_robot.git}"
BRANCH="codex/real-nav2-imu-turn"
WORKDIR="${HOME}/Company_Project/Four-wheel-robot"
ASSUME_YES=0
SKIP_BUILD=0
SKIP_APT_UPGRADE=0
JOBS=""

# ---------- 参数 ----------
while [ $# -gt 0 ]; do
  case "$1" in
    --yes|-y)          ASSUME_YES=1 ;;
    --skip-build)      SKIP_BUILD=1 ;;
    --skip-upgrade)    SKIP_APT_UPGRADE=1 ;;
    --branch)          BRANCH="${2:?--branch 需要参数}"; shift ;;
    --dir)             WORKDIR="${2:?--dir 需要参数}"; shift ;;
    --repo)            REPO_URL="${2:?--repo 需要参数}"; shift ;;
    --jobs)            JOBS="${2:?--jobs 需要参数}"; shift ;;
    -h|--help)         sed -n '2,18p' "$0"; exit 0 ;;
    *) echo "未知参数: $1（--help 查看用法）" >&2; exit 2 ;;
  esac
  shift
done

# ---------- 输出 ----------
if [ -t 1 ]; then
  C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_HDR=$'\033[36m'; C_OFF=$'\033[0m'
else
  C_OK=""; C_WARN=""; C_ERR=""; C_HDR=""; C_OFF=""
fi
STEP=0
step()  { STEP=$((STEP+1)); printf '\n%s=== [%02d] %s ===%s\n' "$C_HDR" "$STEP" "$*" "$C_OFF"; }
ok()    { printf '%s  ✓ %s%s\n' "$C_OK" "$*" "$C_OFF"; }
warn()  { printf '%s  ! %s%s\n' "$C_WARN" "$*" "$C_OFF"; }
die()   { printf '%s  ✗ %s%s\n' "$C_ERR" "$*" "$C_OFF" >&2; exit 1; }
confirm() {
  [ "$ASSUME_YES" = 1 ] && return 0
  local ans
  read -r -p "  $1 [Y/n] " ans || true
  case "${ans:-y}" in y|Y|yes|YES) return 0 ;; *) return 1 ;; esac
}

# ---------- 环境检查 ----------
step "检查主机环境"
[ "$(uname -m)" = "aarch64" ] || warn "当前架构是 $(uname -m)，本脚本为 Jetson(aarch64) 设计，继续可能失败"
. /etc/os-release
[ "${VERSION_ID:-}" = "22.04" ] || die "需要 Ubuntu 22.04（JetPack 6.x）；当前是 ${PRETTY_NAME:-未知}。
  JetPack 7 是 Ubuntu 24.04 + ROS 2 Jazzy，本工作区不兼容。"
ok "系统: ${PRETTY_NAME}"

if ! sudo -n true 2>/dev/null; then
  echo "  需要 sudo 权限安装系统包，下面可能会要求输入密码。"
  sudo true || die "无法获取 sudo 权限"
fi
ok "sudo 可用"

# ---------- 1. 基础包 ----------
step "安装基础工具与编译依赖"
sudo apt-get update -qq
sudo apt-get install -y -qq \
  git curl wget ca-certificates gnupg lsb-release software-properties-common \
  build-essential cmake ninja-build python3-pip python3-venv \
  htop tmux vim
ok "基础工具就绪"

if [ "$SKIP_APT_UPGRADE" = 0 ] && confirm "是否执行 apt upgrade（首次装机建议做，约 10 分钟）？"; then
  sudo apt-get upgrade -y
  ok "系统已更新"
fi

# ---------- 2. swap ----------
step "配置 swap（Jetson 8GB 编译 ROS 需要）"
if swapon --show | grep -q '/swapfile'; then
  ok "已存在 /swapfile，跳过"
else
  sudo fallocate -l 8G /swapfile
  sudo chmod 600 /swapfile
  sudo mkswap /swapfile >/dev/null
  sudo swapon /swapfile
  grep -q '^/swapfile' /etc/fstab || echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab >/dev/null
  ok "8G swap 已创建"
fi

# ---------- 3. ROS 2 Humble ----------
step "安装 ROS 2 Humble"
if dpkg -s ros-humble-desktop >/dev/null 2>&1; then
  ok "ros-humble-desktop 已安装，跳过"
else
  sudo add-apt-repository universe -y >/dev/null
  [ -f /usr/share/keyrings/ros-archive-keyring.gpg ] || \
    sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt-get update -qq
  sudo apt-get install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions
  ok "ROS 2 Humble 安装完成"
fi

step "安装导航/建图相关功能包"
sudo apt-get install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-collision-monitor ros-humble-nav2-smac-planner \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-cv-bridge ros-humble-pcl-conversions \
  ros-humble-tf2-tools ros-humble-rviz2 \
  ros-humble-cartographer ros-humble-cartographer-ros \
  python3-numpy python3-scipy python3-yaml python3-requests python3-opencv
ok "功能包就绪"

if confirm "是否安装 RTAB-Map（约 400MB，备用建图方案，可不装）？"; then
  sudo apt-get install -y ros-humble-rtabmap-ros
  ok "RTAB-Map 已安装"
fi

# ---------- 4. 工作区 ----------
step "获取代码"
if [ -d "$WORKDIR/.git" ]; then
  ok "仓库已存在: $WORKDIR"
  git -C "$WORKDIR" fetch --quiet origin "$BRANCH" || warn "fetch 失败（网络？），继续用本地代码"
  git -C "$WORKDIR" checkout -q "$BRANCH" 2>/dev/null || warn "切换到 $BRANCH 失败，保持当前分支"
  git -C "$WORKDIR" pull --ff-only --quiet 2>/dev/null || warn "拉取更新失败，继续"
else
  mkdir -p "$(dirname "$WORKDIR")"
  git clone --branch "$BRANCH" "$REPO_URL" "$WORKDIR"
fi
ok "代码位于: $WORKDIR"

step "克隆三个第三方包（不在主仓库里）"
SRC="$WORKDIR/ros2_ws/src"
clone_if_missing() {
  local name="$1" url="$2"
  if [ -d "$SRC/$name/.git" ]; then
    ok "$name 已存在"
  else
    git clone --depth 1 "$url" "$SRC/$name"
    ok "$name 已克隆"
  fi
}
clone_if_missing sllidar_ros2   https://github.com/Slamtec/sllidar_ros2.git
clone_if_missing m-explore-ros2 https://github.com/robo-friends/m-explore-ros2.git
clone_if_missing map_edit       https://github.com/Tony-tpc/map_edit.git

step "应用本地补丁"
apply_patch_if_needed() {
  local dir="$1" patch="$2"
  [ -f "$patch" ] || { warn "补丁不存在: $patch"; return 0; }
  if git -C "$dir" apply --reverse --check "$patch" >/dev/null 2>&1; then
    ok "$(basename "$patch") 已应用"
  elif git -C "$dir" apply --check "$patch" >/dev/null 2>&1; then
    git -C "$dir" apply "$patch"
    ok "$(basename "$patch") 已应用"
  else
    warn "$(basename "$patch") 无法直接应用（上游版本可能变了），请手动检查"
  fi
}
apply_patch_if_needed "$SRC/m-explore-ros2" "$WORKDIR/patches/m-explore-ros2-local-changes.patch"
apply_patch_if_needed "$SRC/map_edit"       "$WORKDIR/patches/map_edit-local-changes.patch"

# ---------- 5. 环境变量 ----------
step "配置 shell 环境与 DDS"
mkdir -p "$HOME/.config/fwr"
if [ ! -f "$HOME/.config/fwr/fastdds_udp_only.xml" ]; then
  cat > "$HOME/.config/fwr/fastdds_udp_only.xml" <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_only</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="udp_only_participant" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_only</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
XML
  ok "DDS 配置已生成（UDP only，避免跨机共享内存问题）"
else
  ok "DDS 配置已存在"
fi

BASHRC_MARK="# >>> fwr robot workspace >>>"
if grep -qF "$BASHRC_MARK" "$HOME/.bashrc" 2>/dev/null; then
  ok ".bashrc 已配置，跳过"
else
  cat >> "$HOME/.bashrc" <<EOF

$BASHRC_MARK
source /opt/ros/humble/setup.bash
[ -f "$WORKDIR/ros2_ws/install/setup.bash" ] && source "$WORKDIR/ros2_ws/install/setup.bash"
export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/.config/fwr/fastdds_udp_only.xml
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}
# <<< fwr robot workspace <<<
EOF
  ok ".bashrc 已追加（新开终端生效）"
fi

if ! id -nG "$USER" | grep -qw dialout; then
  sudo usermod -aG dialout "$USER"
  warn "已加入 dialout 组，需重新登录（或 newgrp dialout）才能访问串口"
else
  ok "用户已在 dialout 组"
fi

# ---------- 6. 编译 ----------
if [ "$SKIP_BUILD" = 1 ]; then
  step "跳过编译（--skip-build）"
else
  step "编译工作区（首次约 20~40 分钟）"
  [ -n "$JOBS" ] || JOBS="$(nproc)"
  warn "并行数 $JOBS；若内存不足出现 Killed，请用 --jobs 2 重跑"
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  cd "$WORKDIR/ros2_ws"
  if command -v rosdep >/dev/null 2>&1; then
    rosdep update >/dev/null 2>&1 || warn "rosdep update 失败（不影响编译）"
    rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || warn "rosdep 安装部分依赖失败，继续编译"
  fi
  colcon build --symlink-install --parallel-workers "$JOBS"
  set -u
  ok "编译完成"
fi

# ---------- 7. 自检 ----------
step "自检"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
[ -f "$WORKDIR/ros2_ws/install/setup.bash" ] && source "$WORKDIR/ros2_ws/install/setup.bash"
set -u

missing=""
for p in fwr_bringup fwr_interfaces sllidar_ros2 slam_toolbox nav2_bringup robot_localization; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 || missing="$missing $p"
done
[ -z "$missing" ] && ok "关键包全部可见" || warn "缺少:$missing"

echo
echo "  当前串口设备（插上硬件后应看到三个）:"
ls /dev/serial/by-id/ 2>/dev/null | sed 's/^/    /' || warn "  没有 /dev/serial/by-id/ 条目"

cat <<EOF

${C_OK}初始化完成${C_OFF}

  工作区   : $WORKDIR
  环境变量 : 新开终端自动生效（或 source ~/.bashrc）

  下一步:
    1) 重新登录一次（让 dialout 分组生效）
    2) 插上 ESP32 / IMU / 雷达，确认 ls /dev/serial/by-id/ 有三个设备
    3) 起最小底盘栈:
         ros2 launch fwr_bringup real_mapping.launch.py start_slam:=false
    4) 另开终端检查:
         ros2 topic hz /scan ; ros2 topic hz /imu/data ; ros2 topic hz /wheel/odom
    5) 详细验收流程见 docs/jetson_migration.md 阶段 6~7

EOF
