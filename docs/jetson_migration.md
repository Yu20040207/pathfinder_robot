# Jetson 迁移操作手册（Orin Nano Super 8GB）

> 目标：把现在 Ubuntu 台式机上的工作区完整搬到达 Jetson，行为与今天一致。
> 关键前提：**JetPack 6.x = Ubuntu 22.04 = ROS 2 Humble**，和当前环境完全一致。
> ⚠️ 不要刷 JetPack 7（Ubuntu 24.04 → ROS 2 Jazzy），Nav2 API 有变化，会白折腾。

---

## 阶段 0 · 出发前（在现在的台式机上做，10 分钟）

```bash
cd /home/manbaout/Company_Project/Four-wheel-robot

# 1) 确认全部已提交并推送（应无输出）
git status --short
git log --oneline -1

# 2) 记录环境快照，出问题时对照
{ echo "== ros2 pkg list =="; ros2 pkg list
  echo "== python =="; python3 --version
  echo "== pip freeze =="; .venv/bin/pip freeze
} > docs/env_snapshot_$(date +%F).txt 2>/dev/null

# 3) 打包两个不在 git 里的东西（如果目标是另一台机器）
#    - ~/.config/fwr/fastdds_udp_only.xml  （DDS 配置）
#    - .venv 不要拷（架构相关），只记 requirements
.venv/bin/pip freeze > /tmp/requirements.txt
cp ~/.config/fwr/fastdds_udp_only.xml /tmp/ 2>/dev/null && echo "DDS 配置已导出"
```

**必须记住的数字**（Jetson 上要复现）：

| 项 | 值 |
|---|---|
| 轮距 `track_m` | 0.268 |
| 周长 `circumference_m` | 0.2012 |
| 计数/圈 | 1468 |
| ESP32 串口 | 115200，`BASE_CONTROL_V2` |
| IMU 串口 | 921600 |

---

## 阶段 0.5 · 一键脚本（推荐先试这个）

仓库里带了 `tools/bootstrap_jetson.sh`，把阶段 1~4 的绝大部分工作自动化：

```bash
# 在 Jetson 上（先确保能联网）
git clone --branch codex/real-nav2-imu-turn \
  https://github.com/Yu20040207/pathfinder_robot.git ~/Company_Project/Four-wheel-robot
cd ~/Company_Project/Four-wheel-robot
bash tools/bootstrap_jetson.sh
```

脚本会依次：检查系统版本 → 装基础工具 → 建 8G swap → 装 ROS 2 Humble 与导航包 →
克隆三个第三方包并打补丁 → 写 DDS 配置与 `.bashrc` → 加 dialout 组 → `colcon build` → 自检。

常用参数：

| 参数 | 作用 |
|---|---|
| `--yes` | 无人值守（不逐项确认） |
| `--skip-build` | 只装环境不编译 |
| `--skip-upgrade` | 跳过 apt upgrade |
| `--jobs 2` | 编译并行数（内存不足时用） |
| `--dir ~/robot` | 指定工作区目录 |
| `--branch <名字>` | 指定分支 |

脚本幂等，可以重复执行；中途失败修好后直接再跑一次即可。
**如果脚本跑完自检全绿，可以直接跳到「阶段 6 · 硬件自检」。**

---

## 阶段 1 · Jetson 基础系统（约 1.5 小时）

### 1.1 刷机

1. 另一台电脑装 **NVIDIA SDK Manager**，选 **JetPack 6.2**（Ubuntu 22.04）
2. Orin Nano 进 Recovery 模式（短接 FC REC + GND 后上电），USB-C 连电脑
3. 刷完首次启动：设置用户名（建议 `manbaout`）、时区 `Asia/Shanghai`、主机名 `jetson`

### 1.2 系统调整

```bash
# 电源模式：最大性能（导航+建图需要）
sudo nvpmodel -m 0          # MAXN
sudo jetson_clocks

# 散热：确认风扇转（Orin Nano Super 有风扇）
sudo apt update && sudo apt upgrade -y

# 常用工具
sudo apt install -y git curl wget htop tmux python3-pip python3-venv \
                    build-essential cmake ninja-build

# 增大 swap（8GB 内存编译 ROS 工作区容易 OOM）
sudo fallocate -l 8G /swapfile && sudo chmod 600 /swapfile
sudo mkswap /swapfile && sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### 1.3 时区与语言

```bash
sudo timedatectl set-timezone Asia/Shanghai
timedatectl                                                   # 确认
```

---

## 阶段 2 · 安装 ROS 2 Humble（约 20 分钟）

```bash
# 1) 源
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# 2) 桌面版 + 编译工具
sudo apt install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions

# 3) 本项目用到的功能包（arm64 都官方支持）
sudo apt install -y \
  ros-humble-nav2-bringup ros-humble-navigation2 \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-collision-monitor ros-humble-nav2-smac-planner \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-ros-gz-bridge ros-humble-cv-bridge ros-humble-pcl-conversions \
  ros-humble-tf2-tools ros-humble-rviz2 \
  ros-humble-cartographer ros-humble-cartographer-ros

# 4) Python 依赖（用 apt 版本，避免 pip 污染系统环境）
sudo apt install -y python3-numpy python3-scipy python3-yaml python3-requests \
                    python3-opencv python3-pip
```

> 检查：`source /opt/ros/humble/setup.bash && ros2 pkg list | wc -l` 应有 300+ 个包。

### RTAB-Map（可选的建图备选，较大）

```bash
sudo apt install -y ros-humble-rtabmap-ros
```

---

## 阶段 3 · 拉取代码（10 分钟）

```bash
mkdir -p ~/Company_Project && cd ~/Company_Project

# 1) 主仓库（SSH 需先把 Jetson 的 ssh 公钥加到 GitHub，或改用 https）
git clone git@github.com:Yu20040207/pathfinder_robot.git Four-wheel-robot
cd Four-wheel-robot

# 2) 三个第三方包（不在主仓库里，被 .gitignore 排除）
cd ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
git clone https://github.com/robo-friends/m-explore-ros2.git
git clone https://github.com/Tony-tpc/map_edit.git

# 3) 打上我们今天改过的补丁（探索器空前沿重试 + 地图编辑）
cd ~/Company_Project/Four-wheel-robot
git apply patches/m-explore-ros2-local-changes.patch
git apply patches/map_edit-local-changes.patch
# 若提示冲突：先 checkout 到补丁对应的提交
#   git -C ros2_ws/src/m-explore-ros2 checkout 9b64d8d
#   git -C ros2_ws/src/map_edit      checkout b1c1f8d
```

---

## 阶段 4 · 编译（20~40 分钟，看散热）

```bash
cd ~/Company_Project/Four-wheel-robot/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --parallel-workers 4
```

**常见问题**：

| 报错 | 处理 |
|---|---|
| `Killed` / OOM | swap 不够，或 `--parallel-workers 2` |
| `sllidar_ros2` 找不到 | 第三方包没 clone 或名字不对 |
| `cv_bridge` 缺失 | `sudo apt install ros-humble-cv-bridge` |
| `rclpy` import 失败 | 不要用 venv，直接系统 python3 |

编译完验证：

```bash
source install/setup.bash
ros2 pkg list | grep -E "fwr_bringup|sllidar|explore|map_edit"
```

---

## 阶段 5 · 环境配置（15 分钟）

### 5.1 .bashrc

```bash
cat >> ~/.bashrc <<'EOF'
# --- FWR 机器人 ---
source /opt/ros/humble/setup.bash
source ~/Company_Project/Four-wheel-robot/ros2_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.config/fwr/fastdds_udp_only.xml
export ROS_DOMAIN_ID=0
EOF
source ~/.bashrc
```

### 5.2 DDS 配置（从台式机拷过来）

```bash
mkdir -p ~/.config/fwr
scp <台式机用户>@<台式机IP>:~/.config/fwr/fastdds_udp_only.xml ~/.config/fwr/
# 或手工创建：只用 UDP，避免 Jetson 上共享内存传输导致的怪问题
```

### 5.3 串口权限与 by-id

```bash
# 把自己加入 dialout（重新登录生效）
sudo usermod -aG dialout $USER

# 插上三个设备后确认
ls /dev/serial/by-id/
# 期望：DM-Tech_DM-IMU-L1_* / Espressif_USB_JTAG_* / Silicon_Labs_CP2102N_*
```

> 代码里全部用 **by-id 路径**，不依赖 ttyACM 编号；by-id 由设备 USB 序列号决定，
> 换主机不会变，所以**不需要改代码**。

---

## 阶段 6 · 硬件自检（10 分钟）

```bash
cd ~/Company_Project/Four-wheel-robot
# 最小底盘栈（不开建图）
ros2 launch fwr_bringup real_mapping.launch.py start_slam:=false
```

另开终端：

```bash
ros2 topic hz /scan        # ≈10 Hz
ros2 topic hz /imu/data    # ≈100 Hz
ros2 topic hz /wheel/odom  # ≈10 Hz
ros2 topic echo /base/fault --once   # data: 0
```

**判据**：三个话题都有数据、fault=0、`map→base_footprint` 可查。

手动验证里程计（和今天一样的方法）：

```bash
ros2 param set /base_bridge motion_enabled true
ros2 param set /base_bridge manual_enabled true
# 手推 1 米，里程计应变化 ≈1.000 m（容差 2%）
```

---

## 阶段 7 · 功能验收（30 分钟）

按 `docs/mapping_nav_runbook.md` 走：

1. 起建图：`ros2 launch fwr_bringup real_mapping.launch.py`
2. 键盘走一小圈，看 RViz 地图是否正常生长
3. 存图 → 起导航 → AMCL 播种 → 点近目标
4. 通过标准：目标 `Goal succeeded`、车实际走到

---

## 阶段 8 · 开机自启（可选，10 分钟）

```bash
sudo tee /etc/systemd/system/fwr-nav.service > /dev/null <<'EOF'
[Unit]
Description=FWR robot navigation stack
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=manbaout
Environment="FASTRTPS_DEFAULT_PROFILES_FILE=/home/manbaout/.config/fwr/fastdds_udp_only.xml"
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && \
  source /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/install/setup.bash && \
  exec ros2 launch fwr_bringup real_navigation.launch.py use_saved_map:=true'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now fwr-nav.service
journalctl -u fwr-nav -f     # 看日志
```

> ⚠️ 自启会**自动打开导航栈**（但 `motion_enabled` 仍默认 false，车不会动）。
> 如果不想开机就占 CPU，先别启用。

---

## 阶段 9 · 性能调优（可选）

```bash
# 确认在 MAXN 模式
sudo nvpmodel -q
sudo jetson_clocks --show

# 看 CPU 占用（导航+建图时应在 200~300% 以内，8 核）
htop
```

| 现象 | 处理 |
|---|---|
| 建图时 map 更新卡顿 | `map_update_interval` 2.0 → 3.0 |
| RViz 卡 | Jetson 上 RViz 用 `--opengl 2`，或改成远程 RViz（台式机看） |
| 风扇吵 | 正常，MAXN 模式风扇常转 |

---

## 附：迁移检查清单（打印出来勾）

- [ ] JetPack 6.2 刷好，Ubuntu 22.04，时区上海
- [ ] MAXN 模式 + jetson_clocks + 8G swap
- [ ] ROS 2 Humble + nav2/slam_toolbox/robot_localization 装好
- [ ] 主仓库 clone（SSH 或 https）
- [ ] 三个第三方包 clone：sllidar_ros2 / m-explore-ros2 / map_edit
- [ ] 两个补丁 git apply 成功
- [ ] `colcon build --symlink-install` 成功
- [ ] `.bashrc` 三行 source/export 生效
- [ ] DDS 配置文件就位
- [ ] dialout 组 + 三个 by-id 设备可见
- [ ] `/scan` `/imu/data` `/wheel/odom` 都有数据，fault=0
- [ ] 手推 1 m → 里程计 ≈1.000 m
- [ ] 建图能出图、能存图
- [ ] 导航点近目标成功
- [ ] （可选）systemd 自启

---

## 附：明确不要做的

| ❌ 不要 | 原因 |
|---|---|
| 拷 `ros2_ws/install/` 到 Jetson | 二进制架构相关（x86→ARM） |
| 拷 `.venv` | 同上，且路径写死 |
| 用 JetPack 7 / Ubuntu 24.04 | ROS 2 变 Jazzy，Nav2 配置要重调 |
| 用 `pip install rclpy` | 会破坏系统 ROS 环境 |
| 直接改 `ttyACM*` 编号 | 插拔会变，坚持用 by-id |
