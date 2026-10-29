# 实机键盘建图（人工看护）

键盘节点发布 `/cmd_vel_manual`，与 Nav2 的 `/cmd_vel` 分开。机器人运动需要底盘桥接同时开启 `motion_enabled` 和 `manual_enabled`。两项默认都是 `false`；导航动作正在执行时，人工遥控指令会被拒绝。键盘节点退出、按键停止重复、雷达或 IMU 超时、ESP32 通信故障时，底盘停止发运动指令。ESP32 本身还有 300 ms 通信超时。

先确认小车周围空旷、配重和 IMU 固定，操作者可立即切断 12 V；保留 12 V 接通。仅在已完成悬空和地面短测后执行以下步骤。若独立 IMU 节点还在运行，用 `start_imu:=false` 防止重复占用串口。停止旧的建图/EKF/底盘桥接进程后，在第一个终端启动：

```bash
source /opt/ros/humble/setup.bash
source /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/install/setup.bash
ros2 launch fwr_bringup real_mapping.launch.py start_imu:=false
```

查看实时地图时，可在另一个终端运行：

```bash
source /opt/ros/humble/setup.bash
source /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/install/setup.bash
rviz2 -d /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/src/fwr_bringup/config/real_mapping.rviz
```

RViz 的固定坐标系是 `map`，显示实时 `/map`、C1 `/scan` 和车体模型。

第二个终端确认 `/scan`、`/imu/data`、`/odom` 持续更新，`/base/fault` 为 0。随后在**要接收按键的终端**运行：

```bash
bash /home/manbaout/Company_Project/Four-wheel-robot/scripts/start_keyboard_mapping.sh
```

脚本会检查节点和故障码，开启人工模式，并要求 `/base/ready=true`。**焦点保持在该终端**：W/S 为前进/后退，A/D 按住持续原地左/右转，空格停机，Q 退出。直行目标为 0.10 m/s；转向目标为 0.65 rad/s，底盘桥接用 IMU 角速度反馈修正左右轮指令，上限 0.75 rad/s。它控制持续转速，不按固定角度停机；若需要精确转 10°，使用独立的 `TurnRelative` 动作。按键一次只保持约 0.18 s，持续运动依赖键盘重复按键，松键后自动停机。脚本在运行期间把 GNOME 和 X11 的按键重复延迟暂调为 120 ms，退出时恢复原值，避免长按初期短暂停顿。倒车和原地转向属于人工近距离操作，须目视检查雷达后方盲区；Nav2 自动倒车仍禁止。

终端只在操作状态变化时显示前进、后退、左转、右转与停机，不会为每个自动重复按键刷屏。运行中若 `/base/fault` 非零，会显示故障码及原因、发送停机并退出；启动前已有故障时会拒绝开启运动门。结束后先按空格，再按 Q 退出。脚本退出时会关闭软件运动门。

控制窗口运行期间，脚本临时把 GNOME 和 X11 的全桌面按键重复延迟设为 120 ms，间隔设为 30 ms；这会影响同时打开的输入法和其他窗口。输入文字前先按 Q 退出，正常退出或脚本报错时会恢复原设置。2026-10-20 检查到退出后 GNOME 延迟为 500 ms、间隔 30 ms，X11 延迟 500 ms、重复率约 33 次/秒；若退出后仍多输入字符，需再查输入法或键盘事件，不能仅凭现象认定是固件问题。

地图采集完成并经 RViz 检查后，使用 `ros2 run nav2_map_server map_saver_cli -f /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/src/fwr_bringup/maps/real_first_map` 保存。保存地图不表示已经通过 Nav2 自动导航验收；仍须检查地图质量、真实轮廓和雷达盲区。

### 2026-10-24 多房间建图：先确认能否续建

仓库里现有的 `real_first_map.*`、`real_front_side_map_2026-10-23.*` 只有栅格 `.pgm/.yaml`，**没有 SLAM Toolbox 的 `.posegraph`**。只有栅格地图时不能直接“在旧地图上继续长出新房间”；SLAM Toolbox 的 `map_file_name` 需要之前用 `serialize_map` 保存的序列化位姿图。当前最稳妥的做法是**重新跑一次覆盖全屋的建图**；如果确实要保留旧图，只能把新房间单独建图后再做离线地图合并，复杂且容易错位，不建议作为第一步。

这次建图请同时保存两种文件：

```bash
# 终端 A：先退出 real_navigation / real_localization / 旧 SLAM / 手动 frontier_explorer，
# 确认没有重复的 /base_bridge、/sllidar_node、/dm_imu_l1、/slam_toolbox。
source /opt/ros/humble/setup.bash
source /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/install/setup.bash
ros2 launch fwr_bringup real_mapping.launch.py
```

如果独立 IMU 节点仍在运行，改用 `start_imu:=false`；否则用默认 `true`。

```bash
# 终端 B：实时地图
rviz2 -d /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/src/fwr_bringup/config/real_mapping.rviz

# 终端 C：确认 /scan、/imu/data、/odom 持续更新，/base/fault=0 后，在交互式终端运行键盘控制
bash /home/manbaout/Company_Project/Four-wheel-robot/scripts/start_keyboard_mapping.sh
```

W/S 前后、A/D 转向、空格停、Q 退出。把所有房间走一遍并尽量闭环；地图在 RViz 检查无明显重影后，在终端 B/C 保存：

```bash
# 栅格地图（Nav2 用）
ros2 run nav2_map_server map_saver_cli -f /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/src/fwr_bringup/maps/real_all_rooms

# SLAM 位姿图（下次续建用；必须在 SLAM 仍在运行时执行）
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/src/fwr_bringup/maps/real_all_rooms}"
```

`serialize_map` 会生成 `real_all_rooms.posegraph` 和 `real_all_rooms.data`。以后要续建时，需要在启动 SLAM 时设置 `map_file_name` 和 `map_start_pose`（`real_mapping.launch.py` 目前没有暴露这两个参数，可临时写入 `real_slam.yaml`，或停止该 launch 的 SLAM 后手动运行 `async_slam_toolbox_node`）。

房间标号不要直接改 PGM 图片；使用语义标签文件 + RViz 文字标记，见 [房间标号](room-labeling.md)：`maps/rooms.yaml` + `/room_labels`。

### 2026-10-20 实车连续转向诊断

初版 IMU 转向测试时，0.8 s 左右转能动；左转延长至 1.2 s 后失速锁定。同步采样证实 ROS 手动指令约 1 ms 到达桥接节点，问题发生在电机控制：起步后角速度冲高，PWM 由约 0.72 降至 0.45，轮子随之停转；PWM 回升到 0.58 仍无法重新起步。临时加快起步斜坡的方案未通过测试，已经撤回。

初版固件曾在双轮反向的原地转向模式使用 0.60 PWM 固定下限。它避免了短时失速，但也限制了 IMU 反馈的减速能力。以下记录保留该阶段的测试结果；当前方案及验证结果见文末“双层闭环”一节。

只读诊断话题 `/base/motor_debug` 每个数组依次为：ESP 时间戳 µs、左/右实测轮速 mm/s、左/右目标轮速 mm/s、左/右 PWM、IMU yaw 角速度 rad/s、故障码。用于下一轮不改变控制逻辑的同步采样。

### 2026-10-20 后续诊断

同步曲线发现，旧转向控制在起步超速后把 PWM 从约 0.72 降到 0.45，两轮减速；PWM 回升到 0.58 时未能重新起转。随后加入的 0.60 PWM 固定下限虽消除了这次失速，却无法适应万向轮阻力变化；以下是撤回该方案前的历史测试。

延长至 2.5 s、目标 ±0.75 rad/s 的地面测试，左转稳态平均约 0.61 rad/s，右转约 0.79 rad/s。转向时左电机前进方向约 145 mm/s，反转方向约 90 mm/s；右电机两个方向约 85–90 mm/s。尝试把右转时左轮的最低 PWM 降到 0.55 和 0.575 后，右转速度随地面负载变化较大，无法稳定匹配左转，已撤回该方向补偿并恢复统一 0.60 最低 PWM。下一步对比双轮悬空时的正反转编码器速度，区分驱动系统方向差异与地面万向轮/负载影响。在此之前不宣称左右转速已调齐。

双轮悬空对照已完成：同为 0.60 PWM，转向起步后约 0.3 s，左轮反转/正转分别约 191/196 mm/s，右轮正转/反转分别约 205/215 mm/s，正反向差异远小于地面测试。悬空左转持续到约 0.7 s 时，右轮超过 300 mm/s 并触发固件超速故障码 3，输出立即归零；停机后已清除故障，右转仅做约 0.4 s 短测，结束故障码 0。推断地面负载、万向轮转向和轮胎接地状态对左右角速度差异有显著影响，现有固定最低 PWM 无法保证一致角速度。继续实车驾驶前，需要在接地条件下按 IMU 角速度限制输出，同时确保较小 PWM 不会让驱动轮失速；不得用悬空数据直接校准地面转向。

### 2026-10-20 双层闭环转向实测

依据 [WPILib 的陀螺仪角速度闭环说明](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/gyros-software.html)、[ROS 2 Humble 差速控制器的轮速反馈接口](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html) 和[万向轮阻力研究](https://arxiv.org/abs/2110.05604)，原地转向改为：ESP32 根据各轮编码器速度独立调节 PWM，起步脉冲与运行调速分开；电脑端根据 IMU 实测角速度修正持续转向目标。去掉运行中的固定 0.60 PWM 下限，给轮速控制器增加上下调节余量；在 PWM 上下限使用条件积分，防止万向轮受阻时积累过量误差。停止、故障、通信超时、失速和超速保护保留。

主机 C 测试覆盖轮速偏快时降 PWM、偏慢时增 PWM、饱和时防积分累积、模拟万向轮阻力突变和停机归零；ROS Python 测试覆盖 IMU 控制器饱和后恢复。固件已烧录。地面直接指令两组 2.5 s 左右转，1.2–2.5 s 稳定段的 IMU 平均角速度依次为左 0.620、右 0.678 rad/s，以及左 0.610、右 0.658 rad/s，故障码均为 0。经实际键盘启动脚本的伪终端连续按键 2 s，稳定段左转 +0.664、右转 −0.657 rad/s；按空格和 Q 后 PWM 为 0、运动门关闭、故障码 0。该结果说明现有 2.5 kg 配重及当前地面条件下连续转向已可跟随约 0.65 rad/s 的目标，但尚未覆盖不同总重、地面材质或更长时间运行，不可据此开放无人看护的自主导航。

### 2026-10-20 键盘前后换向失速修正

用户在连续前进后按 S，固件报故障码 1。故障前状态显示右轮仍向前转约 155 mm/s；反向 PWM 缓慢升至约 0.55，但右轮反向编码器几乎没有位移。原因是换向等待旧方向减速时，控制器提前将新方向标记为“已起步”，跳过了带载反向起步脉冲。现在换向时先让 PWM 为零，等待旧方向轮速降低，再为新方向执行受限起步脉冲；失速保护仍有效。复现测试先失败后通过，ESP-IDF 重新编译并烧录。地面限时“前进 0.3 s→后退 1.3 s”复测：两轮实测转速转负并持续运动，故障码始终为 0；实际键盘 W、S 各连续约 2 s 的伪终端测试中，两侧稳定段轮速分别约 63.5/66.9 和 −53.0/−53.5 mm/s，故障码为 0。尚需人工长按复核真实键盘重复输入的手感。
