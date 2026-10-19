# 实机 Nav2 与 IMU 定角转向验收记录（2026-10-18）

## 当前状态

**运动门保持关闭。** `real_mapping.launch.py` 的 `base_bridge.motion_enabled=false`；`real_navigation.launch.py` 的 Nav2 生命周期 `autostart=false`。`allow_pivot=false`。开启之前必须重新测量整车外形、验证后方雷达无遮挡，并完成本文件的带载试验；当前没有真实保存地图，不发送自主导航目标。

静态联调已完成：`ros-humble-robot-localization` 3.5.4 可被 ROS Humble 找到；C1、DM-IMU-L1、ESP32-S3 均有稳定串口路径。C1 节点报告健康 OK，标准模式 10 Hz；`/scan` 帧为 `laser_front`。现有 IMU 节点发布 `/imu/data`。新 ESP32-S3 固件烧录成功；停机查询 27 条遥测均无运动、故障码 0。底盘桥接发布 `/wheel/odom`、`/base/fault=0`、`/base/ready=false`。EKF 输出已映射为 `/odom`，实测 `odom→base_footprint` 连续；SLAM 实测发布 `/map` 和 `map→odom`。Nav2 进程能启动，生命周期等待外部激活；controller_server、planner_server、bt_navigator、behavior_server、velocity_smoother 均可手动完成 configure，但未 activate。EKF 在一次负载较高的启动过程中报告更新周期超时一次，需继续观察。测试结束已停止我启动的节点，现有独立 IMU 节点保持原状态。

注意：这些结果证明**静态数据链路**，不证明带载闭环或自主避障可用。当前 URDF 中雷达/IMU 安装坐标和 Nav2 占地轮廓仍是临时值；雷达后方有遮挡。实机参数禁止自动倒车和原地转向，行为树不包含 BackUp/Spin，速度上限初始为 0.06 m/s。雷达或 IMU 数据超过 250 ms 不更新时，底盘桥接会发送停机指令；ESP32-S3 仍有 300 ms 通信超时与故障锁定。

## 启动和单进程原则

现有独立 `dm_imu_node` 正在运行时，使用 `start_imu:=false`；停止它后才使用默认 `start_imu:=true`。一个 ESP 串口只能有一个 `base_bridge`，不得同时运行 `bench.py`、旧 `wheel_odometry` 节点或第二个导航启动。实机导航启动复用实机建图，内部只包含 `nav2_bringup/navigation_launch.py`，避免额外 SLAM/AMCL 抢占 `map→odom`。需先确认地图、TF、雷达、IMU、底盘状态后再由人工激活 Nav2 并单独打开 `motion_enabled`。

```bash
source /opt/ros/humble/setup.bash
source /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/install/setup.bash
ros2 launch fwr_bringup real_navigation.launch.py start_imu:=false
```

若仅人工推车建图，用 `real_mapping.launch.py start_imu:=false`，保持 `motion_enabled=false`。地图保存、雷达方向验证和物理轮廓测量尚未验收。

## 待完成的实车运动门

1. 测量实际总长、总宽、雷达和 IMU 对底盘中心的位姿；校正 URDF 与 Nav2 footprint，尤其检查雷达背后的盲区。
2. 在当前 2.5 kg 配重、场地清空且用户可立即断电时，做有界的 `v` 指令前进、后退、左右转各多次；记录两轮计数、目标/实际速度、PWM、故障、供电与 IMU 转角。此前左转地面测试出现间歇性失速，必须确认已消除。
3. 以人工推动方式取得地图，检查 `map→odom→base_footprint→base_link→laser_front` 单一连续 TF 与地图对齐。
4. 单独验证 ±5° `TurnRelative`、取消与超时、左右两侧均能稳定停机；后方盲区解决前不开放自动原地转向。
5. 低速短路径、近障和雷达掉线停机验收。只有以上通过，才开放实机 Nav2 发车；最终整机重量变化后全部重测。

## 当前 2.5 kg 配重短测

用户确认底盘处于平整空地、配重和 IMU 固定，并可立即断电。`v 60 60` 0.5 秒：左右计数约 -557/+617，fault=0，自动停机；用户确认前进正常。`v -60 -60` 0.5 秒两次均 fault=0 且自动停机；用户观察车身后退略向右。第三次同步 IMU 与电机采集得到后退 0.5 秒时航向变化约 -3.11°（顺时针/向右），与目测一致；CSV 为 `/tmp/fwr-real-nav2-reverse-imu.csv`。这不是长期闭环或直线精度验收。地面 `v` 左右原地短测：0.3 秒对应 IMU 约 +3.68°/−4.16°，0.5 秒对应 +18.02°/−23.63°，四次均 fault=0、自动停机；两方向角度差说明需要 IMU 定角闭环。对应 CSV 为 `/tmp/fwr-real-nav2-{left,right}-turn-{03,05}.csv`。持续运行、满载、雷达盲区和障碍物停机仍待验证。

## 键盘人工控制进度（2026-10-19）

已实现独立 `/cmd_vel_manual` 与 `keyboard_teleop`，保留 Nav2 `/cmd_vel` 的禁止自动倒车/原地转限制。人工通道默认关闭；开启仍需雷达、IMU 和 ESP 状态新鲜且 fault=0。W/S/A/D 固定限速，松键约 0.18 秒发布零速，桥接端 0.2 秒指令超时。76 项 ROS 包测试通过，已构建；独立键盘节点的 ROS 话题实测为 W→0.06 m/s→零速。实机建图启动后，C1 约 10 Hz、`/map` 发布、`/base/fault=0`、`/base/ready=false`，所以尚未带电遥控。持续地面遥控、松键停机、掉线停机以及地图质量仍待验收。操作步骤见 [keyboard-mapping.md](keyboard-mapping.md)。
