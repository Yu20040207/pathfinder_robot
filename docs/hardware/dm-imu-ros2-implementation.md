# DM-IMU-L1 ROS 2 接入与实机测试

目标：用当前 Ubuntu 主机通过 USB 读取 DM-IMU-L1，并发布可供后续轮式里程计融合使用的 `/imu/data_raw`。

设计：独立的 `fwr_imu` Python ROS 2 包将协议解析与 ROS 节点分离；解析器只接受帧头、寄存器号、帧尾和 CRC16/CCITT-FALSE 均有效的帧。节点使用稳定的 `/dev/serial/by-id/` 路径和 921600 8N1，收齐加速度与角速度后以可配置速率发布 `sensor_msgs/Imu`，坐标系暂为 `imu_link`。解析器也可解出设备四元数，但尚未核实安装轴向及参考坐标系，消息通过 `orientation_covariance[0] = -1` 明确表示姿态不可用。加速度和角速度的协方差仍为未知；在实测噪声并完成轴向校准前，不应直接接入 EKF。节点不发布 `odom`。

步骤：

1. 用实机抓到的帧编写解析器测试，先确认测试因缺少实现而失败，再实现 CRC、重同步和浮点解码。
2. 新增 ROS 2 节点和包元数据；对四元数顺序、单位与协方差做显式处理。
3. 在本机编译，运行测试，再连接实机读取 `/imu/data_raw`，检查频率、静止读数与设备断开行为。
4. 更新硬件文档中的启动命令和下一阶段的安装姿态标定条件。

## 本机运行

```bash
cd /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fwr_imu --symlink-install
source install/setup.bash
ros2 run fwr_imu dm_imu_node
```

另开终端并加载相同 ROS 环境后，可用 `ros2 topic hz /imu/data_raw` 和 `ros2 topic echo /imu/data_raw --once` 检查。设备路径默认为 `/dev/serial/by-id/usb-DM-Tech_DM-IMU-L1_DMIMU20250212-if00`。改变设备或发布频率可用 `--ros-args -p device:=... -p publish_rate_hz:=100.0`。

## 已验证与下一步

2026-10-13，本机实机发布约 100 Hz；消息包含随传感器倾角变化的加速度和接近零的静止角速度。USB CDC-ACM 的非阻塞读取在暂无数据时返回空字节，节点现将其视为暂时无数据。下一阶段应将 IMU 固定在底盘，确认 X/Y/Z 正方向和转弯时角速度符号，记录静止噪声以设置协方差，再与真实轮编码器里程计融合。热拔插后的自动重连仍需实机验收。

同日再次读取 5 秒原始串口数据，每种数据各 4990 帧，协议错误 0 帧。加速度 XYZ 均值为 `(1.608, 2.585, 9.293) m/s²`，标准差为 `(0.100, 0.068, 0.099) m/s²`；角速度 XYZ 均值为 `(0.00196, 0.00272, -0.00006) rad/s`，标准差为 `(0.0384, 0.0170, 0.0122) rad/s`。这组数据只说明当前摆放状态和链路质量；传感器尚未确认固定在底盘，不能据此设定最终轴变换或融合协方差。

## 2026-10-17 底盘安装与姿态诊断

用户已将 IMU 平放固定在底盘，实物标注 +X 朝车头，+Y 朝车体左侧，+Z 朝上。USB 稳定路径 `/dev/serial/by-id/usb-DM-Tech_DM-IMU-L1_DMIMU20250212-if00` 已重新识别。静止 3 秒数据约为加速度 `(-0.047, +0.044, +9.769) m/s²`、角速度接近零；用户手动逆时针左转时 Z 轴角速度为正，设备四元数随后显示相对航向约 +29°。因此当前轴向与 ROS 车体 +X 前、+Y 左、+Z 上一致。

节点继续发布 `/imu/data_raw`（无姿态），并新增 `/imu/data`（设备六轴四元数）及 `/imu/pose`（固定指向车头 +X 的 RViz 箭头）。默认 `frame_id=imu_link`，预览模型中 `imu_link` 固定于 `imu_body`。姿态与角速度协方差仍未知；六轴航向会漂移，**目前不接入 EKF 或导航**。实机节点与 RViz 模型同时启动后，`/imu/data` 约 100 Hz。

用 `ros2 launch fwr_bringup imu_preview.launch.py` 同时启动驱动、模型、预览 TF 和 RViz。`imu_heading_preview` 启动时把相对朝向设为零，仅在该预览中发布 `imu_preview → base_footprint` TF；RViz 固定坐标系为 `imu_preview`，整车模型随 IMU 转动。橙色 `Robot front (IMU +X)` 箭头以 `base_link` 为参考，位于底盘顶部正中心，始终指向车头。旧的 IMU 外观网格已从 URDF 移除；逻辑 `imu_link` 暂与 `base_link` 原点重合，仅用于轴向标识，尚不代表真实安装平移。该 TF 仅供预览，不能与真实轮里程计的 `odom → base_footprint` 同时运行。待量取真实 IMU 安装位置后再更新用于融合的传感器外参。
