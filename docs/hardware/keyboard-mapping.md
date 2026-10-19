# 实机键盘建图（人工看护）

键盘节点发布 `/cmd_vel_manual`，与 Nav2 的 `/cmd_vel` 分开。机器人运动需要底盘桥接同时开启 `motion_enabled` 和 `manual_enabled`。两项默认都是 `false`；导航动作正在执行时，人工遥控指令会被拒绝。键盘节点退出、按键停止重复、雷达或 IMU 超时、ESP32 通信故障时，底盘停止发运动指令。ESP32 本身还有 300 ms 通信超时。

先确认小车周围空旷、配重和 IMU 固定，操作者可立即切断 12 V；保留 12 V 接通。仅在已完成悬空和地面短测后执行以下步骤。若独立 IMU 节点还在运行，用 `start_imu:=false` 防止重复占用串口。停止旧的建图/EKF/底盘桥接进程后，在第一个终端启动：

```bash
source /opt/ros/humble/setup.bash
source /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/install/setup.bash
ros2 launch fwr_bringup real_mapping.launch.py start_imu:=false
```

第二个终端确认 `/scan`、`/imu/data`、`/odom` 持续更新，`/base/fault` 为 0，再显式开启人工模式：

```bash
source /opt/ros/humble/setup.bash
source /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/install/setup.bash
ros2 param set /base_bridge manual_enabled true
ros2 param set /base_bridge motion_enabled true
ros2 topic echo /base/ready --once
```

`/base/ready` 必须是 `true`。第三个终端运行 `ros2 run fwr_bringup keyboard_teleop`，**焦点保持在该终端**：W/S 为前进/后退，A/D 为原地左/右转，空格停机，Q 退出。速度固定为直行 0.06 m/s、转向 0.45 rad/s；不能由键盘提高。按键一次只保持约 0.18 s，持续运动依赖键盘重复按键，松键后自动停机。键盘自动重复的起始间隔可能造成短暂停顿，先用短按熟悉响应，再缓慢沿场地边缘采集地图。倒车和原地转向属于人工近距离操作，须目视检查雷达后方盲区；Nav2 自动倒车仍禁止。

结束后先按空格并按 Q 退出，再关运动门：

```bash
ros2 param set /base_bridge motion_enabled false
ros2 param set /base_bridge manual_enabled false
```

地图采集完成并经 RViz 检查后，使用 `ros2 run nav2_map_server map_saver_cli -f /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/src/fwr_bringup/maps/real_first_map` 保存。保存地图不表示已经通过 Nav2 自动导航验收；仍须检查地图质量、真实轮廓和雷达盲区。
