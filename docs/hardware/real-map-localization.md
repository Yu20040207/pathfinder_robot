# 已保存地图定位（实机）

`real_localization.launch.py` 加载 `real_front_side_map_2026-10-23.yaml`，用 AMCL 对 `/scan_mapping` 定位。编码器与 IMU 融合发布 `odom→base_footprint`，AMCL 独占发布 `map→odom`。底盘桥接默认 `motion_enabled=false`；此入口不启动 Nav2 行驶控制。

先退出运行中的 `real_mapping.launch.py` 或 `real_navigation.launch.py`，再启动定位。不要同时运行 SLAM Toolbox 与 AMCL，否则它们会同时发布 `map→odom`。

```bash
cd /home/manbaout/Company_Project/Four-wheel-robot
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch fwr_bringup real_localization.launch.py
```

在 RViz 将 Fixed Frame 设为 `map`，用 **2D Pose Estimate** 在已保存地图中标出车身中心和车头方向。每次重新启动定位或挪动车身后都要确认初始位姿；不能只凭一张局部地图假设自动全局定位。

只读检查：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 param get /base_bridge motion_enabled
```

验收时保持电机运动门关闭，轻推或手转底盘，观察 RViz 中雷达点与墙体轮廓是否持续重合、`/amcl_pose` 是否随运动更新且无明显跳变。地图目前只覆盖一小段实测区域，进入未建图或轮廓重复的区域前不要开放自主导航。

## 静止时车身朝向漂移的排查

本车的 `/imu/data_raw` 不提供绝对朝向（`orientation_covariance[0] = -1`）。如果 EKF 只融合轮速与陀螺仪角速度，陀螺仪的小偏置会逐渐积累到 `odom→base_footprint`；AMCL 静止时又可能因运动阈值而不更新，RViz 中的车身便会偏离墙线。`real_ekf.yaml` 因此还融合 `/wheel/odom` 的 yaw；轮式里程计在静止时应保持固定。不要把原始 IMU 消息中的恒等四元数当成朝向测量。

复核时先保持 `motion_enabled=false`，比较 `/wheel/odom`、`/odom` 的 yaw，检查 `/scan_mapping` 与保存地图的墙线。全图重定位可调用 `/reinitialize_global_localization`，但在后方被车体遮挡、地图区域有限时可能收敛到错误位置；只有多帧扫描和实物墙面方向都吻合后，才允许导航运动。
# 单次定点目标（2026-10-23）

`/explore/start_fixed_goal` 接收地图坐标 `x/y/yaw`（米、弧度），由
`frontier_explorer` 独占发送 Nav2 目标。实车默认最多距当前位置 0.5 m，
且目标整车外廓必须在已知空白地图内；未知或占据区域会拒绝。
目标到达、失败、取消、传感器超时或底盘故障时，状态回到 `stopped`。
`/explore/stop` 可取消当前目标。开始前还需显式使能底盘运动门并调用
`/base/arm_auto`；这两步只能在有人看护的实车试验中进行。

隔离 Gazebo 已验证一次无障碍单目标到达与停机。动态障碍重新绕行尚未
验收：障碍进入正前方过近时，Collision Monitor 停车且 Nav2 终止目标。
在重新规划验收前，不把定点导航用于无人看护的实车自动行驶。
