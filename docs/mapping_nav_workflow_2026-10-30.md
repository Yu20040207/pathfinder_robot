# 换轮后的标定 + 建图 + 导航流程（2026-10-30 执行）

> 背景：2026-10-29 实测**驱动轮打滑约 37%**（轮子报 120 mm/s，车体只前进 77 mm/s），
> 这是当天所有"走不动、走一下就停、地图歪"现象的共同根因。
> 换橡胶轮后必须**重新标定圆周**，否则里程计尺度会再次出错。

## 0. 参考依据

| 主题 | 出处 |
|---|---|
| Nav2 首次上机流程（标定→建图→定位→导航的顺序） | https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/ |
| 差速轮里程计标定（轮径/轮距/每圈计数三个参数） | https://skills.rest/skill/wheel-odometry-model （源：github.com/wimblerobotics/ros2-copilot-skills） |
| slam_toolbox 建图/定位模式与参数 | https://docs.ros.org/en/humble/p/slam_toolbox/ |
| slam_toolbox 长走廊崩参数/漂移（上游 issue） | https://github.com/SteveMacenski/slam_toolbox/issues/642 |
| Cartographer 2D 配置（走廊更稳的备选） | https://google-cartographer-ros.readthedocs.io/ |
| RTAB-Map（最强回环，可用雷达 ICP 当里程计） | https://github.com/introlab/rtabmap_ros |
| Nav2 差速车控制器调参经验 | https://github.com/ros-navigation/navigation2/issues/5375 |

## 1. 硬件准备（换轮当天）

1. 换四个**橡胶驱动轮**（同径同宽，避免左右不等）。
2. 擦净轮面与地面；地砖上的细沙会让打滑翻倍。
3. **检查前后两个万向轮**：手拨一下应能自由转向。万向轮卡死会表现成"走不动"。
4. 确认无线缆/护板拖地。

## 2. 标定顺序（必须按此顺序，前一步不准会污染后一步）

### 2.1 轮子周长（最重要，换胎后必做）

```
准备：地面贴胶带量 3.000 m 直线；车头对准起点标记
操作：键盘 w 慢速直行（0.10~0.12 m/s）走完 3 m，停在终点标记
读数：/wheel/odom 累计位移（或用 tools/odom_calibrate.py）
计算：新周长 = 旧周长 × (实际距离 / 里程计读数)
重复：3 次取均值（每次回到起点重来）
```

配置文件：`ros2_ws/src/fwr_bringup/fwr_bringup/base_bridge_core.py`
（`BaseBridgeCore(circumference_m=...)`）。**注意 `/wheel/odom` 是 base_bridge 发布的，
不是 `wheel_odometry_node.py`**，改错文件不会生效。

合格标准：误差 **<1%**（今天换轮前是 0.13%，打滑后变成 40%）。

### 2.2 轮距（track）

```
操作：tools/track_calibrate.py 90 0.35（原地正反各转 90°）
原理：真实轮距 = 当前轮距 × (编码器转角 / 陀螺仪积分转角)
```

合格标准：比值 **0.998~1.002**。残余 1~2% 属转向打滑，属正常物理极限。

### 2.3 打滑率复核（换轮效果的直接证据）

```
键盘 0.12 m/s 直行 3 秒，看车体实际位移 / 期望位移
换轮前：63%（打滑 37%）
换轮后目标：>90%
```

若仍 <80%，先查：胎压/轮面清洁、万向轮阻力、左右轮径是否一致。

### 2.4 IMU / EKF 检查

```
静止 60 秒测 IMU 航向漂移（应 ≈ 0.00°/min）
原地转 360°，比较 IMU 绝对航向与陀螺仪积分（应相差 <2%）
```

EKF 配置原则（`config/real_ekf.yaml`）：

- 平移取轮式 `vx`；
- 航向**只取 IMU 绝对姿态 + 角速度**；
- **轮距未标定前不要融合轮式 vyaw**（会和 IMU 抢权重，导致转弯航向滞后、地图重影）。

## 3. 建图：用哪个包

| 方案 | 适合 | 本项目的结论 |
|---|---|---|
| **slam_toolbox**（首选） | Nav2 官方推荐、维护活跃、配置简单 | 用它，但**必须**遵守崩溃安全参数（见下） |
| Cartographer | 长走廊/大场景更稳、有子图+位姿图 | 备选 1（已 apt 装好，`config/fwr_2d.lua` + `real_mapping_cartographer.launch.py` 已就绪） |
| RTAB-Map | 回环闭合最强，可用雷达 ICP 绕过里程计误差 | 备选 2（已 apt 装好，`real_mapping_rtabmap.launch.py` 已就绪） |

### slam_toolbox 硬性参数（踩过坑，勿改）

```yaml
minimum_travel_distance: 0.5      # <0.1 会崩：Mapper FATAL ERROR（issue #642）
minimum_travel_heading: 0.5
scan_buffer_size: 10
stack_size_to_use: 40000000
map_update_interval: 2.0
loop_search_maximum_distance: 8.0 # 走廊靠回环纠偏
minimum_angle_penalty: 0.8        # 别调太低，太松会让匹配在走廊里乱转
```

### 建图操作要点

1. 从**开阔处**起步，先直行 1 m 让地图长出来（slam 只按平移加节点，原地转不建图）。
2. 走廊里：慢（0.10~0.12）、少原地转、**过门洞时稍微转头**（给匹配一个纵向锚点）。
3. **结尾必须回到起点闭环**，回环闭合是纠偏的唯一手段。
4. 存档：
   - 位姿图：`ros2 service call /slam_toolbox/serialize_map ...`（**只在停车时调用**，
     高频序列化会让 slam 崩溃）
   - 栅格图：`ros2 run nav2_map_server map_saver_cli -f <name>`
5. 保存后把 `.pgm/.yaml` 放进 `ros2_ws/src/fwr_bringup/maps/` 并 `colcon build`（建立安装软链）。

## 4. 定位 + 导航

1. `ros2 launch fwr_bringup real_navigation.launch.py use_saved_map:=true localizer:=amcl map:=<新图>`
2. 播种：`python tools/seed_amcl.py <map.yaml> --publish`（全局匹配残差应 <0.10 m）
   或 RViz 工具栏 `2D Pose Estimate` 手动点。
3. 确认：`map→base_footprint` TF 存在、Nav2 四件套 active、`/base/ready=True`。
4. 先在开阔处点 2~3 m 的近目标验证，再点远的。

### 今天已修好、不要回退的参数

| 参数 | 值 | 原因 |
|---|---|---|
| 全局 `inflation_radius` | 0.30 | 0.45 时贴墙停车会让机器人自身落在致命格，规划直接失败 |
| 局部 `inflation_radius` | 0.20 | 窄处（侧向 0.46 m）能通过；真实障碍由碰撞监控兜底 |
| `front_stop` 半宽 | 0.24 | 刚好略超车体 0.215（测试约束：0.22~0.26），窄门不误判 |
| `front_stop.max_points` | 3 | 原来 1 个点就全停，噪点会让车锁死、连旋转都不许 |
| `behavior_server.simulate_ahead_time` | 0.5 | 2.0 时距墙 0.45 m 就预判碰撞，spin 恢复必然失败 |
| `angular_dist_threshold`（rotation shim） | 0.6 | 34° 就先原地转向，避免窄处直接顶上去 |

## 5. 当天执行清单（打勾用）

- [ ] 换橡胶轮、擦净、检查万向轮
- [ ] 2.1 轮子周长标定（3 次均值，误差 <1%）→ 改 `base_bridge_core.py` → `colcon build`
- [ ] 2.2 轮距标定（比值 0.998~1.002）
- [ ] 2.3 打滑率复核（目标 >90%）
- [ ] 2.4 IMU 静止漂移 + 360° 一致性
- [ ] 起 slam_toolbox 建图（开阔处起步，走廊慢行，回到起点闭环）
- [ ] 存图（停车后 serialize + map_saver），放进 maps/ 并 colcon build
- [ ] 起导航栈 + AMCL 播种（残差 <0.10 m）
- [ ] 近目标验证 → 远目标验证
- [ ] 把结果写回 `docs/AGENT_LOG.md`

## 6. 已知坑（今天全踩过，别再踩）

| 坑 | 现象 | 规避 |
|---|---|---|
| `minimum_travel_distance` 调小 | slam_toolbox 崩溃 `Mapper FATAL ERROR` | 保持 0.5 |
| 高频 `serialize_map` | slam 崩溃 | 只在停车时保存 |
| 在要保留的地图上做原地旋转标定 | 地图被转歪 | 标定在建图之前做 |
| 雷达重复进程 | `/scan` 无数据、TF 全崩 | 清理要包含 `sllidar` 路径 |
| ESP32 USB 掉线 | `/wheel/odom` 无数据、ready=False | `lsusb \| grep -i espressif` |
| 探索状态短暂 `stopped` | 底盘注销授权、车不动 | 已修：导航目标 active 时忽略 |
| 贴墙停车 | 规划立即失败 | 膨胀 0.30/0.20，或先把车开到开阔处 |
| 塑料轮打滑 | 走不动、地图拉伸 | 换橡胶轮 + 标定（本文件核心） |
