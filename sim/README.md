# 四房间仿真（Gazebo Fortress + ros_gz）

用**实机同一套算法**跑建图/导航/探索，先在仿真里验证，再上实车。

## 快速开始

```bash
cd ~/Company_Project/Four-wheel-robot
export DISPLAY=:1
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.config/fwr/fastdds_udp_only.xml
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

# ① 只建图（手动发 /cmd_vel_sim 看地图质量）
ros2 launch sim/launch/sim_bringup.launch.py

# ② 自动建图（Nav2 + 房间探索 + 安全链 + 车尾联锁）
ros2 launch sim/launch/sim_auto_mapping.launch.py

# 关掉 RViz
ros2 launch sim/launch/sim_auto_mapping.launch.py start_rviz:=false
```

## 世界

`sim/worlds/four_rooms.sdf`（由 `sim/tools/make_world.py` 生成）：
16×16 m 场地，**中央 2 m 宽走廊**（沿 Y 方向贯通），走廊两侧各两个房间
（左上/左下/右上/右下），**每个房间在走廊一侧开 1.0 m 门**，墙高 5 m。
房间内有家具（桌椅/柜子/沙发/书架/桶/柱子）。机器人从走廊 (0,−6) 朝 +Y 出发。

## 机器人

`sim/models/fwr_bot/robot.urdf` **由实机 URDF 生成**（`fwr_chassis_demo.urdf`），
所以 Gazebo 里看到的和 RViz 里是同一个模型。

- 底盘 0.60×0.43 m，轮距 0.315 m，轮径 0.0325 m（与实车一致）
- 雷达装在车头 `laser_front`，**倒装 180°**（scan 角 0 = 车尾），
  扫描范围 **±130°（原生 260°）**，模拟实车"车身挡 ±45° + 遮罩 ±50°"后的有效视场
- 4 个后向超声在 `x=-0.33`（车体后缘之外），60° 锥角、5 cm~3.5 m
- IMU 在 `imu_link`

重新生成模型（改了 URDF 或传感器后）：

```bash
.venv/bin/python sim/tools/make_model.py          # 见 docs/AGENT_LOG.md 里的脚本内容
ign sdf -p sim/models/fwr_bot/robot.urdf > sim/models/fwr_bot/fwr_bot.sdf
```

## 与实机的对应关系

| 环节 | 仿真 | 实机 |
| --- | --- | --- |
| 雷达 | gz `gpu_lidar`，260° | RPLIDAR C1，遮挡后 260° |
| 建图输入 | **`/scan` 直接喂** | `/scan_mapping`（±50° 遮罩）|
| 超声 | `ultrasonic_sim`（LaserScan→Range）| `ultrasonic_rear`（E08/Modbus）|
| 定位 | `real_ekf.yaml` 同一份 | 同 |
| Nav2 | `real_nav2.yaml` 同一份 | 同 |
| 安全链 | 门控 → 2×碰撞监控 → 方向选择器 | 同 |
| 底盘入口 | `sim_cmd_router`（含车尾联锁）| `base_bridge`（串口）|

**为什么仿真里建图直接吃 `/scan`**：实测给 Cartographer 喂"事后遮罩"的扫描
（NaN 或 inf）会让它位姿乱跳（静止时 map→odom 以 m/s 级漂移）。
仿真里因此让雷达原生 260°，不做遮罩。详见 `docs/AGENT_LOG.md`。

## 必须知道的两个坑

1. **软件渲染**：这台机器 EGL 建不出 dri2 screen，GPU 传感器会静默无数据。
   launch 里已自动设置 `LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3
   GALLIUM_DRIVER=llvmpipe`。
2. **清理进程**：`ign gazebo` 的进程名是 `ruby ign gazebo server/gui`。
   多实例会同时往相同 gz 话题发数据，必须清干净（`/tmp/cleansim.py` 的思路：
   按进程名关键词清理，并排除调用者自身的祖先进程）。

## 观察指标

```bash
# 地图规模与空闲面积
ros2 topic echo /map --once --field info.width
# 探索行为
grep -aE '房间内前沿|房间外前沿|巡逻换角度|短退脱困' <日志> | tail
```
