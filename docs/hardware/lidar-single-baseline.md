# 单雷达基线（v2.0 冻结，2026-11-18）

> **本文件是"双雷达改造"的回退基准。** 双雷达分支出问题时，按这里的参数可以
> 完整恢复单雷达方案。所有值均从当前代码实测提取，不是文档抄录。

---

## 1. 硬件

| 项 | 值 |
| --- | --- |
| 型号 | **RPLIDAR C1**（DTOF，0.05–12 m，5000 Hz，8–12 Hz，0.72°，±30 mm，IP54）|
| 数量 | **1 个**（装在车头）|
| 接口 | USB 转 TTL 串口（CP2102N）|
| 波特率 | **460800** |
| 串口 | `/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_e00aaf475431f111b7818c5f30d20014-if00-port0` |
| 扫描模式 | `Standard` |
| 安装姿态 | **倒装 180°**（`rpy="0 0 3.14159"`）→ **scan 角 0 = 车尾** |

---

## 2. TF（`fwr_chassis_demo.urdf:60-65`）

```
base_footprint → base_link   : xyz = 0 0 0.0325
base_link → laser_front      : xyz = 0.21 0 0.0425   rpy = 0 0 π
```

| 派生量 | 值 |
| --- | --- |
| **扫描面绝对高度** | 0.0325 + 0.0425 = **0.075 m（75 mm）** |
| 雷达光心相对车体 | 前 **+0.21 m**，车体中心 y=0 |

> ⚠️ 这个 75 mm 是**实车实测**（URDF 注释："measured scan z=75 mm"），
> 不再是旧文档里的 230 mm。车体已改造过，雷达已降低。
> **后果：0~75 mm 的障碍看不见**（这是本次要加双雷达/低位感知的动因）。

---

## 3. 遮罩（260° 有效视场）

| 项 | 值 |
| --- | --- |
| 遮罩角度 | **后方 ±50°**（`math.radians(50.0)`）|
| 有效视场 | **260°** |
| 实现 | `scan_mask_core.mask_rear_ranges()` |
| 遮蔽值 | **`+inf`（不是 NaN）** |

**launch 里显式传参**（改代码默认值无效）：

```python
# real_mapping.launch.py:52
'rear_half_angle_rad': math.radians(50.0)
# real_mapping_rtabmap.launch.py:48 同样
```

> 历史坑：节点默认值是 55°，但 launch 写死过 π/2，导致长期只有 180° 视场。
> **必须改 launch，不是改节点默认值。**

**⚠️ 遮蔽值必须是 `+inf`，不能是 `NaN`**（`scan_mask_core.py:26-29`）：
NaN 会让 Cartographer 的 `RangeDataInserter` 产出垃圾位姿
（仿真里静止时 map→odom 以 m/s 级漂移）。ROS 里 `+inf` = "无回波"，Cartographer 明确跳过。

---

## 4. 话题链

```
sllidar_node ──/scan──→ scan_mapping_filter ──/scan_mapping──→ Cartographer
                              │                              └→ 代价地图
                              └──/scan_mapping/healthy
```

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/scan` | LaserScan | 原始 360°，含车体自击 |
| **`/scan_mapping`** | LaserScan | 遮罩后 260°，**建图和代价地图都用它** |
| `/scan_mapping/healthy` | Bool | 覆盖率健康检查（自动建图预检用）|

---

## 5. Cartographer（`config/fwr_2d.lua`）

| 参数 | 值 |
| --- | --- |
| `num_laser_scans` | **1** ← 双雷达要改这里 |
| `tracking_frame` | `imu_link`（有 IMU 时必须用 IMU 自身坐标系）|
| `published_frame` | `odom` |
| `odom_frame` | `odom` |
| `provide_odom_frame` | false |
| `use_odometry` | false |
| `TRAJECTORY_BUILDER_2D.min_range` | **0.20** |
| `TRAJECTORY_BUILDER_2D.max_range` | **12.0** |
| `submaps.num_range_data` | 90 |
| 其余 | 见文件 |

---

## 6. 代价地图（`config/real_nav2.yaml`）

**全局和局部两处都要改**（约 257 行、327 行）：

```yaml
obstacle_layer:
  observation_sources: scan
  scan:
    topic: /scan_mapping
    max_obstacle_height: 2.0
    clearing: true
    marking: true
    data_type: LaserScan
    raytrace_max_range: 3.0
    raytrace_min_range: 0.0
    obstacle_max_range: 2.5
    obstacle_min_range: 0.0
    observation_persistence: 0.0
```

| 参数 | 值 |
| --- | --- |
| `inflation_radius` | 0.48 |
| `cost_scaling_factor` | 1.2 |
| `footprint_padding` | 0.02 |
| `footprint` | `[[0.30,0.18],[0.30,-0.18],[-0.30,-0.18],[-0.30,0.18]]` |
| `robot_base_frame` | `base_footprint` |

---

## 7. 车体尺寸（`fwr_chassis_demo.urdf`）

| 项 | 值 |
| --- | --- |
| 底盘外形 | 600 × 360 mm（footprint ±0.30 × ±0.18）|
| 驱动轮 | **65 mm**（URDF 注释；双雷达改造后可能换成 100 mm，见固件配置）|
| 后向超声 | 4 只 A22（`/ultrasonic_rear/*`，0.14 m 高）|

> ⚠️ `fwr_chassis_demo.urdf` 里的轮径（65 mm）与固件 `board_config.h`
> 里的新轮径（100 mm）**不一致**，是已知遗留。换轮后 URDF 需同步。

---

## 8. 已知局限（双雷达要解决的）

| # | 局限 | 后果 |
| --- | --- | --- |
| 1 | **扫描面 75 mm** | 0~75 mm 障碍完全看不见（脚、线缆、门槛、矮箱）|
| 2 | **只有 260°** | 后方 100° 靠 4 只超声补，超声是 7 Hz 锥形测距，**不能规划绕行** |
| 3 | 后方不可见 | `allow_reversing: false` 长期关闭，无法自动倒车 |
| 4 | 单侧遮挡 | 车体中央设备块挡住后方，无法 360° 建图 |

---

## 9. 回滚方法

```bash
# 恢复本快照
git checkout main -- ros2_ws/src/fwr_bringup/config/fwr_2d.lua \
                     ros2_ws/src/fwr_bringup/config/real_nav2.yaml \
                     ros2_ws/src/fwr_bringup/launch/real_mapping.launch.py \
                     ros2_ws/src/fwr_bringup/fwr_bringup/scan_mask_core.py \
                     ros2_ws/src/fwr_bringup/fwr_bringup/scan_mapping_filter.py

# 或整体切回标签
git checkout lidar-single-baseline
```

---

## 10. 验证命令（改动后必须重跑）

```bash
# 1) 雷达正常
ros2 topic hz /scan                       # 应 ≈10 Hz
# 2) 遮罩生效：260°
ros2 topic echo /scan_mapping --once | head -5
# 3) 单测（含遮罩、健康检查护栏）
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
PYTHONPATH="ros2_ws/src/fwr_bringup:$PYTHONPATH" .venv/bin/python -m pytest \
  ros2_ws/src/fwr_bringup/test -q
```

**基线测试结果（2026-11-18 实测）**：`348 passed, 1 warning in 1.44 s`

> 双雷达改动后必须保持 **348 passed 不减少**；新增测试可以加，但现有的不能挂。
