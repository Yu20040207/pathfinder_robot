# 仿真交接文档（2026-11-01）

> 目的：把"四房间 Gazebo 仿真 + 我们的自动建图栈"接到可以让另一个会话继续调试的程度。
> 包含：怎么跑、哪里通了、卡在哪、以及实机自动建图退化的完整分析（已定位，未改代码）。

---

## 1. 环境

| 项 | 值 |
| --- | --- |
| ROS | Humble |
| 仿真 | **Gazebo Sim 6.18（Fortress）+ ros_gz 0.244.26** |
| 显示 | `export DISPLAY=:1` |
| **必须** | `LIBGL_ALWAYS_SOFTWARE=1`、`MESA_GL_VERSION_OVERRIDE=3.3`、`GALLIUM_DRIVER=llvmpipe`（否则 EGL 建不出 dri2 screen，GPU 传感器静默无数据） |
| DDS | `export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.config/fwr/fastdds_udp_only.xml` |

---

## 2. 文件清单

```
sim/
├── worlds/four_rooms.sdf          # 2×2 四房间 14×14 m，家具，0.9 m 门洞
├── models/fwr_bot/model.sdf       # 差速底盘 + 4 个后向"超声"(窄 lidar) + IMU
│                                  #   雷达 360° gpu_lidar，**倒装 180°**（scan 角 0 = 车尾）
├── config/bridge.yaml             # ros_gz_bridge 11 条映射
├── config/sim_2d.lua              # = fwr_2d.lua，仅 tracking_frame 改 base_link
├── config/sim_view.rviz           # RViz 视图：Target=base_footprint（跟随机器人）
└── launch/
    ├── sim_bringup.launch.py      # 只建图：gazebo + 机器人 + 桥接 + TF + 260°遮罩 + 超声 + EKF + Cartographer + RViz
    └── sim_auto_mapping.launch.py # 完整：上面全部 + Nav2 + 安全链 + rear_guard + room_explorer
```

新增节点（`ros2_ws/src/fwr_bringup/fwr_bringup/`）：

- `ultrasonic_sim.py` — 4 路窄 lidar（LaserScan）→ `sensor_msgs/Range`，
  发到 `/ultrasonic_rear/{left_out,left_in,right_in,right_out}`，**接口与实车 A22 完全一致**
- `sim_cmd_router.py` — 仿真底盘入口：`/cmd_vel_auto_safe` + `/cmd_vel_manual` → `/cmd_vel_sim`，
  内含**车尾联锁**（后退且 `rear_blocked` → 清零），替代 base_bridge 的驱动部分

---

## 3. 怎么跑

```bash
cd /home/manbaout/Company_Project/Four-wheel-robot
export DISPLAY=:1
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.config/fwr/fastdds_udp_only.xml
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
rm -f /dev/shm/fastrtps_*          # 清掉旧的共享内存，避免重复节点

# 只建图（手动发 /cmd_vel_sim 就能让车走）
ros2 launch sim/launch/sim_bringup.launch.py

# 完整自动建图（Nav2 + 探索 + 安全链）
ros2 launch sim/launch/sim_auto_mapping.launch.py
```

### ⚠️ 清理进程的正确姿势

`ign gazebo` 的**真实进程名是 `ruby ign gazebo server/gui`**，按世界名匹配杀不掉。
残留多个实例会同时往相同 gz 话题发数据，表现为"节点重复/数据错乱/Nav2 三份"。

现成脚本：`/tmp/cleansim.py`（按关键词杀、并排除自身祖先进程）。关键点：
**绝对不要用 `pkill -f <关键词>`** —— 关键词会出现在自己的命令行里，会把自己杀掉。
必须用 Python 按 PID 杀，并排除自身 + 祖先进程。

Nav2 的进程名（`planner_server`/`controller_server`/`bt_navigator` 等）也要一起清，
否则每重启一次就叠一套 Nav2（曾叠到 3 份、103 个节点）。

---

## 4. 已经通了的部分 ✅

| 项 | 验证结果 |
| --- | --- |
| 世界 + 机器人生成 | ✓ |
| `/scan` 原始 360° | ✓ |
| `/scan_mapping`（过 ±50° 遮罩） | ✓ 与实机同一条代码路径 |
| 4 路 `/ultrasonic_rear/*` | ✓ 值域 0.05~3.5 m，与实机接口一致 |
| `/imu/data_raw` | ✓ 100 Hz |
| `/odom`（我们的 EKF） | ✓ 30 Hz，`odom→base` 新鲜（+0.002 s） |
| Cartographer 建图 | ✓ `/map` 正常增长（手动前进 6 s → 空闲区 8 → 35.6 m²） |
| RViz 显示 | ✓ `sim/config/sim_view.rviz`，视图跟随机器人 |
| Nav2 三件套 | ✓ 都能 active（**前提是只有一份实例**） |

---

## 5. 还没通的部分 ❌（下一位从这里接手）

### 5.1 主要卡点：Nav2 规划失败 → 探索器反复失败 → 车不动

跑 `sim_auto_mapping.launch.py` 后的现象：

```
[room_explorer]: 房间内前沿：目标 (x, y) 距 x.x m
[planner_server]: GridBased: failed to create plan, invalid use: Starting point in lethal space!
或
[planner_server]: Robot is out of bounds of the costmap!
[room_explorer]: 目标 ... 连续失败，拉黑 → 短退脱困 → 位移 0.00 m
```

**已经修掉的两个诱因**：

1. **超声在"看自己的车壳"**：传感器原本装在 x=−0.26，而车体长 0.60（±0.30），
   在壳内部 → 4 路读数 0.12~0.19 m → 车尾守卫常报警 + 范围层把机器人脚下标成致命格
   → `Starting point in lethal space`。
   **已改为 x=−0.33**（车体后缘外侧，对应真实的外伸支架），TF 同步改了。
2. **生成点贴着家具**：原本 (-3.5,-3.5) 正好在房间 A 的桌子边 → 右后超声读 0.11 m。
   **已改到 (-2.5,-5.5)**。

修完后 `lethal space` 次数 = 0，但**仍然有 `Robot is out of bounds of the costmap`**：
某一时刻机器人的 map 位姿跑到 (-30.16, 0.04)，而全局代价地图只有约 24 m 宽。
当前实测 `map→odom` = (+0.14, −0.26)、`odom→base` = (0,0)，位姿又是正常的。

**下一个会话建议从这里查**：
- 全局代价地图的尺寸是不是被 `/map_sealed` 的瞬时尺寸带偏了
  （建图早期 map 很小，后来变大，代价地图没有跟着 resize？日志里有
  `StaticLayer: Resizing costmap to 651 X 341` 这种记录）
- `map_seal` 每 0.5 s 重发，代价地图 static 层跟着变 → 规划器每秒看到不同世界
  （这也是实机"路径跳变"的同一个原因）
- 是否需要在 sim 里固定一个较大的全局代价地图尺寸（`width/height` 写死并关掉 resize）

### 5.2 仿真特有的两个适配（已改，记录备查）

1. `tracking_frame` 必须是 IMU 的坐标系名：实车 `imu_link`、仿真 `base_link`。
   因此仿真用 `sim/config/sim_2d.lua`，不改实机配置。
2. `directional_safety_selector` 的 `require_manual_status` 默认 true，需要
   `/base/manual_status`；仿真没有 base_bridge → 一直判 `stale-manual` 输出 0。
   仿真 launch 里已设 `require_manual_status: False`。
3. 仿真不需要授权流程：`sim_auto_mapping.launch.py` 用 `auto_explore_state`
   （固定发 `/explore/state=navigating`）代替 `auto_mapping_supervisor`
   （后者要设置 `/base_bridge` 参数，仿真里不存在 → 永远卡在 "opening gates"）。

---

## 6. 实机自动建图退化的完整分析（已定位，**未改代码**）

用户观察：加超声**之前**自动建图效果不错；加之后出现
①路径离障碍近、频繁跳变、越走越贴 ②规划出转弯但底盘停在原地。

### 6.1 「规划出转弯但底盘停住」← 主因

`real_collision_monitor_reverse.yaml`：

```yaml
cmd_vel_in_topic: /cmd_vel_auto_unchecked
polygons: [rear_visible_stop]
rear_visible_stop:
  points: [-0.30,0.30, -0.45,0.30, -0.45,-0.30, -0.30,-0.30]
  action_type: stop        # ← Nav2 的 stop 会把整帧 Twist 清零，线速度角速度一起清
  max_points: 3
```

而 `directional_safety_core.select()` 对"原地旋转"要求**两个监控都给出非零角速度**：

```python
required = ('forward', 'reverse') if self.direction == 'rotate' else (self.direction,)
...
if self.direction == 'rotate' and any(abs(z) <= epsilon for _, z in outputs):
    return fail('zero-angular-reply')      # → 输出 (0,0)
```

⇒ 车尾 0.30~0.45 m 内有 3 个激光点，**任何原地旋转都被否决**。

依据：[navigation2 issue #3313](https://github.com/ros-navigation/navigation2/issues/3313)
（社区在讨论 stop 区域无法"只禁平移不禁旋转"）、
[nav2_collision_monitor Humble 文档](https://docs.ros.org/en/humble/p/nav2_collision_monitor/index.html)。

### 6.2 「路径越走越贴」← 局部代价地图的假墙

```yaml
range_layer:                       # 局部代价地图第 2 个插件
  topics: 4 路 /ultrasonic_rear/*
  input_sensor_type: INPUT_ULTRASOUND
  clear_on_max_reading: false      # ← 读到 max_range（无目标）时不清理旧标记
```

超声无目标时我们发的是 `range = max_range`，但 `clear_on_max_reading: false`
意味着这些"空"读数**不会擦掉之前打的障碍格** → 车尾后方在代价地图上
逐渐长出一堵永不消失的假墙（再膨胀 0.30 m）→ 规划器只能把路径往前挤。

### 6.3 「路径频繁跳变」

全局代价地图 static 层读 `/map_sealed`，`map_seal` **每 0.5 s** 重发；
建图时 Cartographer 的 `/map` 本身也在变，BT 里 `RateController hz=1.0`
→ 规划器**每秒看到一个不同的世界** → 路径在两种走法之间来回跳。

### 6.4 其它相关改动

- `base_bridge` 开始拒绝倒车：曾把 `allow_reversing` 打开 → RPP 规划倒车段 →
  底盘拒绝 → `Controller patience exceeded` → 目标失败 → 探索器重发 → 死循环。
  已回退为 `allow_reversing: false`。
- `cost_travel_multiplier` 曾被我 8.0 → 4.0（"敢贴障碍"的旋钮），已回退到 8.0。

### 6.5 建议改法（按性价比）

| # | 改动 | 预期 |
| --- | --- | --- |
| 1 | **去掉 `collision_monitor_reverse`**（倒车安全由 `base_bridge` 的车尾超声联锁负责，它只拦后退不拦旋转） | 恢复转向能力 |
| 2 | `range_layer` 改 `clear_on_max_reading: true`，或整层移除 | 消除车尾假墙 |
| 3 | `/map_sealed` 刷新降到 0.2~0.5 Hz，或规划时用快照 | 路径稳定不跳 |
| 4 | RPP `use_velocity_scaled_lookahead_dist: true`，低速前视 0.30~0.45 m | 不切角贴墙 |
| 5 | 保持 `cost_travel_multiplier: 8.0`、`inflation_radius: 0.30` | 路径留余量 |

**1、2 是"回归修复"级别**，改完应能回到加超声之前的表现。

---

## 7. 建议的仿真验证顺序

1. 先只跑 `sim_bringup.launch.py`，确认地图正常增长（手动发 `/cmd_vel_sim`）
2. 关掉 `collision_monitor_reverse`（改 `sim_auto_mapping.launch.py` 里不启动它，
   或改选择器对 rotate 只采纳 forward 回执），再跑 `sim_auto_mapping.launch.py`
3. 看探索器能否跑出房间 A → 进房间 B/C/D
4. 记录：覆盖面积、脱困次数、`patience exceeded` 次数
5. 通过后再改实机配置（第 6.5 节的 1、2 条）

---

## 8. 当前运行状态（2026-11-01 09:10）

```
gazebo: 3 个 ruby 进程（1 个实例，正常）
ros2 launch: 1（sim_auto_mapping）
rviz2: 1
日志: /tmp/fwr_runs/simnav10.log
```

仿真里 `lethal space` 已为 0，但仍有偶发 `Robot is out of bounds of the costmap`，
探索器目标持续失败、车基本不动 —— **这是下一位的起点**。
