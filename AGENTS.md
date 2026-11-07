# AGENTS.md — 四轮机器人二代

本文件是给 AI 编码代理和新同事的“开工说明”，DSH / Codex 等会在会话开始时自动加载。
**这里只放长期有效的规则、入口和当前快照；一次性的排查过程写到 `docs/AGENT_LOG.md`，计划进度写到 `.superpowers/sdd/<plan>/progress.md`。**

## 1. 项目速览

- ROS 2 Humble + Python 3.10（项目虚拟环境 `.venv/`）+ ESP-IDF；开发机 Ubuntu，目标主控 Jetson Orin Nano Super 8GB。
- 主工作区 `ros2_ws/`：`fwr_bringup`、`fwr_voice`、`fwr_navigation`、`fwr_imu`、`fwr_interfaces`、`fwr_vision`、`fwr_attendance`。
- 实机：ESP32-S3 底盘控制、DM-IMU-L1、RPLIDAR C1、HP60C 相机；固件在 `firmware/`。
- 当前分支 `codex/real-nav2-imu-turn`，工作区可能有很多未提交/未跟踪改动。**不要用 `git checkout`、`git reset`、`git clean` 清掉别人的工作。**
- `README.md` 部分段落早于实机进展（例如“尚未安装 Nav2”）；**以 `docs/AGENT_LOG.md` 最新条目为准**。

## 2. 开工顺序（每次任务都做）

1. `git status --short` + `git log --oneline -5`：确认现状，不要假设“上次已经提交”。
2. 读 `docs/AGENT_LOG.md` **最后 2–3 条**（`tail -100 docs/AGENT_LOG.md`），不要整篇读；重点是“已验证/未验证/当前运行状态”。
3. 若任务属于某个计划：读 `.superpowers/sdd/<日期-计划名>/progress.md` 和对应 `docs/superpowers/plans/*.md`。标为 `complete` 的任务不要重做，除非出现新证据。
4. 先 `grep` / `glob` 定位，再按行范围 `read`；不要通读 `build/`、`install/`、`log/`、`.venv/`、`ascam_ros2_ws/third_party/`、`data/`、`models/`。
5. 涉及实机时先查 `ls /dev/serial/by-id/`、`ps -ef | grep -E 'ros2|sllidar|dm_imu|base_bridge'`；不要重复启动同一节点。

## 3. 当前已验证快照（2026-10-24）

- 雷达 C1 使用 **by-id** 路径；`/dev/ttyUSB*` 会因插拔/复位重新枚举，不要把编号写死。
- **雷达视野实测（2026-10-30）**：物理遮挡只有后方 **±45°（共 90°）**，
  即有效视场约 **270°**。扫描角 0° = 车尾，-45°~+45° 读到 0.12~0.15 m 的车身自击，
  `/scan_mapping` 现在屏蔽后方 **±50°**，有效视场 **260°**。
  **注意：这个值必须在 launch 里改**（`real_mapping.launch.py` 的
  `rear_half_angle_rad`），改节点代码默认值无效 —— launch 传参会覆盖它。
  2026-10-31 就是因为 launch 里写死 π/2，长期停在 180° 视场，白扔两侧各 45°。
  由此得到两条硬约束：
  - `/scan` 的后方扇区（scan 角度 0° 附近 = ranges 数组中部）读到的是**车身**，不是障碍；
    `/scan_mapping` 用 `mask_rear_ranges` 屏蔽这一段，建图只用前方+两侧。
  - 任何拿 `/scan` 判断“是否被堵 / 能不能转向”的代码**必须忽略后方扇区**，
    否则会把自己车体当成墙（自动建图曾因此误判“三面被堵”进入死循环）。
  - 雷达倒装 180°：`/scan` 角度范围 [-180°,+180°]，**index 0 侧 = 车头方向**，index 中部 = 车尾。
- **设备路径**（2026-11-03 实测更正，原文档写反了）：
  - ESP32-S3 底盘 = **`/dev/ttyACM0`**（`ID_VENDOR=Espressif`），115200，
    `BASE_CONTROL_V2`，`S2` 为 14 字段
  - IMU DM-IMU-L1 = **`/dev/ttyACM1`**（`ID_VENDOR=DM-Tech`），921600
  - ⚠️ **ACM 编号会随插拔/重启变化**，`base_bridge` 用的是
    `DEFAULT_DEVICE`（by-id：`usb-Espressif_USB_JTAG_serial_debug_unit_...`），
    **不要写死 ACM 编号**。烧录时要用 `idf.py -p <正确的 ACM>`，
    端口被 `base_bridge` 占用时会报 `Invalid head of packet`。
- Nav2 / AMCL / EKF / Collision Monitor 已能运行；`real_navigation.launch.py use_saved_map:=true` 曾连续运行 23 小时以上。
- 已知坑：旧的 `sllidar_node` 可能残留成僵尸发布者，导致 `/scan` 有 2 个 publisher；重启导航栈前先清理。
- `base_bridge` 默认 `motion_enabled=false`、`manual_enabled=false`，不会让车动。
- 实机 RViz 的“两点导航”面板：**单击一次**表示从当前位置前往点击点（`/explore/start_fixed_goal`）；**连续点击两次**表示先到起点再到终点（`/explore/start_two_point_goal`）。现场人员点击“开始导航”时面板自行开启 `motion_enabled`、等待 `/base/ready`、申请 `/base/arm_auto`，然后发送目标；点击“停止导航”或任务结束/失败时请求停机并关闭运动门。无需另开终端运行授权脚本。该按钮流程已构建和通过单元测试，**尚未由现场人员验证完整实车按钮操作**；最新证据见 `docs/AGENT_LOG.md` 末尾。代理不得代替用户点击按钮或打开运动门，除非本次会话另有明确授权。


### 当前已验证参数快照（2026-10-30，换橡胶轮后）

**底盘/固件**（`firmware/esp32s3_base_control/main/control.c`）
- `MAX_DUTY 0.90`、`BASE_DUTY 0.60`、`DUTY_RAMP_PER_S 0.5`
- 失速：`STALL_SPEED_MMPS 4`、门槛 12% 指令、`STALL_LIMIT_S 2.0`
- 超速：`OVERSPEED_MMPS 700`、窗口 0.4 s

**里程计**（`base_bridge_core.py`）
- `track_m 0.262`、`circumference_m 0.2075`、`counts_per_rev 1468`
- 3 米实测误差 +0.20%（换胎后标定）

**IMU 航向修正**（`fwr_imu/node.py` + launch `imu_yaw_scale`）
- `yaw_scale 1.019`（雷达互相关实测：转一圈真值 362.58°，陀螺积分 355.84°）
- **EKF 吃的是 `/imu/data_raw` 的 vyaw**，姿态四元数不进 EKF；
  换 IMU/改安装后必须重标此系数

**AMCL**（`real_localization.yaml`）
- `max_beams 120`、`laser_max_range 8.0`、`min/max_particles 1000/5000`
- `recovery_alpha_slow/fast = 0/0`（**关闭随机粒子注入**，否则车不动位姿乱跳）
- `update_min_d/a 0.10`

**Nav2**（`real_nav2.yaml`）
- 巡航 `desired_linear_vel 0.15`、前视 `lookahead_dist 0.55`（0.35~0.75）
- 旋转 shim `angular_dist_threshold 1.2 rad`（<0.6 会“转过头”）
- `use_collision_detection: false`（RPP 预测误报太多，由 collision_monitor 兜底）
- 膨胀 `inflation_radius 0.30`、`cost_scaling_factor 2.0`
- Smac `cost_travel_multiplier 8.0`（本版本**没有** `cost_penalty` 参数）
- `front_stop` 半宽 0.24 m、3 点触发、`simulate_ahead_time 0.5`

**已知 Nav2 坑（务必知道）**
- `collision_monitor` 触发一次 `stop` 后会**卡死不再处理指令** →
  `/cmd_vel_auto_safe` 恒为 0、车不动。用 `tools/chain_watchdog.py` 自动重启，
  或手工 `ros2 lifecycle set /collision_monitor_forward deactivate/activate`。
- 底盘 `motion_enabled=false` 时所有速度指令被丢弃；
  面板“开始导航”会自己开门，但**手动测试时必须先设 true**。

### 工具环境（2026-10-24）

- 用户级 skill：`~/.agents/skills/ros2-engineering-skills`（Apache-2.0，DSH 已识别）。
- ROS MCP：`~/.local/share/fwr-ros-mcp/venv`（`ros-mcp 3.1.2`）+ 只读包装 `~/.local/bin/ros-mcp-readonly`；DSH 的 `mcp-ros` 行已启用，HMR 热加载 11 个只读工具。
- `rosbridge 2.0.8` 监听 9090；连接地址 `127.0.0.1:9090`，端到端只读验证通过（35 节点、`/scan` 720 点）。
- 启用/关闭/回滚步骤见 `docs/agent/ros-mcp-setup.md`；不要用 `FWR_ROS_MCP_MODE=full` 连实机。

### 实机导航已验证参数快照（2026-11-02，四房间实景）

**已跑通完整两点导航**，以下组合可直接复用：

| 参数 | 值 | 备注 |
| --- | --- | --- |
| `inflation_radius` | **0.48** | 0.45 实测离墙 0.10 m；0.48 更居中 |
| `cost_scaling_factor` | 1.2 | 衰减慢，贴墙更贵 |
| `cost_travel_multiplier` | 8.0 | 调 20 无效果，别动 |
| `desired_linear_vel` | 0.15 | 直线速度 |
| `rotate_to_heading_angular_vel` | 0.6 | 转 90° 约 2.6 s |
| `max_angular_accel` | 4.0 | 转弯不顿挫 |
| `use_regulated_linear_velocity_scaling` | **false** | ⚠️ 开 true 会让车完全不动 |
| `front_stop` 深度 | 0.32~0.38 | 比车体前缘 0.30 留 2~8 cm |
| `dilate_cells`（map_seal） | 1 | 2 以上会把门洞挤窄导致规划失败 |
| `map_publish_period` | **0.0** | ⚠️ 必须显式写，否则 30 Hz 重发冲垮 AMCL |
| `track_unknown_space` | false | true 会让全部路径规划失败 |
| BT `number_of_retries` | 6 | 2 时 0.6 s 就放弃 |
| `auto_initial_pose.max_score_m` | 0.15 | 0.10 会拒绝 0.127 m 的正常匹配 |

**控制 / 行为树（v1.0 定版）**
- `angular_dist_threshold` **0.6**（RotationShim 转向阈值；0.35 时门口
  每个弯都停下原地转，实测 150 s 内 4 次线速度归零 → 门口"特别慢"）
- `use_velocity_scaled_lookahead_dist` **true**
- BT `RateController` **0.5 Hz**（⚠️ 1 Hz 时远处目标要 1040 ms 规划，
  刚算完就被取消 → "点开始就结束"）
- `bt_loop_duration` **50 ms**（100 Hz 会被饿死，报 `tick rate exceeded`）
- `xy_goal_tolerance` **0.4**（规划器会把落在膨胀区的终点吸附 0.34 m）
- 选择器：角速度符号容差 **0.05 rad/s**；方向变化**不清空**回执缓存
  （这两条是"一卡一卡"的两个根因，见 AGENT_LOG）

**速度平滑器（2026-11-03 定版，用户确认"好点了"）**
- `max_velocity = [0.15, 0.0, 1.0]` —— 必须与 controller 的 `desired_linear_vel`
  一致。原来写 0.12，把指令**永久截断**，车从来没跑到过 0.15。
- `max_accel = [0.35, 0.0, 4.0]` —— 上限从 0.12 提到 0.15 后，控制器的
  ±0.02 波动会原样传到电机（直线"一卡一卡"）。把加速度限制降到 0.35
  等于补回低通滤波，速度不变但变化更绵。

> ⚠️ **未解之谜（留给后续）**：`/cmd_vel` 均值 0.142，而 `/odom`
> 均值 0.231（高 63%）、标准差 0.11（是指令的 5 倍）。
> 但另一次 1.5 m 直线测试里 odom 只报 1.264 m（偏低）。
> 两者矛盾，说明**里程计标度可能有问题**（轮径/每圈计数/轮距）。
> 影响：Nav2 的"走了多远"判断不准，goal 容差可能提前触发。
> **建议专门做一次卷尺标定**（直行 2 m，对比 odom 与实测）。

**底盘速度补偿（v1.2 定版）**
- `base_bridge.wheel_speed_scale = 0.66` —— 固件速度环**稳态偏高 1.5 倍**
  （下发 141 mm/s，固件自己也报 214 mm/s，说明它知道但积分下限为 0 无法下修）。
  在 ROS 侧按固定比例补偿；**v 与 ω 同比缩放，转弯半径不变**。
  ⚠️ **固件重新标定后必须把它改回 1.0**。
- ⚠️ **不要单独放开固件积分的负下限**：试过 `clamp(-0.30, 0.30)`，
  速度环收敛了（141→143.7）但**前馈仍偏高**，两者抢权导致极限环振荡
  （实测车速 0↔0.46 m/s、80 s 内 12 次停走）。要改就**同时**把前馈
  `0.22+0.0035v` 降到 ~`0.22+0.0022v`，并做台架测速。

**碰撞监控 / 代价地图（2026-11-02 最终）**
- `front_stop` 深度 **0.32~0.38**（半宽 0.24，3 点触发）—— 浅于 0.30 会撞
- `front_slow` 横向 **±0.22**、深度 **0.30~0.50**、`max_points 3`、`slowdown_ratio 0.8`
  （深度 0.75 时穿过门会扫到"对面门框"→ 持续扣 20% 速度）
  （±0.30 / 1 点时会因侧墙和门框误触发，一次跑动 7 次）
- `local_costmap.plugins = [obstacle_layer, inflation_layer]`
- `footprint_padding 0.02`

> ⚠️ **这四个"手感"参数一次只改一个**：`angular_dist_threshold`、
> `use_velocity_scaled_lookahead_dist`、`inflation_radius`、`front_stop.max_points`。
> 2026-11-02 一起改（0.35→0.7、true→false、0.48→0.50、3→6）后出现
> **起步即原地来回转**；全部回退后立刻恢复正常。改一个、跑一次，再动下一个。

## 3.97 🔒 参数冻结规则（2026-11-04 用户明确要求）

**用户原话：「把这些参数都记下来，不要随便修改，后面改的时候一定要经过我批准」**

### 硬规则（违反即为事故）
1. 下列参数**未经用户在当次会话里明确批准，一律不许改**（包括"顺手优化"）。
2. 需要改时：先说清**改哪个、从多少改到多少、为什么、预期效果、如何验证**，
   等用户确认后再动手。
3. 一次只改一个，改完必须实机验证并把数字写进 `docs/AGENT_LOG.md`。
4. 有护栏测试守着的值（见各节注释）**连提议都要先说明为什么护栏允许**。

### 冻结清单（v1.4 实际运行值）
**导航 `config/real_nav2.yaml`**
| 参数 | 值 |
| --- | --- |
| `FollowPath.desired_linear_vel` | 0.20 |
| `FollowPath.lookahead_dist` / `min` / `max` | **0.55** / 0.40 / 0.85 |
| `FollowPath.angular_dist_threshold` | 0.6 |
| `FollowPath.rotate_to_heading_angular_vel` | **0.55** |
| `FollowPath.max_angular_accel` | 4.0 |
| `FollowPath.use_regulated_linear_velocity_scaling` | false |
| `FollowPath.use_cost_regulated_linear_velocity_scaling` | **false** |
| `inflation_radius`（全局/局部） | 0.48 |
| `cost_scaling_factor` | 1.2 |
| `footprint_padding` | 0.02 |
| `xy_goal_tolerance` | 0.4 |
| velocity_smoother `max_velocity` / `max_accel` | [0.20,0,1.0] / [0.35,0,4.0] |
| BT `RateController` | 0.5 Hz |

**碰撞监控 `config/real_collision_monitor_forward.yaml`**
| 参数 | 值 |
| --- | --- |
| `front_stop.points` | [0.32,0.22, 0.38,0.22, 0.38,-0.22, 0.32,-0.22]（半宽 **0.22**，护栏下限）|
| `front_slow.points` | [0.30,0.22, 0.50,0.22, 0.50,-0.22, 0.30,-0.22] |
| `stop_pub_timeout` | **18.0**（护栏要求 ≥17）|
| `source_timeout` | 0.6 |

**底盘 `base_bridge`**
| 参数 | 值 |
| --- | --- |
| `wheel_speed_scale` | **0.66** |
| `sensor_timeout_s` | 0.5 |

**探索器 `frontier_explorer`（导航侧）**
| 参数 | 值 |
| --- | --- |
| `stuck_cancel_s` | 2.5 |
| `max_stuck_retries` | 2 |

**建图侧（`real_auto_mapping.launch.py` 传给 room_explorer 的）**
| 参数 | 值 |
| --- | --- |
| `escape_after_failures` | 9999 |
| `replan_s` | 1.5 |
| `use_room_gate` | False |

## 3.975 🧹 全局代价地图：障碍层**保留** + 空闲时周期清空（2026-11-07 定版）

> 2026-11-05 曾把全局障碍层**删掉**（治"幽灵障碍"），结果**路上新出现的障碍不会被绕行**：
> 规划器看不到动态障碍，车只会被 `front_stop` 停住干等。**已回退，不要再去掉。**

现在 `global_costmap.plugins = [static_layer, obstacle_layer, inflation_layer]`。

### 污染物怎么治：周期清空
`frontier_explorer` 的 `costmap_clear_period_s`（默认 **10 s**）在两个代价地图上调用
`ClearEntireCostmap`，**只在空闲时执行**（导航中清空会在重规划那一帧把动态障碍一起清掉）。
清空后下一帧扫描立刻重新标记当前真实障碍，只有"车身后方永远扫不到的陈旧标记"被彻底清除。

### 实测（2026-11-05）
- 致命格：真墙 4007 / 墙旁 1 格 1 / **幽灵 1（≈0%）**
- 判断幽灵必须用 **`cost == 100`**；用 `>= 65` 会把**膨胀区**误算成幽灵（踩过两次）。

## 3.976 🧭 多点导航的四个"提前结束"陷阱（2026-11-05~07，都已修）

`frontier_explorer._start_multi_point_goal` 的分支规则（**改这里之前先读完本段**）：

| 陷阱 | 现象 | 现在的处理 |
| --- | --- | --- |
| 终点离**车**近 | goal checker 立刻判到达 | `tail_gap < fixed_route_tail_gap_m`（2.5 m）→ **逐段执行** |
| 终点离**路线起点**近（闭环）| 车去起点路上路过终点 → 判到达 → 后面点全跳过 | `loop_gap < 2.5 m` 同样改**逐段执行** |
| 路线首点就在车身上 | through-poses 零长度首段 → 车原地打转 | `route_skip_near_m`（0.6 m）剔除，**但终点永不剔除** |
| 规划失败（如 `Starting point in lethal space!`）| 行为树重试 6 次后 abort → 整条路线结束 | `_finished` 里清空代价地图并**重派同一段**，最多 `max_stuck_retries`（2）次 |

### 相关参数（v1.7 实际值）
| 参数 | 值 | 说明 |
| --- | --- | --- |
| `fixed_route_tail_gap_m` | 2.5 | 终点距车/距起点小于它 → 逐段执行 |
| `route_skip_near_m` | 0.6 | 剔除车已站住的**非终点**路线点 |
| `costmap_clear_period_s` | 10.0 | 空闲时周期清空代价地图 |
| `reverse_pause_enabled` | True | 后退让位期间暂停任务（不中止） |
| `max_reverse_episodes_per_leg` | 2 | 同一点让位超过它 → 跳过该点 |
| `max_stuck_retries` | 2 | 导航失败后清代价地图重试次数 |

### "停止/继续"的正确语义（2026-11-07）
- 停止只**挂起**任务：`_fixed_goals` / `_multi_points` 和段索引都保留。
- 继续走 `/explore/resume`，**按节点记住的进度**接着走本该走的那一段；
  面板只有在节点说"没有可继续的路线"时才按路线重发。
- 一次穿过（through-poses）模式继续时要**重置派发计数**（`sm.reset_dispatched()`），
  否则状态机认为"已走完"立刻停。

## 3.977 🧍 后退让位（trail_reverser）状态：**默认关闭**

- 默认 `trail_reverser:=false`（`real_navigation.launch.py`），代码保留。
- 触发条件（全部满足）：功能启用 / 车尾不近障 / **左右两侧都 < 0.5 m**（用 `max` 判，
  原来左右混一起取 min 是错的）/ 前方被挡 ≥1 s / 轨迹 ≥0.5 m / 冷却 5 s。
- 后退 0.08 m/s，沿记录轨迹；任意一侧 ≥0.9 m、车尾 <15 cm、1.2 m、10 s 任一满足即结束。
- 与任务的关系：发布 `/reverse/active`；explorer 看到后**暂停**任务，结束再恢复；
  同一点让位 >2 次 → 跳过该点（逐段：`skip_leg`；through-poses：删最近点）。
- ⚠️ 该链路**只有单元测试与独立域模拟**（`tools/sim_reverse_chain_*.py`，需 `ROS_DOMAIN_ID≠0`），
  **尚未实机验证**。

## 3.974 🔧 建图匹配坐标翻转（2026-11-05，严重 bug）

`auto_seed_core.score_pose` 用 `height-1-(y-origin)/res`（图片行序）索引 OccupancyGrid，
把地图**上下镜像** → 自动播种把车放到镜像位置（每次启动落在相距 7~13 m 的点）。

- 证据：同一扫描/地图，不翻转残差 0.120 m @ (-0.10,-0.80)（= 用户手动摆的位置）；
  翻转残差 0.123 m @ (6.20,-9.80)（= 播种结果）。
- 修复后：`match 0.009 m -> (-0.14,-0.91,-75°)`，**贴墙率 98%**（修复前 1~3%）。
- 新增检查：贴墙率 ≥60%、未知区 ≤20%、位置必须在空闲区且有 10 cm 净空；
  任一项不过就**拒绝播种**，等人工 2D Pose Estimate（手动位姿现在不会被覆盖）。
- 回归测试：`test_auto_seed_convention.py` 钉死行序约定。

## 3.96 ⭐ v1.4 定版（2026-11-04，解决"过门慢 / 门口卡住"）

v1.3 之后的改动（都已实车验证）：

### 参数
| 参数 | v1.3 | **v1.4** | 原因 |
| --- | --- | --- | --- |
| `front_stop.points` 半宽 | 0.24 | **0.22** | 0.24 比车体(0.215)宽 2.5 cm，门框落进矩形角点 → 42 次停车 |
| `FollowPath.lookahead_dist` | 0.60 | **0.55** | 前视太远切内弯、贴门框 |
| `FollowPath.rotate_to_heading_angular_vel` | 0.6 | **0.55** | 掉头过冲差点撞门框 |
| `FollowPath.use_cost_regulated_linear_velocity_scaling` | true(默认) | **false** | **过门慢的主因**：车身两侧同时落在门框膨胀区 → 代价饱和 → 降速 |
| `stop_pub_timeout` | 18.0 | **18.0（不变）** | 试过 2.0 被两条护栏测试否决（需 ≥17 s 让 Nav2 进度检查放弃目标） |

### 代码
- `frontier_explorer`：监控持续压零 **2.5 s 且停车框确认为空**（数 /scan 落在框内的点数）
  → 取消当前目标并**自动重试**（`max_stuck_retries=2`）。
  真障碍不会被误判取消；只有 `stop_pub_timeout` 残留才重试。

### 实测（v1.4，一趟导航）
```
front_stop 触发        0 次        （v1.3 前 42 次）
行进速度均值           0.196 m/s   （上限 0.20，门口不再掉速）
侧向净空 10%分位       0.43 m      中位 0.94 m
卡住/重试              0 次
目标                   2 次全部 succeeded
```

### 多点导航（2026-11-04 实车验证通过 ✅）
- 闭环路线（终点离车 <1 m）→ 自动改**逐段执行**，实测 6 段全部走完
  （日志：`终点距车仅 0.35 m（闭环路线）→ 改为逐段执行 6 段`）
- 终点 >1 m 的普通路线 → `NavigateThroughPoses` 一次穿过，途经点不停车
- 运行中卡住 0 次、重试 0 次

### 明天的注意点
- 改这些值之前先看本节：`front_stop` 半宽**不能小于 0.22**（有护栏测试守着，
  是车体 0.215 + 5 mm）；`stop_pub_timeout` **不能小于 17**（两条测试守着）。
- `use_cost_regulated_linear_velocity_scaling=false` 意味着**门口不再自动降速**，
  贴墙/贴门的保护只剩 `front_stop` 和选择器，改速度相关参数时要记得这一点。

## 3.95 ⭐ v1.3 冻结快照（2026-11-03 收工，**用户要求：先不要改动**）

明天开工先读这一节。**除用户明确要求外，下面这些值一个都不要动。**

### 导航（`config/real_nav2.yaml`，实际运行值）
| 参数 | 值 |
| --- | --- |
| `FollowPath.desired_linear_vel` | **0.20** |
| `FollowPath.angular_dist_threshold` | **0.6** |
| `FollowPath.lookahead_dist` / `min` / `max` | **0.60 / 0.40 / 0.85** |
| `use_velocity_scaled_lookahead_dist` | true |
| `inflation_radius`（全局/局部） | **0.48** |
| `footprint_padding` | **0.02** |
| `cost_scaling_factor` | 1.2 |
| `xy_goal_tolerance` | 0.4 |
| velocity_smoother `max_velocity` | **[0.20, 0.0, 1.0]** |
| velocity_smoother `max_accel` | **[0.35, 0.0, 4.0]** |
| BT `RateController` | **0.5 Hz** |

### 底盘
- `base_bridge.wheel_speed_scale` = **0.66**

### 今天修好并冻结的行为
- 安全链：左右扇区不再取同一个值；ω 符号抖动只归零转向、保留前进
- 单点 / 两点 / **多点导航**（多点用 `NavigateThroughPoses`，途经点不停车；
  终点距车 <1 m 的闭环自动改逐段执行）
- 多点预览画出穿过所有途经点的完整折线

### 明天的任务（用户指定）
**解决"靠近门口速度慢"**。已知相关信息：
- `front_slow` 参数（横向 ±0.22、深度 0.30~0.50、`slowdown_ratio 0.8`）
- `approach_velocity_scaling_dist`（默认 0.6，**未显式设置**）
- RPP 的曲率限速（转弯时 0.06~0.15）
- 排查顺序：先看 `/cmd_vel_nav` 在门口的实际数值 vs `/cmd_vel`，确认是哪一层在降速
  （控制器自身 / 平滑器 / collision_monitor 的 slowdown），再动参数。**一次只改一个。**

### ⚠️ 建图调参禁止修改导航共用文件（2026-11-03 血的教训）

`real_nav2.yaml` 和 `real_navigate_to_pose.xml` 是**导航与建图共用**的。
2026-11-03 为了建图效率改了下面这些值，**导航立刻退化**（直线左右摆、门口转不过去）：

| 参数 | 导航（v1.2，正确值） | 建图时曾改成 | 后果 |
| --- | --- | --- | --- |
| `FollowPath.angular_dist_threshold` | **0.6** | 3.1416 | 关掉原地对准 → 门口转不过去 |
| `FollowPath.lookahead_dist` | **0.60** | 0.45 | 前视太短 → 直线左右摆 |
| `FollowPath.min_lookahead_dist` | **0.40** | 0.32 | 同上 |
| `FollowPath.approach_velocity_scaling_dist` | 默认 0.6 | 0.25 | 临近终点减速区变化 |
| BT `RateController` | **0.5 Hz** | 0.25 Hz | 重规划变慢 |

**当前状态**：以上全部已恢复 v1.2（commit `4c5a70c`）。
**规则**：以后建图调参只能在**建图专用**的文件/launch 里做，
或者干脆接受建图慢一点。**不要再动这两个共用文件。**

**自动建图的坑（2026-11-03 全部踩过）**
- **建图必须用 `real_mapping.rviz`**（launch 里已改好）：`real_localization.rviz`
  里的 Map Edit 面板会持续重发旧 `/map`，与 slam 的新地图交替刷屏 → 看着像"地图不长"。
- **`use_room_gate: False`**：房间门控会把走廊当成一个房间，只要它还有前沿就永不
  进别的房间（实测出房间后困在走廊）。关掉用全局前沿。
- **改完 launch 参数必须 `ros2 param get /<node> <param>` 核对**：曾把参数加到
  相邻的 `map_free_around_robot` 上，探索器一直用默认值，优化实际没生效。
- **效率参数（2026-11-03 实测最优）**：`angular_dist_threshold 3.1416`（不平移转向）、
  `approach_velocity_scaling_dist 0.25`、BT `RateController 0.25 Hz`、
  探索器 `replan_s 1.5`、目标超时 15 s。实测前进 45.6% / 静止 7.8% / 地图 +8529(80 s)。
- **不要靠加 inflation/footprint_padding 让过门更居中**：0.50 / 0.05 都会让墙边
  候选落点变成 cost>=99 → 探索器连续拉黑 → 车完全不走（实测）。保持 0.48 / 0.02。
- **脱困有两条触发路径**：一条受 `escape_after_failures`，另一条（连续 3 次不可通行）
  曾经是**硬编码**。脱困 = `_set_manual(True)` = 自动链路被门控清零 = 车彻底停住。
  现在两条都服从同一参数。
- **死胡同判定要数 `cost < 99` 的格子**，不能数 `cost == 0`：膨胀 0.48 让墙边
  代价都 >0，数 0 会把几乎所有前沿误判成死胡同（实测同一目标连续 73 次被拒）。
- **墙脚目标落点搜索半径 1.2 m，且找不到要回退前沿点本身**（不能返回 None）：
  返回 None 会导致"连续拉黑 → 只转不走"。修复后原地转 66% → 3.2%。
- **墙脚目标落点搜索半径 0.8 m**（原 0.4 m）：墙脚附近全被膨胀覆盖，0.4 m 内
  找不到合法落点 → 目标被拉黑 → 连续失败进脱困 → 左右转循环。
- **前沿不能太近**：兜底分支已要求 ≥0.5 m。以前会派 0.2 m 的前沿 → Nav2 认为已到点、
  `/cmd_vel_nav` 恒 0 → 车原地不动、探索器反复派同一点。
- **起步要开阔**：kickstart 在 `front < 0.45 m` 时会反复 blocked/orient，
  走不完就等 30 s 超时；这期间 `manual_enabled=True`，**自动链路全被门控清零**。
- **建图前查雷达**：`ros2 topic hz /scan` 应 ≈10 Hz。残留 `sllidar` 进程会占着
  `/dev/ttyUSB0`，新雷达起不来且**不报错** → ready=False、slam 无输入、TF 外推失败。
  用 `kill -9 <旧 PID>` 清掉。
- **超声不要改成"只在倒车时轮询"**：`rear_guard` 无数据会 fail-safe 报车尾被挡，
  探索器脱困永远失败 → `manual_enabled` 卡 True → 自动链路被门控清零。
- **节点"启动后一行日志都没有"= 大概率没挂上定时器**。已有回归测试
  `test_auto_mapping_nodes_wired` 守住这条。

**安全链两个已修 bug（2026-11-03，别再引入）**
- `directional_safety_selector._sector_min`：`if side>0 / else` 两块代码**曾经完全一样**，
  导致左右永远取同一值、"往空旷一侧转"永不触发。左前 = 150~180°，右前 = -180~-150°。
- `directional_safety_core`：ω 符号与监控回执不符时**整帧归零**会导致直线顿挫。
  现在前进行驶只归零转向、保留前进；纯旋转仍整帧拒绝（有单测守护，别改）。

**⚠️ 局部代价地图不要启用 `range_layer`**：它等不到超声数据时会把代价地图
更新拖住 → 控制器拿不到新鲜地图 → `computeVelocityCommands` 失败 →
`[follow_path] Aborting handle` 每秒一次、`/cmd_vel_nav` 恒 0。
现象是"短目标能到、长目标必卡"。倒车安全由 `rear_guard` 独立保障。

**Map Edit 面板**：橡皮工具已加发布去重 + 4 Hz 限速（原来每帧重发 `/map`，
30 Hz 冲垮 AMCL，是"车完全不动"的总根源）。

## 3.9 血泪教训（2026-11-01，务必遵守）

1. **"规划穿墙"先查数据源，别查算法**
   那天把规划器、代价地图、膨胀、旋转逻辑查了个遍，真凶是 `map_seal` 输出的
   `/map_sealed` 与 `/map` **逐点错位**（151 个墙点 0 个对得上）——
   代价地图里的墙根本不在看到的位置。
   **排查顺序**：`/map` → `/map_sealed` → `/global_costmap/costmap`
   三级**逐点**比对（同一世界坐标上三者的值必须一致），再怀疑算法。

2. **任何"坐标搬运/缩放/裁剪"代码必须逐点校验**
   固定画布、图像缩放、坐标系变换这类代码，**只看尺寸对是不够的**。
   要取真实特征点（墙点）验证落位。写完后立刻 A/B 比对。

3. **RViz 里的线不等于实时数据**
   Path 显示会一直保留最后一次收到的消息。判断"规划对不对"要抓话题
   （`/plan`、`/two_point_preview_path`）的**实时内容**，不是看画面。

4. **改安全/代价参数必须 A/B 回归**
   例：`track_unknown_space: true` 一次就让 3 组路径**全部规划失败**。
   改任何影响规划的参数，先记录"改之前能成功的 3 组起终点"，改完重跑对比。

5. **新增功能不得破坏旧功能**（用户反复强调）
   加新按钮/新逻辑时**复用已验证的函数**（如 `loadAndPublishMap`），
   不要另写一套并行实现。每次改完必须回归：旧路径至少跑一次。

## 3.98 🛡 防"车不动"的两道自检（2026-11-04）

2026-11-04 连续两次因为**低级代码错误**导致"点开始导航但车完全不动"：
新增回调方法没插进文件、新增字段没在 `__init__` 赋值。都补了自动检查。

### ① 静态守护（每次跑测试都会执行）
`test_nodes_attributes_defined.py`：AST 扫描所有节点类，
**读取了从未赋值过的 `self.xxx`** 就报错。这类 bug 运行时表现就是节点崩、车不动。
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
PYTHONPATH=ros2_ws/src/fwr_bringup .venv/bin/python -m pytest ros2_ws/src/fwr_bringup/test -q
```

### ② 运行自检（每次启动栈、开导航之前跑）
```bash
bash tools/check_stack.sh /tmp/fwr_runs/<本次日志>.log
```
检查六项：进程存活 / 节点在 ROS 图里 / 三个导航服务在位 /
**冻结参数是否被意外改动** / 雷达频率 / launch 日志有无 `process has died`。
全绿才开导航；不绿会列出具体哪一项。

## 4. 验证矩阵（先跑最小集，最后再全量）

| 改动类型 | 最小验证 |
| --- | --- |
| 纯 Python 核心/协议/几何 | `PYTHONPATH=ros2_ws/src/fwr_bringup .venv/bin/python -m pytest -q ros2_ws/src/fwr_bringup/test/test_<area>.py` |
| 影响多个节点/launch | 先跑对应 test 文件，再 `source /opt/ros/humble/setup.bash && .venv/bin/python -m pytest -q tests ros2_ws/src/fwr_bringup/test` |
| ROS 包/接口/launch | `cd ros2_ws && colcon build --packages-select fwr_interfaces fwr_bringup --symlink-install` |
| launch 语法 | `.venv/bin/python -m py_compile <launch.py>`；必要时 `ros2 launch <pkg> <file> --show-args` |
| ESP32-S3 固件 | 主机测试与 `idf.py build` 见 `firmware/esp32s3_base_control/README.md`；烧录前断开电机 12 V |
| 仿真 | 用隔离 `ROS_DOMAIN_ID` + 独立日志，例：`/tmp/fwr_*.log`；不要占用实机话题 |

测试证据必须写清：**命令 + 结果 + 日志路径**。只跑过测试不算“实机已验证”。

## 5. 安全红线

- **未经用户在当前会话明确同意，不向实机发运动命令**，不直接 `ros2 topic pub /cmd_vel`，不把 `motion_enabled` 改成 true。
- 不自动 `git commit` / `git push`；需要时先问用户。
- 不把 API key、密码写进仓库或配置文件；使用 `FWR_LLM_API_KEY` 等环境变量。
- 烧录/带电测试：先断电机 12 V、车轮悬空、可随时断电。软件停止 ≠ 硬件急停。
- 测试不依赖真实硬件；硬件结论必须单独标注“已验证 / 未验证”。

## 6. 省 token / 防重复（硬规则）

- **一个事实只写一处**：长期规则写本文件；会话结果写 `AGENT_LOG.md`；计划进度写 `progress.md`。不要三处复制。
- **先搜后做**：动手前用 `grep` 搜代码、日志和计划，确认没做过；不要凭印象重跑一遍排查。
- **输出要小**：`head` / `tail`、`--once`、`-l`、`wc -l`、`timeout`；大输出先落盘再 `grep`，不要把整个日志贴进上下文。
- **独立调用并行**：一个消息里批量发互不依赖的读取/搜索；大范围探索交给 subagent，只让它回传 ≤20 行结论。
- **不在会话中途重写本文件**：会改变提示前缀、降低 KV 缓存命中；要么开工前改，要么收尾时改。
- **阶段边界压缩**：长任务在完成一个阶段后用 `/compact` 或开新会话；不要把整段历史拖到上下文腐烂。
- **最小验证**：改什么跑什么；全量测试只在交接前跑一次，不反复刷。
- **失败也记**：试过但没用的方案写进日志，避免下个会话再踩。

## 7. 会话收尾（必须）

在 `docs/AGENT_LOG.md` 末尾追加一条（只追加、不删除历史，时间用 Asia/Shanghai）：

```markdown
## YYYY-MM-DD HH:MM · 一句话标题

### 会话目标
### 仓库改动（文件 / 新增 / 修改 / 删除）
### 仓库外改动（环境变量、系统配置、设备）
### 已验证（命令 + 结果 + 日志路径）
### 未验证 / 遗留问题
### 当前运行状态（进程、PID、端口）
### 下一步
```

同时更新对应 `progress.md`；只有**长期有效的事实**才回写本文件。下一位接手者应先读最后 2–3 条日志，不要假设“上次说的”已经落地。

更细的按需流程见项目 skill：`.agents/skills/fwr-workflow/SKILL.md`。
调研依据与插件清单见 `docs/agent/token-efficient-workflow.md`。
