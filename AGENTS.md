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
