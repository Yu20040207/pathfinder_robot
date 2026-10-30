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
  ±60° 起是真实距离。`/scan_mapping` 现在屏蔽后方 ±55°（留 10° 过渡余量），
  有效视场约 250°；历史配置（±90°）白扔了两侧各 45°。
  由此得到两条硬约束：
  - `/scan` 的后方扇区（scan 角度 0° 附近 = ranges 数组中部）读到的是**车身**，不是障碍；
    `/scan_mapping` 用 `mask_rear_ranges` 屏蔽这一段，建图只用前方+两侧。
  - 任何拿 `/scan` 判断“是否被堵 / 能不能转向”的代码**必须忽略后方扇区**，
    否则会把自己车体当成墙（自动建图曾因此误判“三面被堵”进入死循环）。
  - 雷达倒装 180°：`/scan` 角度范围 [-180°,+180°]，**index 0 侧 = 车头方向**，index 中部 = 车尾。
- ESP32-S3 底盘控制：`/dev/ttyACM1`、115200、`BASE_CONTROL_V2`，`S2` 为 14 字段；IMU：`/dev/ttyACM0`、921600。
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
