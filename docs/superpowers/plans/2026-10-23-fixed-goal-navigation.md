# 实车单次定点导航实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让操作员通过现有自动导航安全链发送一个定点目标，并在结束或异常时停机。

**Architecture:** `frontier_explorer` 独占导航状态与 Nav2 goal。新服务接收目标；单次模式复用原有指令门与传感器保护，但不选择前沿、不自动恢复。

**Tech Stack:** ROS 2 Humble、rclpy、Nav2、pytest。

**Spec:** `docs/superpowers/specs/2026-10-23-fixed-goal-navigation-design.md`

## Global Constraints

- 不操作真实硬件直至软件和无动力验证结束。
- 不增加第二个 `/explore/state` 发布者。
- `motion_enabled=false` 保持默认；任何失效均归零。
- 不提交或推送 Git；保留现有未提交改动。

## Review Focus

- 非有限或地图外坐标必须拒绝，不能发送 Nav2 goal。
- 占据或未知地图单元必须拒绝；不能以地图边缘单元作为车体中心。
- 目标处理时重复调用、探索启动或手动接管不能造成第二个 goal。
- 取消请求到 Nav2 action 完成之间不能提前重新 arm。
- 故障、定位跳变、超时后不能自动恢复运动。

---

### Task 1: 目标请求与地图验证

**Files:**
- Create: `ros2_ws/src/fwr_interfaces/srv/StartFixedGoal.srv`
- Modify: `ros2_ws/src/fwr_interfaces/CMakeLists.txt`
- Create: `ros2_ws/src/fwr_bringup/fwr_bringup/fixed_goal_core.py`
- Create: `ros2_ws/src/fwr_bringup/test/test_fixed_goal_core.py`

**Interfaces:** `validate_fixed_goal(grid, x, y, yaw, max_distance_m) -> tuple[bool, str]`；服务输入 `float64 x/y/yaw`，输出 `bool accepted, string reason`。

- [ ] 写合法目标、非有限、越界、未知/占据、距离上限的失败测试。
- [ ] 用 `python3 -m pytest -q ros2_ws/src/fwr_bringup/test/test_fixed_goal_core.py` 确认红灯。
- [ ] 实现纯验证函数与服务定义；编译 `fwr_interfaces`。
- [ ] 重跑该测试确认通过。

### Task 2: 单次目标状态机与 ROS 接口

**Files:**
- Modify: `ros2_ws/src/fwr_bringup/fwr_bringup/frontier_explorer.py`
- Modify: `ros2_ws/src/fwr_bringup/fwr_bringup/exploration_core.py`
- Modify: `ros2_ws/src/fwr_bringup/test/test_exploration_core.py`
- Create: `ros2_ws/src/fwr_bringup/test/test_fixed_goal_integration.py`

**Interfaces:** `/explore/start_fixed_goal` 服务；沿用 `/explore/stop`、`/explore/state` 和 `NavigateToPose`。

- [ ] 先写忙碌拒绝、导航成功后单次停机、故障/取消/超时不重派发测试并确认失败。
- [ ] 实现节点接收、派发和收尾，保留原有探索逻辑。
- [ ] 运行相关单元测试，修正失败。
- [ ] `colcon build --packages-select fwr_interfaces fwr_bringup`，再运行配置及状态机测试。

### Task 3: 仿真与实车无动力验收

**Files:**
- Modify: `scripts/check_fixed_goal_reroute_sim.py`
- Modify: `docs/hardware/real-map-localization.md`

- [ ] 仿真脚本改用新服务，不发布虚假的 `/explore/state`；确认遇障重规划、取消后归零。
- [ ] 以隔离 `ROS_DOMAIN_ID` 运行仿真，记录结果。
- [ ] 实车只读检查服务、地图、AMCL、Nav2 生命周期、故障与运动门；保持 `motion_enabled=false`。
- [ ] 验证拒绝无动力启动，记录带电短测前仍需检查的现场条件。

## 2026-10-23 进度

- Task 1：目标验证测试先失败后通过；`fwr_interfaces` 构建成功。
- Task 2：单次状态机测试先失败后通过；ROS 服务已接入，`fwr_bringup` 构建成功；完整测试 171 项通过。
- Task 3：隔离仿真单次目标成功到达并回到 `stopped`；脚本不再伪造状态。正前方加入障碍时 Collision Monitor 停车、Nav2 终止目标，重新绕行尚未通过。运行中的实车节点仍是旧进程，`motion_enabled=false`，新服务尚未在实车无动力复核。

## 2026-10-23 部署补充

- 修正静态地图晚加入问题：`map_server` 的持久地图使用 transient-local 订阅，并在保存地图模式下按已接收的地图缓存判断；实时 SLAM 仍要求 1 秒内更新。测试 173 项通过。
- 仅重启实车探索节点加载新服务；无动力调用显示 `preflight failed: arm`，说明地图已经就绪，而底盘仍未授权。`/base_bridge motion_enabled=false`、`/explore/state=stopped`。
