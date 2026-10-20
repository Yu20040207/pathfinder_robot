# 自动前沿探索建图实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在封闭、有人看护的试验区，让实机用 C1 雷达在线建图、自动选择安全的未知区域、绕开新障碍，并在限定条件下沿最近轨迹短距离倒车脱困。

**Architecture:** SLAM Toolbox 继续独占地图与 `map→odom`；前沿探索器只向 Nav2 发送目标。Nav2 与回退控制器的速度命令经单一选择器和 Collision Monitor 后才到独占 ESP32 串口的底盘桥接。桥接对负速度另设默认关闭的限时权限，故障或链路中断立即停机。

**Tech Stack:** ROS 2 Humble、Python/rclpy、Nav2、SLAM Toolbox、nav2_collision_monitor、Gazebo Ignition Fortress、pytest、ESP32-S3 现有固件。

**Spec:** `docs/superpowers/specs/2026-10-20-autonomous-frontier-mapping-design.md`

## Global Constraints

- 底盘初始 footprint 为 0.60 m × 0.43 m，另加现有 0.02 m padding；首次实车自动运动前核对最大实际外廓，超出则先改参数。
- `/scan`、`/imu/data`、`/odom`、`/map` 与 TF 使用真实时间；ESP32 固件保留 300 ms 通信超时、失速锁定，不自动清故障。
- Nav2 普通 `/cmd_vel` 仍禁止负线速度；回退只走默认关闭的专用入口，单段 ≤0.10 m、单次累计 ≤0.30 m、最多一次尝试。
- 雷达车后盲区没有实时安全保证；实车回退仅限封闭、无人进入且有人目视车尾、可立即断电的试验区。
- 自动模式不能随进程重启、串口重连、故障清除而自动恢复；键盘接管优先停自动目标。
- 实机 Nav2 保持 `use_sim_time=false`，Gazebo 保持 `use_sim_time=true`；两者的启动和参数文件分开。编写 launch 与仿真文件时分别遵守 `ros2-launch`、`ros2-gazebo` 技能。

## Review Focus

1. `/map` 局部跳变或 `map→odom` 突变时，探索与回退均停车；Task 2、3 的测试注入位姿跳变。
2. Nav2 取消尚未完成时不得开始回退；Task 2 的假动作服务器测试验证先取消、再发回退请求。
3. 碰撞监测节点退出或其输出停更时，桥接不得继续沿用上一次速度；Task 4、5 的测试注入超时。
4. 后方局部地图旧障碍被清除但实际有新障碍时，不宣称传感器已证明安全；Task 3、7 的测试/验收要求现场看护并用障碍注入停机。
5. 探索结束、人工停止或目标反复失败后，节点重启不能自行打开运动门；Task 2、6 的测试验证默认停机。

---

## 文件结构

- `frontier_core.py`：纯栅格前沿搜索、可通行筛选、目标评分与迟滞；`test_frontier_core.py` 验证。
- `recent_track.py`：最近里程计轨迹与车体扫过区域；`backtrack_core.py`：分段逆向跟踪和中止条件；两者有纯逻辑测试。
- `frontier_explorer.py`：ROS 探索状态机、Nav2 action、地图/位姿订阅、回退触发与诊断；不直接向电机发串口命令。
- `auto_command_gate.py`：普通 Nav2 与回退命令互斥选择，只发布未经碰撞监测的自动速度；其输出再经 Collision Monitor。
- `fwr_interfaces/srv/RecoveryPermit.srv`：回退 episode 编号与启停请求，由桥接检查并限制单次回退预算。
- `base_bridge.py` / `base_bridge_core.py`：仅接收 Collision Monitor 输出的自动速度，人工通道保持独立；回退权限、速度、时长、故障门在此二次核验。
- `real_collision_monitor.yaml`、`real_exploration.yaml`、`real_navigation.launch.py`：实机参数与启动；`corridor_sim.launch.py` 扩展独立的 SLAM 探索仿真。
- `scripts/start_auto_mapping.sh`、`docs/hardware/auto-mapping.md`：显式开启、停机、保存地图与现场操作。

### Task 1: 纯前沿选择

**Files:**
- Create: `ros2_ws/src/fwr_bringup/fwr_bringup/frontier_core.py`
- Create: `ros2_ws/src/fwr_bringup/test/test_frontier_core.py`

**Interfaces:**
- Produces: `FrontierGoal(x: float, y: float, yaw: float, grid_index: int, score: float)`；`choose_frontier(grid: OccupancyGrid, robot: PoseStamped, blocked: set[int], current: FrontierGoal | None) -> FrontierGoal | None`。

- [ ] **Step 1: Write failing tests** for a free/unknown boundary, inflated footprint exclusion using half extents 0.32 m × 0.235 m, a narrow passage, unreachable frontier, failed index blacklist and keeping the current goal unless a new candidate has at least 20% lower cost.
- [ ] **Step 2: Run** `python3 -m pytest -q ros2_ws/src/fwr_bringup/test/test_frontier_core.py`; expect failures from missing module/behavior.
- [ ] **Step 3: Implement** four-neighbor reachable-free search, eight-neighbor frontier grouping, candidate centers on the free side, grid-resolution-aware clearance and distance/information score in `frontier_core.py`; reject unknown/occupied footprint cells.
- [ ] **Step 4: Run** the targeted tests; expect all pass. Commit only these two files.

### Task 2: 探索状态机与 Nav2 目标

**Files:**
- Create: `ros2_ws/src/fwr_bringup/fwr_bringup/frontier_explorer.py`
- Create: `ros2_ws/src/fwr_bringup/fwr_bringup/exploration_core.py`
- Create: `ros2_ws/src/fwr_bringup/test/test_exploration_core.py`
- Modify: `ros2_ws/src/fwr_bringup/setup.py`
- Modify: `ros2_ws/src/fwr_bringup/package.xml`

**Interfaces:**
- Consumes: Task 1 `choose_frontier`.
- Produces: `/explore/start` 与 `/explore/stop` (`std_srvs/Trigger`)，`/explore/state` (`std_msgs/String`)，Nav2 `NavigateToPose` action client；`ExplorationStateMachine.update(map_ok, pose_ok, fault, goal_status, now_s) -> ExplorationDecision`。

- [ ] **Step 1: Write failing tests** for disabled startup, valid goal dispatch, goal hysteresis, canceled goal before requesting recovery, failure blacklist after two attempts at one goal, stop after three consecutive failed goals, map/TF jump, operator stop and restart without auto resume.
- [ ] **Step 2: Run** `python3 -m pytest -q ros2_ws/src/fwr_bringup/test/test_exploration_core.py`; expect failures.
- [ ] **Step 3: Implement** pure state transitions, then the rclpy adapter subscribing `/map`, `/odom`, `/base/fault`, checking Nav2 lifecycle/action and publishing `/explore/state` at 10 Hz while active. Cancel/wait for terminal action state before allowing recovery. Register console script and `std_srvs`/`nav2_msgs` dependencies.
- [ ] **Step 4: Run** targeted tests and `python3 -m py_compile` on the new modules; expect pass. Commit this task's files.

### Task 3: 最近轨迹与受限回退

**Files:**
- Create: `ros2_ws/src/fwr_bringup/fwr_bringup/recent_track.py`
- Create: `ros2_ws/src/fwr_bringup/fwr_bringup/backtrack_core.py`
- Create: `ros2_ws/src/fwr_bringup/test/test_backtrack_core.py`
- Modify: `ros2_ws/src/fwr_bringup/fwr_bringup/frontier_explorer.py`

**Interfaces:**
- Produces: `RecentTrack.observe(odom: Odometry, now_s: float) -> None` and `BacktrackController.begin(track: RecentTrack, grid: OccupancyGrid, now_s: float) -> bool`; `BacktrackController.update(odom: Odometry, grid: OccupancyGrid, sensors_fresh: bool, fault: int, now_s: float) -> tuple[Twist, str]` where status is `running|done|aborted`.

- [ ] **Step 1: Write failing tests** for ≤0.10 m segment, ≤0.30 m total, one attempt, along-track steering sign, unknown/occupied swept footprint, stale scan/IMU/odom/map, pose jump, no wheel progress, time limit and cancel returning zero velocity.
- [ ] **Step 2: Run** `python3 -m pytest -q ros2_ws/src/fwr_bringup/test/test_backtrack_core.py`; expect failures.
- [ ] **Step 3: Implement** timestamped odom buffer and checked reverse tracking; publish `/cmd_vel_recovery_raw` only after Task 2 confirms Nav2 goal canceled. On any abort, zero command and exploration state `stopped`.
- [ ] **Step 4: Run** targeted tests; expect pass. Commit this task's files.

### Task 4: 自动命令选择与桥接权限

**Files:**
- Create: `ros2_ws/src/fwr_bringup/fwr_bringup/auto_command_gate.py`
- Create: `ros2_ws/src/fwr_bringup/test/test_auto_command_gate.py`
- Create: `ros2_ws/src/fwr_interfaces/srv/RecoveryPermit.srv`
- Modify: `ros2_ws/src/fwr_interfaces/CMakeLists.txt`
- Modify: `ros2_ws/src/fwr_bringup/fwr_bringup/base_bridge.py`
- Modify: `ros2_ws/src/fwr_bringup/fwr_bringup/base_bridge_core.py`
- Modify: `ros2_ws/src/fwr_bringup/test/test_base_bridge_core.py`

**Interfaces:**
- Consumes: upstream Nav2 `/cmd_vel`, Task 3 `/cmd_vel_recovery_raw`, Task 2 `/explore/state` with local receive timestamp.
- Produces: `/cmd_vel_auto_unchecked` (`Twist`) into Collision Monitor; only `/cmd_vel_auto_safe` from monitor reaches bridge. `AutoCommandGate.select(nav_cmd, recovery_cmd, mode, now_s) -> Twist` defaults to zero. `RecoveryPermit.srv` request is `bool enable, uint32 episode_id`, response is `bool accepted, string reason`; the bridge tracks a monotonic episode ID, requires Nav2 action inactive, manual mode off and healthy sensors, and enforces ≤0.30 m or ≤10 s even if the caller fails. The gate emits reverse only for a fresh `reversing` state; bridge accepts negative speed only during the matching permit.

- [ ] **Step 1: Write failing tests** for manual takeover, simultaneous nav/recovery input, recovery mode transition with stale nav command, command timeout, bridge reject of unfiltered `/cmd_vel`, bridge negative-command rejection outside recovery, 0.10/0.30 m limits and fault/serial reset closing permit.
- [ ] **Step 2: Run** `python3 -m pytest -q ros2_ws/src/fwr_bringup/test/test_auto_command_gate.py ros2_ws/src/fwr_bringup/test/test_base_bridge_core.py`; expect failures.
- [ ] **Step 3: Implement** source arbiter, recovery permit service and filtered-command callback. Keep the bridge as sole serial owner, disable direct Nav2 bypass, and require a fresh recovery episode ID/state plus monotonic distance/time budget before any negative wheel command. Have Task 2's ROS adapter request/revoke the permit around Task 3's controller, including cancellation/fault exits. Register the new service in `fwr_interfaces/CMakeLists.txt`.
- [ ] **Step 4: Run** targeted and existing bridge tests; expect pass. Commit this task's files.

### Task 5: Collision Monitor 与实机启动链

**Files:**
- Create: `ros2_ws/src/fwr_bringup/config/real_collision_monitor.yaml`
- Create: `ros2_ws/src/fwr_bringup/config/real_exploration.yaml`
- Modify: `ros2_ws/src/fwr_bringup/launch/real_navigation.launch.py`
- Modify: `ros2_ws/src/fwr_bringup/config/real_nav2.yaml`
- Modify: `ros2_ws/src/fwr_bringup/test/test_real_nav2_config.py`
- Create: `ros2_ws/src/fwr_bringup/test/test_real_auto_mapping_config.py`

**Interfaces:**
- Collision Monitor input `/cmd_vel_auto_unchecked`, output `/cmd_vel_auto_safe`; scan `/scan`; base frame `base_footprint`. Real launch still defaults `autostart=false`, `motion_enabled=false`, recovery disabled.

- [ ] **Step 1: Write failing config tests** that assert one SLAM owner, no direct `/cmd_vel` subscriber on bridge, monitor input/output wiring, front stop/slow polygons, visible rear-side stop polygon, scan/TF timeout, model footprint with padding, Nav2 no generic BackUp/Spin and default motion gates closed.
- [ ] **Step 2: Run** the two config test files; expect failures.
- [ ] **Step 3: Add** monitor and exploration nodes/parameters in the real launch. Use Nav2 Humble's installed Collision Monitor parameter names; configure polygons from measured stopping distance before real motion. Preserve no automatic reverse in Nav2 behavior tree and RPP. Compile the launch with `python3 -m py_compile`.
- [ ] **Step 4: Run** config tests and `colcon build --symlink-install --packages-select fwr_bringup`; expect pass. Commit this task's files.

### Task 6: 仿真探索与故障注入

**Files:**
- Modify: `ros2_ws/src/fwr_bringup/launch/corridor_sim.launch.py`
- Create: `ros2_ws/src/fwr_bringup/config/corridor_auto_bridge.yaml`
- Create: `ros2_ws/src/fwr_bringup/test/test_sim_auto_mapping_config.py`
- Create: `scripts/check_auto_mapping_sim.py`

**Interfaces:**
- Uses Tasks 1–5 against Gazebo in a distinct `ROS_DOMAIN_ID`; simulation mapping + Nav2 have one map publisher and one `map→odom` owner.

- [ ] **Step 1: Write failing launch/config tests** ensuring `mapping:=true` excludes the static map server and starts exploration disabled; simulator receives commands only through the safety gate.
- [ ] **Step 2: Run** targeted tests; expect failures.
- [ ] **Step 3: Adapt** the existing corridor to Nav2-with-SLAM. In auto mode, bridge `/cmd_vel_auto_safe` to Gazebo `/cmd_vel` via `corridor_auto_bridge.yaml`, keeping the original manual/simulation config unchanged. Add a bounded headless check that records map growth, no goal flip-flop, obstacle reroute, one reverse ≤0.30 m, scan loss and operator-stop zero output. Do not use the physical ROS domain.
- [ ] **Step 4: Run** `python3 -m pytest -q ros2_ws/src/fwr_bringup/test` and the headless simulation check; expect passing assertions and no unexpected command after stop. Commit this task's files.

### Task 7: 显式实机启动、地图保存与分级验收

**Files:**
- Create: `scripts/start_auto_mapping.sh`
- Create: `docs/hardware/auto-mapping.md`
- Create: `ros2_ws/src/fwr_bringup/test/test_auto_mapping_script.py`

**Interfaces:**
- Script checks model/footprint agreement, sensor and TF freshness, fault 0, Nav2/monitor state, operator trial arming; enables only the automatic mode. Stop/exit cancels goal, sends zero, closes bridge gates and saves trial map outside packaged simulation maps.

- [ ] **Step 1: Write failing script tests** for a stale scan, absent Collision Monitor, nonzero fault, duplicate keyboard controller, interrupted shell process and no auto-resume after restart.
- [ ] **Step 2: Run** targeted script tests; expect failures.
- [ ] **Step 3: Implement** explicit start/stop and date-stamped `map_saver_cli` output; document the closed-area operator checklist, measured footprint/stopping-distance fields, RViz checks and the sequence static → forward goal → 0.10 m reverse → 0.30 m reverse → bounded exploration.
- [ ] **Step 4: Run** `bash -n scripts/start_auto_mapping.sh`, targeted tests, full package tests and `git diff --check`; expect pass. With motor gates closed, perform static ROS topic/TF/action/monitor inspection. Commit files.
- [ ] **Step 5: Perform live trials only after static/simulation evidence and on-site conditions in the spec are verified**; record `/base/fault`, `/odom`, `/imu/data`, `/scan`, selected goals, filtered commands and actual stopping distance. Increase the trial boundary only if the previous stage stopped correctly and produced a coherent saved map.

## Completion Evidence

Archive the simulation logs, static ROS checks, real trial logs and saved map path in `docs/hardware/auto-mapping.md`. Mark the feature usable only for the explicitly validated load, floor, room boundary and supervision mode. Keep the automatic motion gates off at the end of each test.
