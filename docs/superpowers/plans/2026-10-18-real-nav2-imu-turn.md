# Real Robot Nav2 and IMU Turn Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Connect the real ESP32-S3 chassis, C1 lidar, and DM-IMU-L1 to ROS 2, online SLAM, and Nav2, then add bounded relative pivot and arc turns.

**Architecture:** One ROS 2 bridge owns the ESP32 serial port, publishes wheel data and faults, and arbitrates Nav2 velocity commands against a relative-turn action. An EKF owns `odom→base_footprint`; SLAM owns `map→odom`. Real launches and parameters stay separate from Gazebo. Motion remains disabled until loaded drive and sensing gates pass.

**Tech Stack:** ROS 2 Python (`rclpy`), ESP-IDF C, `robot_localization`, `slam_toolbox`, Nav2, pytest, C host tests.

**Spec:** `docs/superpowers/specs/2026-10-18-real-nav2-imu-turn-design.md`

## Global Constraints

- Real nodes use `use_sim_time: false`; the simulation config is unchanged.
- Wheel circumference 0.210 m, track 0.263 m, and 1468 counts/rev are initial calibration values.
- ESP32 serial watchdog remains 300 ms; fault latch cannot be cleared automatically.
- `t` remains a bounded diagnostic command, never Nav2's continuous velocity interface.
- Rear C1 scan is obstructed: no autonomous reverse or unattended pivot until rear sensing is validated.
- Do not run powered ground autonomy before the current left-turn stall and loaded `v` control are resolved.
- Respect the user's standing instruction to keep 12 V connected during tests, with immediate disconnect available.

## Review Focus

- Malformed, partial, or overlong ESP32 serial records must not become valid odometry or a motion command (Task 1 tests).
- USB disconnect/reconnect during a command must stop motion and require a fresh deliberate command (Task 2 tests).
- Stale `/cmd_vel`, stale IMU, and active fault must yield zero output without auto-clearing the latch (Tasks 2 and 5 tests).
- Encoder wrap/reboot and opposite wheel sign conventions must not jump the odometry pose (Task 2 tests).
- Navigation and a relative-turn goal arriving together must have a single owner and an explicit rejection/cancel result (Task 5 tests).

---

### Task 1: ESP32 serial contract and safe parser

**Files:** Modify `firmware/esp32s3_base_control/main/main.c`, `firmware/esp32s3_base_control/main/protocol.h`, `firmware/esp32s3_base_control/tests/test_protocol.c`; create `ros2_ws/src/fwr_bringup/fwr_bringup/base_protocol.py`, `ros2_ws/src/fwr_bringup/test/test_base_protocol.py`.

**Interfaces:** `parse_state_line(line: str) -> BaseState | None`, where `BaseState` carries ESP monotonic microseconds, left/right signed counts, skipped counts, measured/setpoint speeds, PWM, active, and fault. `encode_wheel_speed(left_mm_s: float, right_mm_s: float) -> bytes` produces one newline-terminated `v` record. Protocol exposes `BASE_CONTROL_V2` handshake and a fixed maximum line length; `x` remains immediate stop. No change to motor actuation in this task.

- [ ] Write C and Python failing tests for complete/partial/overlong/malformed state lines, finite speed bounds of ±100 mm/s, fault codes, and handshake mismatch.
- [ ] Run `pytest -q ros2_ws/src/fwr_bringup/test/test_base_protocol.py` and the existing firmware host-test command from `firmware/esp32s3_base_control/README.md`; verify new cases fail.
- [ ] Add versioned state record including `skipped[2]` and parser/encoder with bounded input and finite numeric checks. Keep legacy diagnostics available for bench use.
- [ ] Re-run both suites; require all new and existing protocol/control tests pass. Commit this task's files.

### Task 2: Single-owner chassis bridge and wheel odometry

**Files:** Create `ros2_ws/src/fwr_bringup/fwr_bringup/base_bridge.py`, `ros2_ws/src/fwr_bringup/fwr_bringup/base_bridge_core.py`, `ros2_ws/src/fwr_bringup/test/test_base_bridge_core.py`; modify `ros2_ws/src/fwr_bringup/fwr_bringup/wheel_odometry.py`, `ros2_ws/src/fwr_bringup/test/test_wheel_odometry.py`, `ros2_ws/src/fwr_bringup/setup.py`, `ros2_ws/src/fwr_bringup/package.xml`.

**Interfaces:** `BaseBridgeCore.accept_cmd(v_mps: float, omega_rps: float, now_s: float) -> tuple[float,float]` computes wheel mm/s using 0.263 m track and rejects values outside firmware limits; `step(state: BaseState | None, now_s: float) -> BridgeOutput` returns odometry, fault/health and the next serial command. ROS node publishes `/wheel/odom` (`base_footprint` child), `/joint_states`, `/base/fault`, `/base/ready`; subscribes `/cmd_vel`. It is the only process opening the ESP port; the old inspection `wheel_odometry` executable remains available only when the bridge is stopped.

- [ ] Write failing tests for forward/reverse/left turn mapping; encoder sign, wrap, and reboot; stale state or velocity; fault latch; USB drop/reconnect; invalid record; and stopped startup. Assert no command after reconnect until a new `/cmd_vel` arrives.
- [ ] Run `pytest -q ros2_ws/src/fwr_bringup/test/test_base_bridge_core.py ros2_ws/src/fwr_bringup/test/test_wheel_odometry.py`; verify new cases fail.
- [ ] Implement pure core, nonblocking exclusive serial open, periodic state handling, wheel odometry/joint-state publishing, and zero/`x` on shutdown. Never send `c` automatically.
- [ ] Re-run tests and `colcon build --packages-select fwr_bringup --symlink-install`; require pass. Commit task files.

### Task 3: Loaded continuous wheel-speed controller

**Files:** Modify `firmware/esp32s3_base_control/main/control.c`, `firmware/esp32s3_base_control/main/control.h`, `firmware/esp32s3_base_control/tests/test_control.c`, `firmware/esp32s3_base_control/README.md`.

**Interfaces:** Existing `base_command(base_control_t *, float left_mm_s, float right_mm_s, int64_t now)` remains the sole continuous `v` path; accepts ±100 mm/s, outputs bounded PWM, and preserves 300 ms timeout and fault latch. `base_turn_command` keeps its 0.5 s diagnostic limit. Tuning constants are named and logged; there is no automatic fault clear.

- [ ] Add failing host tests for loaded forward/reverse/pivot startup, left wheel no-progress fault, one wheel speeding ahead, command timeout, sign reversal, and PWM zero after stop/fault.
- [ ] Run firmware host tests and confirm new cases fail.
- [ ] Tune only the `v` control path, using encoder feedback and bounded startup assistance; do not treat open-loop `d`, `b`, or `t` tests as proof of `v` control.
- [ ] Re-run host tests and `idf.py build`; require pass. Do not flash or command ground motion until Task 6's hardware gate. Commit task files.

### Task 4: Real TF, IMU fusion, and manual SLAM

**Files:** Create `ros2_ws/src/fwr_bringup/config/real_ekf.yaml`, `ros2_ws/src/fwr_bringup/config/real_slam.yaml`, `ros2_ws/src/fwr_bringup/launch/real_mapping.launch.py`, `ros2_ws/src/fwr_bringup/test/test_real_mapping_config.py`; modify `ros2_ws/src/fwr_bringup/package.xml` and, only after measuring mounts, `ros2_ws/src/fwr_bringup/description/fwr_chassis_demo.urdf`.

**Interfaces:** EKF subscribes `/wheel/odom` and `/imu/data`, publishes `/odom` and only `odom→base_footprint`; `slam_toolbox` consumes `/scan`, `/odom`, publishes `/map` and only `map→odom`. `robot_state_publisher` owns fixed/joint TF. All real nodes use wall time.

- [ ] Write failing config tests for frame IDs, topic names, unique TF ownership, `use_sim_time: false`, no static `map→odom`, and no Gazebo launch inclusion.
- [ ] Run `pytest -q ros2_ws/src/fwr_bringup/test/test_real_mapping_config.py`; verify failures.
- [ ] Add dependencies/config/launch, using installed ROS distro's `robot_localization` and `slam_toolbox` parameters. Measure C1/IMU frame offsets before editing URDF; leave provisional values explicit if measurement is unavailable.
- [ ] Build with `colcon build --packages-select fwr_imu fwr_bringup --symlink-install`; run config tests. On hardware, with motors stopped, verify `/scan`, `/imu/data`, `/odom`, and one continuous `map→odom→base_footprint→base_link→laser_front`; save a manually acquired map. Commit task files.

### Task 5: Relative turn action and command arbitration

**Files:** Create `ros2_ws/src/fwr_interfaces/action/TurnRelative.action`, `ros2_ws/src/fwr_bringup/fwr_bringup/turn_controller.py`, `ros2_ws/src/fwr_bringup/test/test_turn_controller.py`; modify `ros2_ws/src/fwr_interfaces/CMakeLists.txt`, `ros2_ws/src/fwr_interfaces/package.xml`, `ros2_ws/src/fwr_bringup/fwr_bringup/base_bridge.py`, `ros2_ws/src/fwr_bringup/fwr_bringup/base_bridge_core.py`.

**Interfaces:** Goal: `float32 angle_deg`, `uint8 mode` (`PIVOT=1`, `ARC=2`); result: `bool success`, `string reason`, `float32 actual_angle_deg`; feedback: `float32 remaining_angle_deg`. `TurnController.update(relative_yaw_rad: float, yaw_rate_rad_s: float, now_s: float, left_progress_m: float, right_progress_m: float) -> tuple[float,float] | TurnFailure` returns wheel mm/s. Goal owns the bridge until success/cancel/failure; Nav2 `/cmd_vel` is rejected during ownership. Mode selection does not bypass the rear-coverage gate.

- [ ] Write failing tests for ±5° wraparound, pivot/arc wheel signs, near-target slowdown and ±2° stop band, simultaneous Nav2/turn ownership, cancellation, stale IMU, stale encoder, overshoot, fault and timeout. Assert output zero on every terminal path.
- [ ] Run `pytest -q ros2_ws/src/fwr_bringup/test/test_turn_controller.py`; verify failures.
- [ ] Implement action and pure turn controller with bounded speed/PWM through `v` only; enforce fresh IMU, recent encoder progress, wall deadline, and no auto-retry after fault.
- [ ] Build `fwr_interfaces` then `fwr_bringup`, run tests. Do not expose ground pivot until Task 6. Commit task files.

### Task 6: Real Nav2 configuration and staged hardware acceptance

**Files:** Create `ros2_ws/src/fwr_bringup/config/real_nav2.yaml`, `ros2_ws/src/fwr_bringup/launch/real_navigation.launch.py`, `ros2_ws/src/fwr_bringup/test/test_real_nav2_config.py`, `docs/hardware/real-nav2-acceptance.md`; modify `ros2_ws/src/fwr_bringup/package.xml`.

**Interfaces:** `real_navigation.launch.py` includes `nav2_bringup/bringup_launch.py` only after map/TF/bridge readiness; initial motion gate defaults to disabled. Config uses real `/scan`, `map/odom/base_footprint`, measured footprint plus clearance, low forward speed and gradual steering; reverse and autonomous pivot are disabled while rear scan is obscured. Startup must not publish motion merely because Nav2 lifecycle nodes activate.

- [ ] Write failing tests for `use_sim_time: false`, frame/topic alignment, lidar-only obstacle+inflation layers, no automatic backup/spin behavior, disabled initial motion gate, and separate simulation parameters.
- [ ] Run `pytest -q ros2_ws/src/fwr_bringup/test/test_real_nav2_config.py`; verify failures.
- [ ] Implement real params/launch from installed Nav2 asset baseline, with measured footprint and scan geometry; document how to keep the gate closed when measurements or `robot_localization` are missing.
- [ ] Run ROS build/config tests. Verify static stop paths (stale command, IMU/scan loss, bridge exit, serial loss) before powering motion.
- [ ] With the user present and space/weight checked, flash the Task 3 firmware and test `v` at current 2.5 kg: repeated short forward, reverse, and both pivot directions; confirm no left stall, fault, reboot, or uncontrolled yaw. If any fail, keep gate closed and diagnose; do not claim Nav2 driving ready.
- [ ] Only after stable drive, measured TF/footprint, saved map and localization: test bounded ±5° action, cancellation, then one short low-speed Nav2 path with obstacle and stop checks. Record actual IMU error, wheel counts, and stop behavior in the acceptance document. Retest after final payload mass changes. Commit config and acceptance record.

## Final verification

- [ ] Run firmware host tests, package pytest suites, `colcon build` for changed packages, and `git diff --check`.
- [ ] Check a single owner for each serial device and each TF edge; inspect live `/base/fault`, `/odom`, `/scan`, `/imu/data`, and Nav2 action availability.
- [ ] Obtain independent code review before calling integration complete. Report separately which software gates passed and which real-ground gates remain blocked.
