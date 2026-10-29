# Directional Collision Monitor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make supervised, bounded reverse recovery pass through a rear-facing Nav2 Collision Monitor while forward travel retains front obstacle protection.

**Architecture:** Two ROS Humble Collision Monitor lifecycle nodes consume `/cmd_vel_auto_unchecked` and the same live scan, each with direction-specific zones. A fail-closed Python selector is the sole publisher of `/cmd_vel_auto_safe`; the existing bridge or simulation permit remains its consumer. Both launch paths and real preflight require the new chain, with simulation proving actual bounded reverse motion before any real reverse trial.

**Tech Stack:** ROS 2 Humble, Nav2 Collision Monitor 1.1.20, Python/rclpy, pytest, Gazebo Fortress.

**Spec:** `docs/superpowers/specs/2026-10-21-directional-collision-monitor-design.md`

## Global Constraints

- Keep ROS Humble Collision Monitor 1.1.20; do not assume `VelocityPolygon` exists or upgrade Nav2.
- Forward monitor uses front stop and slow zones; reverse monitor uses the side/rear visible stop zone. Both consume the same raw Twist and live scan.
- `/cmd_vel_auto_safe` has exactly one publisher: `directional_safety_selector`. Bridge/permit remain subscribers, and manual control remains separate.
- Default output is zero. For zero linear speed with nonzero angular speed, require new, matching outputs from both monitors and forward only the smaller same-sign angular magnitude; either monitor's stop or timeout yields zero (user-approved design addition on 2026-10-21).
- Direction changes clear stale results; reject stale, opposite-sign or larger-than-input filtered commands. Missing selected monitor, scan or explore state yields zero.
- Preserve bridge watchdog/fault/authorization, backtrack permission and the 0.10 m single-segment and 0.30 m cumulative reverse bounds. Real automatic reverse remains disabled.
- Keep existing worktree modifications; do not run real hardware, commit or push in this plan's execution.

## Review Focus

1. A zero Twist emitted by a blocked forward monitor must never cause the reverse output to be chosen — selector core tests in Task 1.
2. Out-of-order Twist messages after a direction switch must remain stopped — selector core tests in Task 1.
3. A monitor that is active but stops publishing, or a stale scan, must stop the vehicle — selector node and integration tests in Tasks 2 and 4.
4. Two lifecycle nodes must both become active and must not create a second safe-topic publisher — static and preflight tests in Tasks 3 and 4.
5. Existing forward obstacle rerouting must still complete with dual-monitored zero-linear turns — regression in Task 5; a failure blocks real deployment.

---

### Task 1: Pure fail-closed selector policy

**Files:**
- Create: `ros2_ws/src/fwr_bringup/fwr_bringup/directional_safety_core.py`
- Create: `ros2_ws/src/fwr_bringup/test/test_directional_safety_core.py`

**Interfaces:**
- Produce `DirectionalSafetyCore.observe_raw(linear_x: float, angular_z: float, now: float) -> None`, `observe_filtered(direction: str, linear_x: float, angular_z: float, now: float) -> None`, `observe_scan(now: float) -> None`, `observe_explore_state(state: str, now: float) -> None`, `observe_manual(enabled: bool, now: float) -> None`, and `select(now: float) -> tuple[float, float]`.
- `direction` accepts only `forward` or `reverse`. Invalid/non-finite values fail closed. Freshness uses monotonic receive time: raw/filter ≤0.15 s, scan/state ≤0.25 s. A filtered result must arrive after the current raw command and current direction epoch; the filtered linear/angular absolute values may not exceed the raw values (small floating-point tolerance only).

- [ ] **Step 1: Write failing policy tests.** Cover forward pass, forward stop zero, reverse pass, startup zero, zero-linear angular command requiring both monitors, direction switch zero until new matching result, stale/reordered result, wrong sign, overlarge linear/angular, NaN/Inf, scan/state/raw/filter timeout, and manual/disabled state zero.
- [ ] **Step 2: Run `python3 -m pytest ros2_ws/src/fwr_bringup/test/test_directional_safety_core.py -q`; confirm tests fail because the core is absent.**
- [ ] **Step 3: Implement these methods and direction-epoch cache invalidation.** Use the gate's existing state semantics (`reversing` for reverse, `navigating` for forward); never reuse an earlier monitor output to match a later raw Twist. If same-direction raw messages arrive faster than filtered messages, allow only a filtered result observed after the latest raw message and within its freshness window.
- [ ] **Step 4: Rerun the core tests; confirm all pass.**

### Task 2: ROS selector wrapper and entry point

**Files:**
- Create: `ros2_ws/src/fwr_bringup/fwr_bringup/directional_safety_selector.py`
- Create: `ros2_ws/src/fwr_bringup/test/test_directional_safety_selector.py`
- Modify: `ros2_ws/src/fwr_bringup/setup.py`

**Interfaces:**
- Consume Task 1 core. Subscribe to `/cmd_vel_auto_unchecked`, `/cmd_vel_auto_forward_filtered`, `/cmd_vel_auto_reverse_filtered`, parameterized `scan_topic` (`/scan` real, `/scan_safety` sim), `/explore/state` (`std_msgs/String`) and `/base/manual_enabled` (`std_msgs/Bool`). Publish `/cmd_vel_auto_safe` at 20 Hz, including explicit zero on startup, invalid or stale input, and shutdown. Expose entry point `directional_safety_selector = fwr_bringup.directional_safety_selector:main`.

- [ ] **Step 1: Write failing wrapper tests.** Check exact topic names, scan parameter, 20 Hz zero-output timer, invalid state/scan timeout behavior, and entry point registration; mock rclpy or test isolated callbacks without running ROS hardware.
- [ ] **Step 2: Run `python3 -m pytest ros2_ws/src/fwr_bringup/test/test_directional_safety_selector.py -q`; confirm failure.**
- [ ] **Step 3: Add the ROS wrapper and entry point.** Use monotonic receive timestamps, the existing `/explore/state` semantics, and publish the core's selected Twist on each timer tick. Ensure destroy/shutdown emits zero while publisher is still alive.
- [ ] **Step 4: Rerun wrapper and core tests; confirm pass.**

### Task 3: Split monitor configuration and wire both launch paths

**Files:**
- Create: `ros2_ws/src/fwr_bringup/config/real_collision_monitor_forward.yaml`
- Create: `ros2_ws/src/fwr_bringup/config/real_collision_monitor_reverse.yaml`
- Modify: `ros2_ws/src/fwr_bringup/launch/real_navigation.launch.py`
- Modify: `ros2_ws/src/fwr_bringup/launch/corridor_sim.launch.py`
- Modify: `ros2_ws/src/fwr_bringup/test/test_real_auto_mapping_config.py`
- Modify: `ros2_ws/src/fwr_bringup/test/test_sim_auto_mapping_config.py`
- Retire from active launch: `ros2_ws/src/fwr_bringup/config/real_collision_monitor.yaml`

**Interfaces:**
- Both monitors take `/cmd_vel_auto_unchecked`. Named `collision_monitor_forward` and `collision_monitor_reverse`, they publish `/cmd_vel_auto_forward_filtered` and `/cmd_vel_auto_reverse_filtered`. One lifecycle manager activates both named nodes. Real configs use `base_footprint` and `/scan`; sim rewrites frame to `base_link`, time to simulated time, and scan to `/scan_safety`. Both launch files start the Task 2 selector with the matching scan parameter.

- [ ] **Step 1: Change config tests so they fail on the current single-monitor wiring.** Assert zone separation, exact in/out topics, two lifecycle node names, scan rewrite, selector launch, `stop_pub_timeout` at least 17 s and at most 20 s, and only one source publisher of `/cmd_vel_auto_safe`.
- [ ] **Step 2: Run `python3 -m pytest ros2_ws/src/fwr_bringup/test/test_real_auto_mapping_config.py ros2_ws/src/fwr_bringup/test/test_sim_auto_mapping_config.py -q`; confirm expected failure.**
- [ ] **Step 3: Add two YAMLs and rewire real and sim launches.** Preserve polygon geometry and `source_timeout: 0.25`; forward config lists `front_stop, front_slow`, reverse config lists `rear_visible_stop`. Keep the legacy config file unreferenced until tests/docs have migrated, then remove it if no active reader remains.
- [ ] **Step 4: Rerun config tests and validate both launch files with `python3 -m py_compile` and `ros2 launch ... --show-args` in a sourced workspace; confirm pass without starting hardware.**

### Task 4: Real preflight and lost-monitor safety checks

**Files:**
- Modify: `ros2_ws/src/fwr_bringup/fwr_bringup/auto_mapping_preflight.py`
- Create: `ros2_ws/src/fwr_bringup/test/test_directional_preflight.py`
- Modify: `ros2_ws/src/fwr_bringup/test/test_directional_safety_selector.py`

**Interfaces:**
- Preflight queries `/collision_monitor_forward/get_state` and `/collision_monitor_reverse/get_state`; both must be `active`. It also checks the selector node exists and the safe topic's sole publisher is that selector. Keep all current map/TF, Nav2, fault, manual and measured-footprint checks.

- [ ] **Step 1: Write failing tests.** One inactive monitor, missing selector, duplicate safe publisher, stale selected monitor output, lost scan, and manual takeover all reject/zero; both active monitors and a sole selector satisfy the new monitor portion of preflight.
- [ ] **Step 2: Run `python3 -m pytest ros2_ws/src/fwr_bringup/test/test_directional_preflight.py ros2_ws/src/fwr_bringup/test/test_directional_safety_selector.py -q`; confirm expected failure.**
- [ ] **Step 3: Implement dual lifecycle checks and publisher ownership in preflight; ensure selector timer zeroes on each stale/lost input.** Do not relax other preflight gates.
- [ ] **Step 4: Rerun these tests plus existing preflight tests; confirm pass.**

### Task 5: Simulation acceptance and documentation

**Files:**
- Modify: `scripts/check_auto_mapping_sim.py`
- Modify: `ros2_ws/src/fwr_bringup/test/test_auto_mapping_script.py`
- Modify: `docs/hardware/auto-mapping.md`
- Verify: `scripts/check_fixed_goal_reroute_sim.py`

**Interfaces:**
- Add an explicit reverse-required acceptance mode to the simulator probe: pass only if a front obstacle stops forward motion, reverse safe commands are observed, measured reverse distance is `> 0` and `<= 0.30 m` across the episode, each segment stays `<= 0.10 m` (up to 0.005 m telemetry sampling tolerance), and the robot stops afterward. Keep ordinary front-stop mode separate. Document real reverse remains disabled pending supervised blind-zone and stopping-distance validation.

- [ ] **Step 1: Write failing probe tests.** A run with `reverse_unchecked > 0` but `reverse_commands == 0` fails reverse-required mode; distance 0 or >0.30 m and a segment >0.10 m fail; rear obstacle, stale scan, monitor exit and fault all force zero; a good bounded reverse passes.
- [ ] **Step 2: Run `python3 -m pytest ros2_ws/src/fwr_bringup/test/test_auto_mapping_script.py -q`; confirm expected failure.**
- [ ] **Step 3: Implement the reverse-required result and document commands, safety boundaries, and evidence fields.** Do not label `reversing` state alone as success.
- [ ] **Step 4: Run focused Python tests and `git diff --check`; confirm pass.**
- [ ] **Step 5: Run Gazebo tests without real hardware.** Run the front-obstacle reverse-required scenario; check `reverse_commands > 0`, `0 < reverse_distance <= 0.30`, each segment `<= 0.10 m` plus sampling tolerance, and post-reverse zero. Then run rear obstacle, scan dropout, monitor loss, rapid direction switch, unknown/occupied track and localization-jump scenarios, plus `scripts/check_fixed_goal_reroute_sim.py --obstacle-y=-0.42` for known-clear side rerouting. Record exact commands/logs and failures; if dual-monitor turning prevents rerouting, keep real deployment closed and diagnose.
- [ ] **Step 6: Run the full `ros2_ws/src/fwr_bringup/test` pytest suite and static real launch/preflight checks.** Report passing counts and remaining real-world limitations. Do not arm real motion or test real reverse in this plan.

## Execution handoff

Implement inline with `superpowers:executing-plans` after user review. Preserve all existing local changes, do not commit or push, and keep real hardware untouched during these tasks.
