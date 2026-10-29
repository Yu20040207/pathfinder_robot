# Front and Side Scan Mapping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Remove permanent rear self-returns from SLAM and Nav2 obstacle maps while retaining original lidar data for diagnostics and collision monitoring.

**Architecture:** A small ROS 2 node copies `/scan` to `/scan_mapping`, replacing samples in a configurable rear-facing angular sector with NaN without changing scan geometry or timestamps. Real SLAM and Nav2 obstacle layers consume `/scan_mapping`; base readiness and both Collision Monitors retain `/scan`. The physical rear blind zone remains unknown, with real automatic reverse disabled.

**Tech Stack:** ROS 2 Humble, Python/rclpy, `sensor_msgs/LaserScan`, slam_toolbox, Nav2, pytest.

**Spec:** `docs/superpowers/specs/2026-10-22-front-side-scan-mapping-design.md`

## Global Constraints

- Current C1 zero angle points toward the physical rear; verify against live scan before fixing the angle mask.
- Keep raw `/scan` unchanged and available; filtered output is `/scan_mapping`.
- Preserve `header`, scan angular metadata, timing and unmasked values. Use NaN for masked ranges, not `range_max` or `Inf` that could be interpreted as clearing space.
- Set initial rear sector to raw −90° through +90°, then tune its edges only from no-motion front/left/right obstacle observations. Keep the sector configurable in radians.
- SLAM and both Nav2 obstacle layers use `/scan_mapping`; bridge scan freshness, selector and both Collision Monitors continue using `/scan`.
- Unknown rear space stays unknown. Real automatic reverse stays disabled; the future rear ultrasound is outside this plan.
- Preserve all existing local modifications and the current live mapping session until a deliberate stopped-session restart. Do not run real motors, commit or push as part of plan implementation.

## Review Focus

1. Negative angle increment and an angle domain crossing ±π must mask the correct physical rear rays — Task 1 tests.
2. `ranges` and `intensities` can have different lengths or intensities can be empty; only range samples are masked and metadata remains valid — Task 1 tests.
3. Each front/side 60° sector needs at least 50% usable samples; an all-`+Inf` no-return sector remains valid — Task 2 tests and runtime diagnostics.
4. A consumer accidentally left on `/scan` would continue mapping chassis hits — Task 3 static configuration test.
5. Filtering must not accidentally make the rear blind zone appear free or enable reverse — Tasks 3 and 4 acceptance checks.

---

### Task 1: Pure angular mask

**Files:**
- Create: `ros2_ws/src/fwr_bringup/fwr_bringup/scan_mask_core.py`
- Create: `ros2_ws/src/fwr_bringup/test/test_scan_mask_core.py`

**Interfaces:**
- Produce `mask_rear_ranges(ranges: Sequence[float], angle_min: float, angle_increment: float, rear_half_angle: float) -> list[float]`.
- A sample at `angle_min + i * angle_increment`, normalized to [−π, π], is NaN when its absolute angle is at most `rear_half_angle`; otherwise retain its exact value. Reject non-finite angle metadata, zero increment, `rear_half_angle` outside (0, π), or a scan whose declared geometry cannot be interpreted.

- [x] **Step 1: Write failing tests.** Assert rear rays at 0 and both sector edges become NaN; rays at ±π and ±100° remain unchanged; negative increment and normalized wrap work; existing NaN and `Inf` outside the mask remain unchanged; bad metadata raises `ValueError`.
- [x] **Step 2: Run `python3 -m pytest ros2_ws/src/fwr_bringup/test/test_scan_mask_core.py -q`; confirm the missing core fails.**
- [x] **Step 3: Implement the exact interface in `scan_mask_core.py`.** Work from each sample's angle rather than slicing indexes; do not alter unmasked range values.
- [x] **Step 4: Rerun the focused tests; require all pass.**

### Task 2: ROS scan publisher and deployment entry point

**Files:**
- Create: `ros2_ws/src/fwr_bringup/fwr_bringup/scan_mapping_filter.py`
- Create: `ros2_ws/src/fwr_bringup/test/test_scan_mapping_filter.py`
- Modify: `ros2_ws/src/fwr_bringup/setup.py`
- Modify: `ros2_ws/src/fwr_bringup/package.xml`

**Interfaces:**
- Consume Task 1's mask. Subscribe to `/scan` with sensor-data QoS. Publish `sensor_msgs/LaserScan` on `/scan_mapping` with compatible QoS. Declare `rear_half_angle_rad` default `1.5707963267948966`; require at least 50% usable samples in each front-left/front/front-right 60° sector (finite in range or `+Inf` no return), and preserve the input `header`, geometry, timing and intensities.
- Expose console script `scan_mapping_filter = fwr_bringup.scan_mapping_filter:main`.

- [x] **Step 1: Write failing wrapper tests.** Verify topic names, QoS, parameter default, metadata preservation, original message non-mutation, no output for malformed scan, unhealthy state when a front/side sector falls below 50% usable coverage, and healthy state when forward samples are `+Inf` no returns.
- [x] **Step 2: Run `python3 -m pytest ros2_ws/src/fwr_bringup/test/test_scan_mapping_filter.py -q`; confirm expected failures.**
- [x] **Step 3: Implement the node, diagnostics and entry point.** Check three front/side 60° sectors and distinguish valid no-return `+Inf` from invalid NaN/zero. Do not turn an empty front scene into an occupied obstacle; mark scan health separately for preflight.
- [x] **Step 4: Rerun wrapper and core tests; require all pass.**

### Task 3: Real launch and consumer routing

**Files:**
- Modify: `ros2_ws/src/fwr_bringup/launch/real_mapping.launch.py`
- Modify: `ros2_ws/src/fwr_bringup/config/real_slam.yaml`
- Modify: `ros2_ws/src/fwr_bringup/config/real_nav2.yaml`
- Modify: `ros2_ws/src/fwr_bringup/config/real_mapping.rviz`
- Modify: `ros2_ws/src/fwr_bringup/test/test_real_auto_mapping_config.py`

**Interfaces:**
- Mapping launch starts exactly one `scan_mapping_filter`; navigation includes mapping launch and thus inherits it. SLAM `scan_topic` and the local/global `obstacle_layer.scan.topic` keys are `/scan_mapping`. Keep `/scan` in both real Collision Monitor YAMLs and `directional_safety_selector` launch parameters. RViz shows raw `/scan` and `/scan_mapping` with distinct colors.

- [x] **Step 1: Extend the existing static config test so it fails while SLAM and costmaps still read `/scan`.** Also assert there is one filter node, no changed safety scan source, and reverse permissions remain disabled.
- [x] **Step 2: Run `python3 -m pytest ros2_ws/src/fwr_bringup/test/test_real_auto_mapping_config.py -q`; confirm the new assertions fail.**
- [x] **Step 3: Change only the real mapping launch, SLAM/Nav2 YAML and RViz scan displays.** Follow the `ros2-launch` and `ros2-nav2` skill instructions; leave Gazebo scan paths unchanged.
- [x] **Step 4: Rerun static tests, `python3 -m py_compile` for the launch/node and `ros2 launch fwr_bringup real_mapping.launch.py --show-args` after a sourced build; require pass without starting hardware.**

### Task 4: Fail-closed preflight and no-motor acceptance

**Files:**
- Modify: `ros2_ws/src/fwr_bringup/fwr_bringup/auto_mapping_preflight.py`
- Modify: `ros2_ws/src/fwr_bringup/test/test_real_auto_mapping_config.py`
- Modify: `docs/hardware/auto-mapping.md`

**Interfaces:**
- Auto mapping preflight requires `/scan_mapping` to have a recent timestamp, a healthy filter diagnostic, and exactly one expected publisher. Keep existing raw `/scan`, TF, fault, map, costmap, Collision Monitor, footprint and stopping-distance gates. A filter failure or all-masked front scan denies motion authorization.

- [x] **Step 1: Add failing preflight tests for missing/stale filtered scan, unhealthy filter, duplicate publisher and healthy scan; confirm raw scan still cannot substitute for `/scan_mapping`.**
- [x] **Step 2: Run focused preflight tests; confirm expected failures.**
- [x] **Step 3: Implement only the new filtered-scan gate and document topic split, blind-zone limit and saved old map's false-obstacle status.**
- [x] **Step 4: Run focused tests, full `ros2_ws/src/fwr_bringup/test` pytest suite, `colcon build --packages-select fwr_bringup`, and `git diff --check`; report exact results.**
- [x] **Step 5: With motors inactive, test recorded/synthetic scans in an isolated ROS domain.** Check original and filtered timestamps and rates match; static self hits disappear; a box in each of front, left and right remains visible; masked rear is unknown, not marked free.
- [x] **Step 6: Only after the no-motor checks pass, plan a deliberate mapping-session restart with the user.** Keep automatic motion closed. The later supervised keyboard loop must save a new map and compare interior false points against the old map before any auto-exploration acceptance.

## Execution handoff

Implement inline in this task with `superpowers:executing-plans` after user review. Preserve the uncommitted workspace, running RViz and mapping session until a deliberate stop/restart; do not commit, push or operate motors.

## Execution record

- 2026-10-22: Tasks 1–3 and Task 4 software/preflight work implemented. Focused tests were observed failing before implementation and passing afterward. Real static `/scan` inspection: 124 returns inside chassis outline before mask, 0 after mask; 292 other finite returns retained. ROS isolated-domain scan test preserved timestamp and masked rear. Whole package test: 162 passed; build passed.
- Ruling: require at least 50% usable samples in each front-left/front/front-right 60° sector instead of one remaining sample. Review identified the one-sample case as a weak health indication; real stationary C1 data had full coverage in each sector. Cost if wrong: some unusual but usable scans may stop automatic motion until the threshold is tuned.
- 2026-10-23: Added regression for the one-sample failure (RED→GREEN), updated spec and docs, and reran the full package test: 163 passed. Rebuilt package, restarted mapping launch with motion gate closed, and reopened RViz. Runtime health is true, `/base/fault=0`, SLAM subscribes `/scan_mapping`. Await supervised front/left/right obstacle and short keyboard-map validation; no real motor command was sent during this implementation.
- 2026-10-23: Physical front/left/right cardboard placement was checked without motor commands. Filter retained the same nearest distance as raw scan: 0.562 m front, 0.655 m left, 0.608 m right. Rear chassis returns were absent from filtered scan; health true and fault 0. Only supervised keyboard mapping and saved-map visual comparison remain.
- 2026-10-23: User completed supervised short keyboard mapping and reported no dotted black trail in RViz. Teleop exited; both bridge motion/manual gates are false, fault 0, filter healthy. Saved `real_front_side_map_2026-10-23.yaml` and PGM, 277×478 at 0.05 m/cell. This completes the filtering/keyboard-mapping acceptance; automatic navigation and reverse are separate unverified gates.
