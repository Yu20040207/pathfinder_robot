# 参数快照 v1.8（2026-11-12 17:30）

实机：N305（manbaout@192.168.0.20），底盘 WSDC2412D-V3 + 100:1 电机 + 100 mm 轮。
本文件记录这一刻**实际运行**的关键参数，供回退与对比用。

## 车体与规划几何
| 参数 | 值 |
| --- | --- |
| 车壳尺寸（实测） | 0.60 × **0.36** m（长×宽） |
| costmap `footprint` | `[[0.30,0.18],[0.30,-0.18],[-0.30,-0.18],[-0.30,0.18]]` |
| `footprint_padding` | **0.06**（有效包络 0.48 × 0.60 m）|
| 内切半径 | 0.18 m |
| `inflation_radius`（全局/局部）| 0.35 m |
| `cost_scaling_factor` | 1.2 |

## 控制器（FollowPath = RPP）
| 参数 | 值 |
| --- | --- |
| `desired_linear_vel` | 0.20 m/s |
| `lookahead_dist` / min / max | **0.52** / 0.40 / 0.85 |
| `use_velocity_scaled_lookahead_dist` | true |
| `angular_dist_threshold` | 0.6 rad |
| `rotate_to_heading_angular_vel` | 0.55 rad/s |
| `max_angular_accel` | 4.0 |
| `transform_tolerance` | **0.5** |
| `use_regulated_linear_velocity_scaling` / `use_cost_regulated_...` | false / false |
| `xy_goal_tolerance` | 0.4 m |

## behavior_server / BT
| 参数 | 值 |
| --- | --- |
| `transform_tolerance` | **0.5** |
| `simulate_ahead_time` | 0.5 |
| BT `RateController` | 0.5 Hz |
| `bt_loop_duration` | 50 ms |

## 速度平滑器
| 参数 | 值 |
| --- | --- |
| `max_velocity` | [0.20, 0.0, 1.0] |
| `max_accel` | [0.35, 0.0, 4.0] |

## 底盘 base_bridge
| 参数 | 值 |
| --- | --- |
| `wheel_speed_scale` | 1.0 |
| `manual_enabled` / `motion_enabled`（空闲时）| false / false |
| `manual_max_speed_mps` | 0.30 |
| `manual_cmd_hold_s` | 1.00 s |
| `manual_requires_scan` | false |
| `manual_rear_guard_enabled` | false |
| `rear_guard_enabled` | true（自动链路） |
| `sensor_timeout_s` | 0.5 |
| `allow_pivot` | **true** |
| `max_reverse_mps` | 0.12 |
| 串口保活 | 状态静默 2 s 重新握手 / 6 s 重开串口 |
| 里程计标定 | `track_m` 0.317、`circumference_m` 0.314159、`counts_per_rev` 6400 |

## 探索器 frontier_explorer（导航侧）
| 参数 | 值 |
| --- | --- |
| `stuck_cancel_s` | 2.5 |
| `max_stuck_retries` | 2 |
| `fixed_route_tail_gap_m` | 2.5 |
| `route_skip_near_m` | 0.6（仅剔除"第一个贴在车身上的点"）|
| `min_fixed_goal_distance_m` | **0.5**（单点近目标拒绝）|
| `costmap_clear_period_s` | 0.0（任务结束清一次）|
| `prealign_enabled` | **false**（掉头选边暂不启用）|

## 手动模式（RViz 面板）
| 项 | 值 |
| --- | --- |
| 速度档位 | 0.12 / 0.15 / 0.18 / 0.20 / 0.25 / 0.30 m/s（默认 0.12）|
| 转向角速度 | 0.5 rad/s（固定）|
| 键盘 | W/S/A/D + 方向键；X11 物理按键查询（RViz 截断长按）|
| 发布 | 独立线程 20 Hz |
| 门控保活 | 手动模式每 1 s 置 motion_enabled/manual_enabled=true |

## 固件（未被本次提交修改）
| 常量 | 值 |
| --- | --- |
| `MAX_DUTY` / `BASE_DUTY` / `DUTY_RAMP_PER_S` | 0.95 / 0.60 / 0.5 |
| `FF_BASE_DUTY` / `FF_PER_MMPS` | 0.02 / 0.0024 |
| `STARTUP_PULSE_DUTY` / `LOW_SPEED_FLOOR_DUTY` | 0.12 / 0.15 |
| 台架实测 | duty 0.30→117、0.45→180、0.60→242、0.65→264 mm/s；闭环 100/200/300 全跟踪 |
