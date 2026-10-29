-- Cartographer 2D 建图配置（四轮机器人二代）
-- 目标：室内走廊场景比 slam_toolbox 更稳（子图 + 位姿图 + 在线相关匹配）。
-- 依赖：sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  -- 有 IMU 时 tracking_frame 必须是 IMU 自身坐标系（cartographer 要求）
  tracking_frame = "imu_link",
  -- 我们自己有 EKF 发布的 odom→base_footprint，所以由 cartographer 只补 map→odom
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  -- 2026-10-29 实测：IMU 绝对航向与轮式航向都有约 2% 尺度误差（转 360° 丢 5-7°），
  -- 把它喂给 cartographer 会污染 map→odom（实测累积到 43°）。改为只用雷达 + 陀螺：
  -- 扫描匹配持续修正航向，不依赖带尺度误差的绝对航向。
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- 走廊关键项：开启在线相关匹配（分支限界），平行墙场景比纯优化匹配稳
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 运动滤波：位移/转角达到阈值才处理，抑制重复帧
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.15
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.3)

-- 子图与回环
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

POSE_GRAPH.optimize_every_n_nodes = 30
POSE_GRAPH.constraint_builder.min_score = 0.6
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.max_num_final_iterations = 200

return options
