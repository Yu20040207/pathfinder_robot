from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def parameters(name, node):
    return yaml.safe_load((ROOT / 'config' / name).read_text())[node]['ros__parameters']


def test_real_ekf_uses_wheel_and_gyro_with_one_odom_tf_owner():
    params = parameters('real_ekf.yaml', 'ekf_filter_node')
    assert params['use_sim_time'] is False
    assert (params['map_frame'], params['odom_frame'], params['base_link_frame']) == (
        'map', 'odom', 'base_footprint')
    assert params['world_frame'] == 'odom'
    assert params['publish_tf'] is True
    assert params['odom0'] == '/wheel/odom'
    assert params['imu0'] == '/imu/data_raw'
    assert params['two_d_mode'] is True
    # 航向必须来自 IMU：原地转向时车轮打滑，编码器 yaw 会明显偏小，
    # 用它会带着 SLAM 地图一起被剪切。平移仍用轮式里程计。
    assert params['imu0_config'][5] is True   # absolute yaw from IMU
    assert params['imu0_config'][11] is True  # yaw rate
    assert params['odom0_config'][5] is False
    assert params['odom0_config'][6] is True  # wheel vx for translation
    # 轮距未标定前编码器角速度和 IMU 抢权重会让转弯航向滞后（地图重影），
    # 2026-10-29 起航向完全由 IMU（绝对角 + 角速度）提供。
    assert params['odom0_config'][11] is False  # wheel yaw rate NOT fused


def test_real_slam_owns_map_to_odom_with_real_scan():
    params = parameters('real_slam.yaml', 'slam_toolbox')
    assert params['use_sim_time'] is False
    assert (params['map_frame'], params['odom_frame'], params['base_frame']) == (
        'map', 'odom', 'base_footprint')
    assert params['scan_topic'] == '/scan_mapping'
    assert params['mode'] == 'mapping'


def test_real_mapping_launch_has_no_simulator_or_fake_tf():
    source = (ROOT / 'launch' / 'real_mapping.launch.py').read_text()
    assert 'gazebo' not in source.lower()
    assert 'static_transform_publisher' not in source
    assert "DeclareLaunchArgument('use_sim_time', default_value='false')" in source
    for package in ('fwr_bringup', 'fwr_imu', 'sllidar_ros2',
                    'robot_localization', 'slam_toolbox', 'robot_state_publisher'):
        assert f"package='{package}'" in source


def test_ekf_filtered_output_is_nav_odom():
    source = (ROOT / 'launch' / 'real_mapping.launch.py').read_text()
    assert "('odometry/filtered', '/odom')" in source
