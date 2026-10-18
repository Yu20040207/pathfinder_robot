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
    assert params['imu0_config'][11] is True  # yaw rate, not drifting absolute yaw
    assert params['imu0_config'][5] is False


def test_real_slam_owns_map_to_odom_with_real_scan():
    params = parameters('real_slam.yaml', 'slam_toolbox')
    assert params['use_sim_time'] is False
    assert (params['map_frame'], params['odom_frame'], params['base_frame']) == (
        'map', 'odom', 'base_footprint')
    assert params['scan_topic'] == '/scan'
    assert params['mode'] == 'mapping'


def test_real_mapping_launch_has_no_simulator_or_fake_tf():
    source = (ROOT / 'launch' / 'real_mapping.launch.py').read_text()
    assert 'gazebo' not in source.lower()
    assert 'static_transform_publisher' not in source
    assert "DeclareLaunchArgument('use_sim_time', default_value='false')" in source
    for package in ('fwr_bringup', 'fwr_imu', 'sllidar_ros2',
                    'robot_localization', 'slam_toolbox', 'robot_state_publisher'):
        assert f"package='{package}'" in source
