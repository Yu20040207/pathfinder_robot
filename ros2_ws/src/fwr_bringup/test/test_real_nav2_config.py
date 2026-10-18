from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def params(node):
    data = yaml.safe_load((ROOT / 'config' / 'real_nav2.yaml').read_text())
    return data[node]['ros__parameters']


def test_real_frames_speed_and_recovery_policy():
    controller = params('controller_server')
    rpp = controller['FollowPath']
    assert controller['use_sim_time'] is False
    assert rpp['desired_linear_vel'] <= 0.08
    assert rpp['allow_reversing'] is False
    assert rpp['use_rotate_to_heading'] is False
    assert params('bt_navigator')['robot_base_frame'] == 'base_footprint'
    assert params('bt_navigator')['global_frame'] == 'map'
    assert params('behavior_server')['behavior_plugins'] == ['wait']
    assert params('velocity_smoother')['min_velocity'][0] == 0.0


def test_costmaps_use_real_lidar_and_no_voxel():
    for name, frame in [('local_costmap', 'odom'), ('global_costmap', 'map')]:
        data = yaml.safe_load((ROOT / 'config' / 'real_nav2.yaml').read_text())
        costmap = data[name][name]['ros__parameters']
        assert costmap['use_sim_time'] is False
        assert costmap['robot_base_frame'] == 'base_footprint'
        assert costmap['global_frame'] == frame
        assert 'obstacle_layer' in costmap['plugins']
        assert 'inflation_layer' in costmap['plugins']
        assert 'voxel_layer' not in costmap['plugins']
        assert costmap['obstacle_layer']['scan']['topic'] == '/scan'
        assert costmap['footprint']


def test_real_launch_keeps_motion_gated_and_one_slam_owner():
    source = (ROOT / 'launch' / 'real_navigation.launch.py').read_text()
    assert 'navigation_launch.py' in source
    assert 'bringup_launch.py' not in source
    assert 'real_mapping.launch.py' in source
    assert "'motion_enabled': False" in (ROOT / 'launch' / 'real_mapping.launch.py').read_text()
    assert 'gazebo' not in source.lower()
    assert "DeclareLaunchArgument('use_sim_time', default_value='false')" in source


def test_real_behavior_trees_cannot_reverse_or_spin():
    for name in ('real_navigate_to_pose.xml', 'real_navigate_through_poses.xml'):
        tree = (ROOT / 'behavior_trees' / name).read_text()
        assert '<BackUp' not in tree
        assert '<Spin' not in tree
        assert '<FollowPath' in tree
