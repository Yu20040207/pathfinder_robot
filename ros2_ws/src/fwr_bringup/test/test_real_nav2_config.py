from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def params(node):
    data = yaml.safe_load((ROOT / 'config' / 'real_nav2.yaml').read_text())
    return data[node]['ros__parameters']


def test_real_frames_speed_and_recovery_policy():
    controller = params('controller_server')
    shim = controller['FollowPath']
    assert controller['use_sim_time'] is False
    # 差速底盘允许原地旋转：Nav2 起步对准航向和小半径转弯都靠它。
    # 倒车仍关闭，旋转命令还要经方向选择器两路碰撞监控放行。
    assert shim['plugin'] == 'nav2_rotation_shim_controller::RotationShimController'
    assert shim['primary_controller'] == (
        'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
    assert shim['rotate_to_heading_angular_vel'] <= 1.5
    assert shim['rotate_to_goal_heading'] is False
    assert shim['desired_linear_vel'] <= 0.20
    assert shim['allow_reversing'] is False
    # 航向对准交给 Rotation Shim；RPP 再开 rotate_to_heading 会和它形成极限环
    assert shim['use_rotate_to_heading'] is False
    assert params('velocity_smoother')['max_velocity'][2] <= 1.5
    assert params('bt_navigator')['robot_base_frame'] == 'base_footprint'
    assert params('bt_navigator')['global_frame'] == 'map'
    assert params('behavior_server')['behavior_plugins'] == ['wait', 'spin']
    assert params('behavior_server')['max_rotational_vel'] <= 1.5
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
        assert costmap['obstacle_layer']['scan']['topic'] == '/scan_mapping'
        assert costmap['footprint']


def test_real_launch_keeps_motion_gated_and_one_slam_owner():
    source = (ROOT / 'launch' / 'real_navigation.launch.py').read_text()
    assert 'navigation_launch.py' in source
    assert 'bringup_launch.py' not in source
    assert 'real_mapping.launch.py' in source
    assert "'motion_enabled': False" in (ROOT / 'launch' / 'real_mapping.launch.py').read_text()
    assert 'gazebo' not in source.lower()
    assert "DeclareLaunchArgument('use_sim_time', default_value='false')" in source


def test_real_navigation_can_use_saved_map_localization_instead_of_slam():
    source = (ROOT / 'launch' / 'real_navigation.launch.py').read_text()
    assert "DeclareLaunchArgument('use_saved_map', default_value='false')" in source
    assert 'real_localization.launch.py' in source
    assert 'UnlessCondition' in source
    assert 'IfCondition' in source
    assert "'map': LaunchConfiguration('map')" in source


def test_real_behavior_trees_recover_without_reverse():
    """Card recovery turns in place; reversing stays off until rear sensing is validated."""
    for name in ('real_navigate_to_pose.xml', 'real_navigate_through_poses.xml'):
        tree = (ROOT / 'behavior_trees' / name).read_text()
        assert '<BackUp' not in tree
        assert '<FollowPath' in tree
        assert '<RecoveryNode' in tree
        assert '<Spin' in tree
        # 路径保持：只有路径失效/目标更新时才重规划，避免左右绕行路线抖动。
        # 单次恢复转向必须有界（60° 左右），不能整圈快转。
        import re
        spin_dist = float(re.search(r'<Spin spin_dist="([0-9.]+)"', tree).group(1))
        assert spin_dist <= 1.6


def test_real_rviz_uses_guarded_two_point_panel_without_direct_goal_tool():
    rviz = (ROOT / 'config' / 'real_localization.rviz').read_text()
    assert 'fwr_rviz_nav/RealTwoPointNavPanel' in rviz
    assert 'rviz_default_plugins/PublishPoint' in rviz
    assert '/two_point_preview_path' in rviz
    assert '/two_point_markers' in rviz
    assert 'nav2_rviz_plugins/GoalTool' not in rviz
