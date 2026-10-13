"""Start the lightweight 2WD chassis and front C1 in a Gazebo corridor."""

import os

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('fwr_bringup')
    world = os.path.join(share, 'worlds', 'corridor_c1.sdf')
    bridge = os.path.join(share, 'config', 'corridor_bridge.yaml')
    slam_params = os.path.join(share, 'config', 'corridor_slam.yaml')
    rviz_config = os.path.join(share, 'config', 'corridor_scan.rviz')
    nav_rviz_config = os.path.join(share, 'config', 'corridor_navigation.rviz')
    nav2_params = os.path.join(share, 'config', 'corridor_nav2.yaml')
    nav2_params_with_backup = RewrittenYaml(
        source_file=nav2_params,
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': os.path.join(
                share, 'behavior_trees', 'navigate_to_pose_with_backup.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(
                share, 'behavior_trees', 'navigate_through_poses_with_backup.xml'),
        },
        convert_types=False,
    )
    nav_map = os.path.join(share, 'maps', 'corridor_nav.yaml')
    # Resolve model://fwr_bringup/... from the installed package on x86 or Jetson.
    resource_path = os.pathsep.join(filter(None, [
        os.path.dirname(share), os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]))
    gazebo_env = {
        'IGN_RENDER_ENGINE': 'ogre2',
        'IGN_GAZEBO_RESOURCE_PATH': resource_path,
    }
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('mapping', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('navigation', default_value='false'),
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('headless')),
            additional_env=gazebo_env,
        ),
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-s', '-r', world],
            output='screen',
            condition=IfCondition(LaunchConfiguration('headless')),
            additional_env=gazebo_env,
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='corridor_bridge',
            parameters=[{'config_file': bridge, 'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['--x', '0.19', '--y', '0', '--z', '0.2298',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser_front'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', parameters=[slam_params],
            condition=IfCondition(LaunchConfiguration('mapping')),
            output='screen',
        ),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            parameters=[{'yaml_filename': nav_map, 'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('navigation')),
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                         'node_names': ['map_server']}],
            condition=IfCondition(LaunchConfiguration('navigation')),
            output='screen',
        ),
        Node(
            package='fwr_bringup', executable='gazebo_ground_truth_localizer',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('navigation')),
            output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('rviz'), "' == 'true' and '",
                LaunchConfiguration('navigation'), "' == 'false'",
            ])),
            output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', nav_rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('rviz'), "' == 'true' and '",
                LaunchConfiguration('navigation'), "' == 'true'",
            ])),
            output='screen',
        ),
        TimerAction(
            period=5.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_with_backup,
                    'autostart': 'True',
                    'use_composition': 'False',
                }.items(),
            )],
            condition=IfCondition(LaunchConfiguration('navigation')),
        ),
    ])
