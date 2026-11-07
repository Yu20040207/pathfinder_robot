"""One-command automatic mapping mission (new file, no manual/nav changes).

Composition only:
  * the existing real_auto_mapping.launch.py, with the already-validated
    viewpoint-planner configuration;
  * a new auto_mapping_mission finalizer that closes the gate and saves the
    SLAM grid + posegraph when the viewpoint planner reports completion.

It does not edit or override manual-mapping or navigation parameters.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('fwr_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    map_dir = LaunchConfiguration('map_dir')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument(
            'map_dir', default_value=os.path.expanduser('~/robot_maps'),
            description='Directory for the automatic final map'),
        DeclareLaunchArgument(
            'max_runtime_s', default_value='0.0',
            description='Hard motion time cap in seconds; 0 disables. '
                        'Use 60-120 for a supervised small-area trial.'),
        DeclareLaunchArgument(
            'max_goal_distance_m', default_value='0.0',
            description='Optional hard radius around the start pose; 0 disables.'),
        DeclareLaunchArgument(
            'room_aware', default_value='false',
            description='Adaptive door-width room split; use true for a single-room run.'),
        DeclareLaunchArgument(
            'door_clearance_m', default_value='0.50',
            description='Half doorway width used to split rooms.'),
        DeclareLaunchArgument(
            'chain_watchdog', default_value='false',
            description='Leave false when lifecycle_manager_collision owns the monitor.'),
        DeclareLaunchArgument('room_min_cells', default_value='1200',
            description='Wait for this many room cells before enabling room filter.'),
        # Child nodes inherit this; slam_map_save uses it for the final files.
        SetEnvironmentVariable('FWR_MAP_DIR', map_dir),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / 'launch' / 'real_auto_mapping.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'start_rviz': start_rviz,
                'slam_engine': 'slam',
                # The 2026-11-04 real-robot validated viewpoint configuration.
                'use_explorer': 'false',
                'viewpoint_planner': 'true',
                'viewpoint_enabled': 'true',
                'mapping_bt': 'true',
                'stuck_reverser': 'false',
                'kickstart': 'false',
                'kick_reverse': 'false',
                'stuck_guard': 'true',
                'chain_watchdog': LaunchConfiguration('chain_watchdog'),
                'max_goal_distance_m': LaunchConfiguration('max_goal_distance_m'),
                'room_aware': LaunchConfiguration('room_aware'),
                'door_clearance_m': LaunchConfiguration('door_clearance_m'),
                'room_min_cells': LaunchConfiguration('room_min_cells'),
            }.items()),
        Node(package='fwr_bringup', executable='auto_mapping_mission',
             name='auto_mapping_mission', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'completion_hold_s': 3.0,
                          'save_on_done': True,
                          'close_gate_on_done': True,
                          'max_runtime_s': ParameterValue(LaunchConfiguration('max_runtime_s'), value_type=float)}]),
    ])
