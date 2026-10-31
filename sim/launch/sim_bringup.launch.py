"""四房间仿真：Gazebo Fortress + 我们的建图/定位栈。

与实机 launch 的差别只在于「数据来源」：仿真里传感器来自 Gazebo，
其余（scan_mapping_filter 的 260° 遮罩、EKF、Cartographer、rear_guard…）全部复用实机同一套代码。

用法：
    export DISPLAY=:1
    ros2 launch sim/launch/sim_bringup.launch.py            # 建图
    ros2 launch sim/launch/sim_bringup.launch.py nav:=true  # 建图 + Nav2 + 房间探索
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
SIM = os.path.join(REPO, 'sim')


def generate_launch_description() -> LaunchDescription:
    fwr = get_package_share_directory('fwr_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav = LaunchConfiguration('nav')
    rviz = LaunchConfiguration('start_rviz')

    world = os.path.join(SIM, 'worlds', 'four_rooms.sdf')
    model = os.path.join(SIM, 'models', 'fwr_bot', 'model.sdf')
    bridge_cfg = os.path.join(SIM, 'config', 'bridge.yaml')

    # 软件渲染：这台机器 EGL 建不出 dri2 screen，GPU 传感器会静默不出数据
    software_gl = [
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
    ]

    gz = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world],
        output='screen',
        name='gazebo')

    spawn = TimerAction(period=12.0, actions=[ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-name', 'fwr_bot', '-file', model,
             '-x', '-3.5', '-y', '-3.5', '-z', '0.12'],
        output='screen',
        name='spawn_robot')])

    bridge = TimerAction(period=10.0, actions=[Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}],
        output='screen')])

    # ---- 静态 TF（与实机一致：laser_front 在 x=+0.21 且倒装 180°）
    tf_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_laser_front',
        arguments=['0.21', '0.0', '0.25', str(math.pi), '0.0', '0.0',
                   'base_footprint', 'laser_front'],
        parameters=[{'use_sim_time': use_sim_time}])

    tf_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}])

    # 仿真 IMU 的 frame 名不受 gz_frame_id 控制时兜底
    tf_imu = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_imu_link',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_footprint', 'imu_link'],
        parameters=[{'use_sim_time': use_sim_time}])

    tf_us = []
    for name, y, yaw in (('left_out', '0.105', str(math.radians(-10.0))),
                         ('left_in', '0.05', '0.0'),
                         ('right_in', '-0.05', '0.0'),
                         ('right_out', '-0.105', str(math.radians(10.0)))):
        tf_us.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'tf_us_{name}',
            arguments=['-0.26', y, '0.14', '0.0', '0.0', yaw,
                       'base_footprint', f'ultrasonic_rear_{name}'],
            parameters=[{'use_sim_time': use_sim_time}]))

    # ---- 260° 建图扫描（和实机同一个节点、同一个参数）
    scan_filter = Node(
        package='fwr_bringup', executable='scan_mapping_filter',
        name='scan_mapping_filter',
        parameters=[{'use_sim_time': use_sim_time,
                     'rear_half_angle_rad': math.radians(50.0)}],
        remappings=[('scan_in', '/scan'), ('scan_out', '/scan_mapping')])

    # ---- 4 路后向超声（LaserScan → Range，接口与实机一致）
    us_sim = Node(
        package='fwr_bringup', executable='ultrasonic_sim',
        name='ultrasonic_sim',
        parameters=[{'use_sim_time': use_sim_time}])

    # ---- EKF：吃仿真的 /wheel/odom + /imu/data_raw（与实机同一条路径）
    ekf = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(fwr, 'config', 'real_ekf.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odom')])

    # ---- Cartographer 2D（同一份 lua）
    carto = Node(
        package='cartographer_ros', executable='cartographer_node',
        name='cartographer_node',
        arguments=['-configuration_directory', os.path.join(SIM, 'config'),
                   '-configuration_basename', 'sim_2d.lua'],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('scan', '/scan_mapping'),
                    ('odom', '/odom'),
                    ('imu', '/imu/data_raw')])

    carto_grid = Node(
        package='cartographer_ros', executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[{'use_sim_time': use_sim_time, 'resolution': 0.05}])

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(fwr, 'config', 'real_localization.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        *software_gl,
        gz,
        spawn,
        bridge,
        tf_laser, tf_base, tf_imu, *tf_us,
        scan_filter, us_sim, ekf, carto, carto_grid,
        rviz_node,
    ])
