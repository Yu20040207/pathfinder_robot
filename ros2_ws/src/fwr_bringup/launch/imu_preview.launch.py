"""Show the real IMU's relative attitude by rotating the CAD robot in RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('fwr_bringup'))
    description = (share / 'description' / 'fwr_chassis_demo.urdf').read_text()
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='imu_preview_robot_state_publisher', output='screen',
             parameters=[{'robot_description': description, 'use_sim_time': use_sim_time}]),
        Node(package='fwr_bringup', executable='cad_preview_joint_states',
             output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='fwr_imu', executable='dm_imu_node', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='fwr_imu', executable='imu_preview_tf', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='rviz2', executable='rviz2', name='imu_preview_rviz',
             output='screen', arguments=['-d', str(share / 'config' / 'imu_preview.rviz')],
             parameters=[{'use_sim_time': use_sim_time}]),
    ])
