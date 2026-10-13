"""Voice-driven, kinematic chassis preview in RViz (no motor output)."""

import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("fwr_bringup"))
    candidate = next((parent / ".venv/bin/python" for parent in share.parents
                      if (parent / ".venv/bin/python").is_file()), None)
    voice_python = os.environ.get("FWR_PYTHON", str(candidate or sys.executable))
    robot_description = (share / "description/fwr_chassis_demo.urdf").read_text()
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    microphone_enabled = ParameterValue(LaunchConfiguration("microphone_enabled"), value_type=bool)

    return LaunchDescription([
        # The kinematic preview uses wall time and does not publish /clock.
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("microphone_enabled", default_value="true"),
        DeclareLaunchArgument("show_rviz", default_value="true"),
        DeclareLaunchArgument("voice_python", default_value=voice_python),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[{"robot_description": robot_description,
                                            "use_sim_time": use_sim_time}]),
        Node(package="fwr_navigation", executable="voice_chassis_demo_node",
             output="screen", parameters=[{"use_sim_time": use_sim_time}]),
        Node(package="fwr_voice", executable="voice_demo_executor", output="screen"),
        Node(package="fwr_voice", executable="voice_node", output="screen",
             prefix=LaunchConfiguration("voice_python"),
             parameters=[str(share / "config/voice.yaml"),
                         {"simulation": True, "microphone_enabled": microphone_enabled}]),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", str(share / "config/voice_chassis_demo.rviz")],
             condition=IfCondition(LaunchConfiguration("show_rviz"))),
    ])
