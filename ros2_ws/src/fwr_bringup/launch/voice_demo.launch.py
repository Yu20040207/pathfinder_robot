"""Text/WAV-driven voice simulation; no motor commands."""

from pathlib import Path
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("fwr_bringup")) / "config/voice.yaml"
    share = Path(get_package_share_directory("fwr_bringup"))
    candidate = next((parent / ".venv/bin/python" for parent in share.parents
                      if (parent / ".venv/bin/python").is_file()), None)
    voice_python = os.environ.get("FWR_PYTHON", str(candidate or sys.executable))
    return LaunchDescription([
        DeclareLaunchArgument("voice_python", default_value=voice_python),
        Node(package="fwr_voice", executable="voice_demo_executor", output="screen"),
        Node(package="fwr_voice", executable="voice_node", output="screen",
             prefix=LaunchConfiguration("voice_python"),
             parameters=[str(config), {"simulation": True}]),
    ])
