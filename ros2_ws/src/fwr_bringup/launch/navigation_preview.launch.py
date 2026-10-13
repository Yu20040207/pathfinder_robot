"""Run only the safe map-to-path preview node."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("fwr_bringup")) / "config/navigation_preview.yaml"
    return LaunchDescription([
        Node(package="fwr_navigation", executable="path_preview_node",
             name="path_preview_node", output="screen", parameters=[str(config)]),
    ])
