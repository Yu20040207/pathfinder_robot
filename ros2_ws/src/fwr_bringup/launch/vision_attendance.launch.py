"""Bring up HP60C, face recognition and attendance with stable camera topics."""

import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def camera_remappings(index: int = 0):
    return [
        (f"~/rgb{index}/image", "/robot/camera/color/image_raw"),
        (f"~/rgb{index}/camera_info", "/robot/camera/color/camera_info"),
        (f"~/depth{index}/image_raw", "/robot/camera/depth/image_raw"),
        (f"~/depth{index}/camera_info", "/robot/camera/depth/camera_info"),
    ]


def vision_python_default() -> str:
    explicit = os.environ.get("FWR_PYTHON")
    if explicit:
        return explicit
    share = Path(get_package_share_directory("fwr_bringup"))
    for parent in share.parents:
        candidate = parent / ".venv/bin/python"
        if candidate.is_file():
            return str(candidate)
    return sys.executable


def _camera_node(context, config):
    index = int(LaunchConfiguration("camera_index").perform(context))
    if index < 0:
        raise ValueError("camera_index must be nonnegative")
    return [Node(
        package="ascamera", executable="ascamera_node", namespace="ascamera_hp60c",
        name="camera_publisher", output="screen",
        condition=IfCondition(LaunchConfiguration("use_camera_driver")),
        parameters=[str(config / "hp60c.yaml"), {"confiPath": LaunchConfiguration("camera_config_dir")}],
        remappings=camera_remappings(index),
    )]


def generate_launch_description():
    config = Path(get_package_share_directory("fwr_bringup")) / "config"
    data_home = Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local/share"))
    default_db = str(data_home / "four_wheel_robot/attendance.sqlite3")
    return LaunchDescription([
        DeclareLaunchArgument("use_camera_driver", default_value="true"),
        DeclareLaunchArgument("camera_config_dir", default_value=os.environ.get("FWR_CAMERA_CONFIG_DIR", "")),
        DeclareLaunchArgument("camera_index", default_value="0"),
        DeclareLaunchArgument("color_topic", default_value="/robot/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/robot/camera/depth/image_raw"),
        DeclareLaunchArgument("vision_python", default_value=vision_python_default()),
        DeclareLaunchArgument("db_path", default_value=os.environ.get("FWR_DB_PATH", default_db)),
        OpaqueFunction(function=_camera_node, args=[config]),
        Node(package="fwr_vision", executable="face_node", name="face_node",
             output="screen", prefix=LaunchConfiguration("vision_python"),
             parameters=[str(config / "vision.yaml"), {
                 "db_path": LaunchConfiguration("db_path"),
                 "color_topic": LaunchConfiguration("color_topic"),
                 "depth_topic": LaunchConfiguration("depth_topic"),
             }]),
        Node(package="fwr_attendance", executable="attendance_node", name="attendance_node",
             output="screen", parameters=[str(config / "attendance.yaml"),
                                          {"db_path": LaunchConfiguration("db_path")}]),
    ])
