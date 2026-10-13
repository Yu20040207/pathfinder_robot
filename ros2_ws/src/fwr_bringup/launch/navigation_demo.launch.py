"""Run a self-contained /demo path-planning and motion simulation."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="fwr_navigation", executable="path_preview_node",
             name="demo_path_preview_node", output="screen", parameters=[{
                 "map_topic": "/demo/map",
                 "pose_topic": "/demo/pose",
                 "goal_topic": "/demo/goal",
                 "path_topic": "/demo/path",
                 "clearance_m": 0.05,
             }]),
        Node(package="fwr_navigation", executable="navigation_demo_node",
             name="navigation_demo_node", output="screen"),
    ])
