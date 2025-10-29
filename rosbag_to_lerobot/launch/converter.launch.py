#!/usr/bin/env python3

"""
Launch file for the rosbag to LeRobot dataset converter.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """Generate launch description."""

    # Config file argument (required)
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('rosbag_to_lerobot'), 'config', 'ffw_arm_only_config.yaml'
        ]),
        description='Path to YAML config file (required)'
    )

    # Create the node
    converter_node = Node(
        package='rosbag_to_lerobot',
        executable='rosbag_to_lerobot_converter',
        name='rosbag_to_lerobot_converter',
        parameters=[
            # Load config file (required)
            LaunchConfiguration('config_file'),
            # Also pass the path explicitly so the node can read nested mappings
            { 'config_yaml_path': LaunchConfiguration('config_file') },
        ],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        converter_node,
    ])
