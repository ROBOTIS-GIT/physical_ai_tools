#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Seongwoo Kim

"""Launch file for Physical AI Behavior Tree node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Setup launch nodes with access to launch configurations."""

    # Get parameter values at runtime
    robot_type = LaunchConfiguration('robot_type').perform(context)

    # Get physical_ai_server config directory
    physical_ai_server_pkg_share = get_package_share_directory('physical_ai_server')
    config_dir = os.path.join(physical_ai_server_pkg_share, 'config')

    # Build config path based on robot_type
    robot_config_path = os.path.join(config_dir, f'{robot_type}_config.yaml')

    # Check if config exists, fallback to omx_f if not found
    if not os.path.exists(robot_config_path):
        print(f"Warning: Config file not found: {robot_config_path}")
        print(f"Falling back to omx_f_config.yaml")
        robot_config_path = os.path.join(config_dir, 'omx_f_config.yaml')

    # Create behavior tree node
    bt_node = Node(
        package='physical_ai_bt',
        executable='bt_node',
        name='bt_node',
        output='screen',
        parameters=[
            robot_config_path,
            {
                'robot_type': LaunchConfiguration('robot_type'),
                'tree_xml': LaunchConfiguration('tree_xml'),
                'tick_rate': LaunchConfiguration('tick_rate'),
                'inference_timeout': LaunchConfiguration('inference_timeout'),
                'rule_timeout': LaunchConfiguration('rule_timeout'),
                'position_threshold': LaunchConfiguration('position_threshold'),
                'current_positions': LaunchConfiguration('current_positions'),
                'target_positions': LaunchConfiguration('target_positions'),
            }
        ]
    )

    return [bt_node]


def generate_launch_description():
    """Generate launch description for behavior tree node."""

    # Declare launch arguments
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='omx_f',
        description='Type of robot (e.g., omx_f)'
    )

    tree_xml_arg = DeclareLaunchArgument(
        'tree_xml',
        default_value='omx_test.xml',
        description='Name of XML file in physical_ai_bt/trees/ directory'
    )

    tick_rate_arg = DeclareLaunchArgument(
        'tick_rate',
        default_value='10.0',
        description='Behavior tree tick rate in Hz'
    )

    inference_timeout_arg = DeclareLaunchArgument(
        'inference_timeout',
        default_value='5.0',
        description='Inference monitoring duration in seconds'
    )

    rule_timeout_arg = DeclareLaunchArgument(
        'rule_timeout',
        default_value='15.0',
        description='Rule-based action timeout in seconds'
    )

    position_threshold_arg = DeclareLaunchArgument(
        'position_threshold',
        default_value='0.1',
        description='Position error threshold in radians'
    )

    current_positions_arg = DeclareLaunchArgument(
        'current_positions',
        default_value='[0.0, -0.378752, -0.151795, 1.541355, 0.006874, 0.73828]',
        description='Starting joint positions'
    )

    target_positions_arg = DeclareLaunchArgument(
        'target_positions',
        default_value='[-1.5, -0.378752, -0.151795, 1.541355, 0.006874, 0.73828]',
        description='Target joint positions'
    )

    return LaunchDescription([
        robot_type_arg,
        tree_xml_arg,
        tick_rate_arg,
        inference_timeout_arg,
        rule_timeout_arg,
        position_threshold_arg,
        current_positions_arg,
        target_positions_arg,
        OpaqueFunction(function=launch_setup)
    ])
