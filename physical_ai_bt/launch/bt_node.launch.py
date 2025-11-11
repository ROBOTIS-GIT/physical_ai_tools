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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for BT node."""

    # Get config file path
    physical_ai_server_share = get_package_share_directory('physical_ai_server')
    config_file = os.path.join(
        physical_ai_server_share,
        'config',
        'omx_f_config.yaml'
    )

    # Declare launch arguments
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='omx_f',
        description='Robot type (e.g., omx_f, omx_s)'
    )

    inference_timeout_arg = DeclareLaunchArgument(
        'inference_timeout',
        default_value='5.0',
        description='Timeout in seconds for inference action'
    )

    rule_timeout_arg = DeclareLaunchArgument(
        'rule_timeout',
        default_value='15.0',
        description='Timeout in seconds for rule-based action'
    )

    tick_rate_arg = DeclareLaunchArgument(
        'tick_rate',
        default_value='10.0',
        description='Behavior tree tick rate in Hz'
    )

    # Create BT node
    bt_node = Node(
        package='physical_ai_bt',
        executable='physical_ai_bt',
        name='physical_ai_bt_node',
        output='screen',
        parameters=[
            config_file,
            {
                'robot_type': LaunchConfiguration('robot_type'),
                'inference_timeout': LaunchConfiguration('inference_timeout'),
                'rule_timeout': LaunchConfiguration('rule_timeout'),
                'tick_rate': LaunchConfiguration('tick_rate'),
            }
        ]
    )

    return LaunchDescription([
        robot_type_arg,
        inference_timeout_arg,
        rule_timeout_arg,
        tick_rate_arg,
        bt_node,
    ])
