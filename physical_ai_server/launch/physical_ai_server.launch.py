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
# Author: Dongyun Kim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Find package share directory for the physical_ai_server package
    pkg_dir = get_package_share_directory('physical_ai_server')

    # Launch arguments
    robot_type = LaunchConfiguration('robot_type', default='ai_worker')
    operation_mode = LaunchConfiguration('operation_mode', default='collection')
    timer_frequency = LaunchConfiguration('timer_frequency', default='100.0')  # Hz
    config_file = LaunchConfiguration(
        'config_file',
        default=os.path.join(pkg_dir, 'config', 'robot_config.yaml')
    )

    # Declare launch arguments so they can be set from the command line
    declare_robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='ai_worker',
        description='Type of robot to operate'
    )

    declare_operation_mode_arg = DeclareLaunchArgument(
        'operation_mode',
        default_value='collection',
        description='Operation mode: collection or inference'
    )

    declare_timer_frequency_arg = DeclareLaunchArgument(
        'timer_frequency',
        default_value='30.0',
        description='Timer callback frequency in Hz'
    )

    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'robot_config.yaml'),
        description='Path to the robot configuration YAML file'
    )

    # Create node action for the physical_ai_server node
    physical_ai_server = Node(
        package='physical_ai_server',
        executable='physical_ai_server',
        name='physical_ai_server',
        output='screen',
        parameters=[
            config_file,
            {
                'robot_type': robot_type,
                'operation_mode': operation_mode,
                'timer_frequency': timer_frequency
            }
        ]
    )

    # Return launch description
    return LaunchDescription([
        declare_robot_type_arg,
        declare_operation_mode_arg,
        declare_timer_frequency_arg,
        declare_config_file_arg,
        physical_ai_server
    ])
