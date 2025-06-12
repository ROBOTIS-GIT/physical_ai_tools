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
# Author: Dongyun Kim, Seongwoo Kim

import os
import glob

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Find package share directory for the physical_ai_server package
    pkg_dir = get_package_share_directory('physical_ai_server')
    
    # Get all YAML config files from the config directory
    robot_config_dir = os.path.join(pkg_dir, 'config', 'robot_config')
    control_config_dir = os.path.join(pkg_dir, 'config', 'control_config')
    robot_config_files = glob.glob(os.path.join(robot_config_dir, '*.yaml'))
    control_config_files = glob.glob(os.path.join(control_config_dir, '*.yaml'))
    config_files = sorted(robot_config_files + control_config_files)
    print(f"Loading config files: {config_files}")

    physical_ai_server = Node(
        package='physical_ai_server',
        executable='physical_ai_server',
        name='physical_ai_server',
        output='screen',
        parameters=config_files
    )

    return LaunchDescription([
        physical_ai_server
    ])