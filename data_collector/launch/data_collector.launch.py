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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    config_dir = get_package_share_directory('data_collector')
    robot_params = load_yaml(os.path.join(config_dir, 'config', 'joint_order.yaml'))

    return LaunchDescription([
        Node(
            package='data_collector',
            executable='data_collector',
            name='data_collector',
            output='screen',
            parameters=[
                robot_params['data_collector']['ros__parameters'],
            ]
        ),
        Node(
            package='data_collector',
            executable='trajectory_stamper',
            name='trajectory_stamper',
            output='screen'
        )
    ])
