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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    config_dir = get_package_share_directory('data_collector')
    omx_params_path = os.path.join(config_dir, 'config', 'joint_order_omx.yaml')
    inspire_params_path = os.path.join(config_dir, 'config', 'joint_order_inspire.yaml')
    normal_params_path = os.path.join(config_dir, 'config', 'joint_order.yaml')

    robot_params_omx = load_yaml(omx_params_path)
    robot_params_inspire = load_yaml(inspire_params_path)
    robot_params_normal = load_yaml(normal_params_path)

    mode = LaunchConfiguration('mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='worker',
            description='Mode of the data collector: omx or worker'
        ),

        Node(
            package='data_collector',
            executable='data_collector_omx',
            name='data_collector_omx',
            output='screen',
            parameters=[robot_params_omx['data_collector']['ros__parameters']],
            condition=IfCondition(EqualsSubstitution(mode, 'omx'))
        ),

        Node(
            package='data_collector',
            executable='data_collector_inspire',
            name='data_collector_inspire',
            output='screen',
            parameters=[robot_params_inspire['data_collector']['ros__parameters']],
            condition=IfCondition(EqualsSubstitution(mode, 'inspire'))
        ),

        Node(
            package='data_collector',
            executable='trajectory_stamper_inspire',
            name='trajectory_stamper_inspire',
            output='screen',
            condition=IfCondition(EqualsSubstitution(mode, 'inspire'))
        ),

        Node(
            package='data_collector',
            executable='trajectory_stamper',
            name='trajectory_stamper',
            output='screen',
            condition=IfCondition(EqualsSubstitution(mode, 'worker'))
        ),
    ])
