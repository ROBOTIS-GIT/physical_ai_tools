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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Setup launch nodes with access to launch configurations."""

    robot_type = LaunchConfiguration('robot_type').perform(context)

    physical_ai_server_pkg_share = get_package_share_directory('physical_ai_server')
    physical_ai_bt_pkg_share = get_package_share_directory('physical_ai_bt')

    server_config_dir = os.path.join(physical_ai_server_pkg_share, 'config')
    bt_params_path = os.path.join(physical_ai_bt_pkg_share, 'bt_bringup', 'params', 'bt_node_params.yaml')
    robot_config_path = os.path.join(server_config_dir, f'{robot_type}_config.yaml')

    if not os.path.exists(robot_config_path):
        print(f"Warning: Config file not found: {robot_config_path}")
        print(f"Falling back to ffw_sg2_rev1_config.yaml")
        robot_config_path = os.path.join(server_config_dir, 'ffw_sg2_rev1_config.yaml')

    import yaml
    with open(robot_config_path, 'r') as f:
        config = yaml.safe_load(f)

    server_config = config.get('physical_ai_server', {}).get('ros__parameters', {}).get(robot_type, {})
    joint_list = server_config.get('joint_list', [])
    joint_order = server_config.get('joint_order', {})
    joint_topic_list = server_config.get('joint_topic_list', [])

    bt_params = {
        f'{robot_type}.joint_list': joint_list,
        f'{robot_type}.joint_topic_list': joint_topic_list,
    }
    for joint_name, order in joint_order.items():
        bt_params[f'{robot_type}.joint_order.{joint_name}'] = order

    bt_node = Node(
        package='physical_ai_bt',
        executable='bt_node',
        name='bt_node',
        output='screen',
        parameters=[
            bt_params_path,
            bt_params
        ]
    )

    return [bt_node]


def generate_launch_description():

    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='ffw_sg2_rev1',
        description='Type of robot (e.g., ffw_sg2_rev1)'
    )

    return LaunchDescription([
        robot_type_arg,
        OpaqueFunction(function=launch_setup)
    ])
