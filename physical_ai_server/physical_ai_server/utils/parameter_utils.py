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


from typing import Any, Dict, List

from rclpy.node import Node


def declare_parameters(
    node: Node,
    robot_type: str,
    param_names: List[str],
    default_value: Any = None
) -> None:
    for name in param_names:
        param_path = f'{robot_type}.{name}'
        if not node.has_parameter(param_path):
            node.declare_parameter(param_path, default_value)


def load_parameters(
    node: Node,
    robot_type: str,
    param_names: List[str]
) -> Dict[str, Any]:
    params = {}
    for name in param_names:
        param_path = f'{robot_type}.{name}'
        params[name] = node.get_parameter(param_path).value
    return params


def log_parameters(node: Node, params: Dict[str, Any], log_level: str = 'info') -> None:
    for name, value in params.items():
        if log_level == 'debug':
            node.get_logger().debug(f'{name}: {value}')
        elif log_level == 'info':
            node.get_logger().info(f'{name}: {value}')
        elif log_level == 'warn':
            node.get_logger().warn(f'{name}: {value}')
        elif log_level == 'error':
            node.get_logger().error(f'{name}: {value}')


def parse_topic_list_with_names(topic_list: List[str]) -> Dict[str, str]:
    parsed_topics = {}
    if topic_list:
        for topic_entry in topic_list:
            try:
                key, value = topic_entry.split(':', 1)
                parsed_topics[key] = value
            except ValueError:
                # Skip entries that don't have the expected format
                continue
    return parsed_topics
