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

from typing import Dict, List, Any, Optional

import rclpy
from rclpy.node import Node


class ParameterParser:
    """
    A class to parse parameters from ROS2 YAML configuration files.
    Specialized for handling key-value pairs stored in lists.
    """

    def __init__(self, node: Node, model_type: str = 'ffw_1_0'):
        """
        Initialize the parameter parser with a ROS2 node.

        Args:
            node: The ROS2 node to read parameters from
            model_type: The model type key in parameters
        """
        self.node = node
        self.model_type = model_type
        self.param_prefix = f"{self.model_type}."

    def parse_key_value_list(self, param_list: List[str]) -> Dict[str, str]:
        """
        Parse a list of strings in "key:value" format to a dictionary.

        Args:
            param_list: List of strings in format "key:value"

        Returns:
            Dictionary of parsed key-value pairs
        """
        result = {}
        for item in param_list:
            parts = item.split(':', 1)  # Split only on the first colon
            if len(parts) == 2:
                key, value = parts
                result[key] = value
            else:
                self.node.get_logger().warn(f"Invalid parameter format: {item}")
        return result

    def get_camera_topics(self) -> Dict[str, str]:
        """
        Get parsed camera topics from parameters.

        Returns:
            Dictionary mapping camera topic keys to their values
        """
        try:
            param_name = f"{self.param_prefix}camera_topic"
            param_list = self.node.get_parameter(param_name).value
            return self.parse_key_value_list(param_list)
        except Exception as e:
            self.node.get_logger().error(f"Error parsing camera topics: {e}")
            return {}

    def get_joint_topics(self) -> Dict[str, str]:
        """
        Get parsed joint topics from parameters.

        Returns:
            Dictionary mapping joint topic keys to their values
        """
        try:
            param_name = f"{self.param_prefix}joint_topic"
            param_list = self.node.get_parameter(param_name).value
            return self.parse_key_value_list(param_list)
        except Exception as e:
            self.node.get_logger().error(f"Error parsing joint topics: {e}")
            return {}

    def get_observation_types(self) -> List[str]:
        """
        Get observation types from parameters.

        Returns:
            List of observation types
        """
        try:
            param_name = f"{self.param_prefix}observation"
            return self.node.get_parameter(param_name).value
        except Exception as e:
            self.node.get_logger().error(f"Error getting observation types: {e}")
            return []

    def get_joint_list(self) -> List[str]:
        """
        Get joint list from parameters.

        Returns:
            List of joint types (follower, leader_left, etc.)
        """
        try:
            param_name = f"{self.param_prefix}joint_list"
            return self.node.get_parameter(param_name).value
        except Exception as e:
            self.node.get_logger().error(f"Error getting joint list: {e}")
            return []