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

import glob
import os

from ament_index_python.packages import get_package_share_directory
from physical_ai_server.inference.inference_manager import InferenceManager

from .base_handler import BaseServiceHandler


class RobotServiceHandler(BaseServiceHandler):
    """
    Service handler for robot configuration and policy management.

    Manages robot type selection, policy lists, and saved policy retrieval.
    """

    def __init__(self, node, inference_manager):
        """
        Initialize robot service handler.

        Args:
            node: ROS2 node instance
            inference_manager: InferenceManager instance for policy operations
        """
        super().__init__(node)
        self.inference_manager = inference_manager
        self.robot_type_list = self._load_robot_types()

    def _load_robot_types(self):
        """
        Load available robot types from configuration files.

        Returns:
            list: List of available robot type names
        """
        pkg_dir = get_package_share_directory('physical_ai_server')
        config_dir = os.path.join(pkg_dir, 'config')
        config_files = glob.glob(os.path.join(config_dir, '*.yaml'))
        config_files.sort()

        robot_type_list = []
        for config_file in config_files:
            robot_type = os.path.splitext(os.path.basename(config_file))[0]
            if robot_type.endswith('_config'):
                robot_type = robot_type[:-7]
            robot_type_list.append(robot_type)

        self.logger.info(f'Available robot types: {robot_type_list}')
        return robot_type_list

    def get_robot_types_callback(self, request, response):
        """
        Retrieve list of available robot types.

        Args:
            request: GetRobotTypeList service request
            response: GetRobotTypeList service response

        Returns:
            response: Modified response with robot type list
        """
        if self.robot_type_list is None:
            self.logger.error('Robot type list is not set')
            response.robot_types = []
            return self._create_error_response(
                response,
                'Robot type list is not set'
            )

        self.logger.info(f'Available robot types: {self.robot_type_list}')
        response.robot_types = self.robot_type_list
        return self._create_success_response(
            response,
            'Robot type list retrieved successfully'
        )

    def get_policy_list_callback(self, request, response):
        """
        Retrieve list of available policies.

        Args:
            request: GetPolicyList service request
            response: GetPolicyList service response

        Returns:
            response: Modified response with policy list
        """
        policy_list = InferenceManager.get_available_policies()
        if not policy_list:
            self.logger.warning('No policies available')
            response.policy_list = []
            return self._create_error_response(
                response,
                'No policies available'
            )
        else:
            self.logger.info(f'Available policies: {policy_list}')
            response.policy_list = policy_list
            return self._create_success_response(
                response,
                'Policy list retrieved successfully'
            )

    def get_saved_policies_callback(self, request, response):
        """
        Retrieve list of saved policies from cache.

        Args:
            request: GetSavedPolicyList service request
            response: GetSavedPolicyList service response

        Returns:
            response: Modified response with saved policy information
        """
        saved_policy_path, saved_policy_type = (
            InferenceManager.get_saved_policies()
        )

        if not saved_policy_path and not saved_policy_type:
            self.logger.warning('No saved policies found')
            response.saved_policy_path = []
            response.saved_policy_type = []
            return self._create_error_response(
                response,
                'No saved policies found'
            )
        else:
            self.logger.info(f'Saved policies path: {saved_policy_path}')
            response.saved_policy_path = saved_policy_path
            response.saved_policy_type = saved_policy_type
            return self._create_success_response(
                response,
                'Saved policies retrieved successfully'
            )