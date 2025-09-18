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

from typing import Any

from physical_ai_interfaces.srv import GetPolicyList, GetSavedPolicyList
from physical_ai_server.inference.inference_manager import InferenceManager
from physical_ai_server.service_managers.base_service_manager import BaseServiceManager

from rclpy.node import Node


class InferenceServiceManager(BaseServiceManager):
    """
    Service manager for inference related operations.

    Manages services for:
    - Policy listing and management
    - Saved policy retrieval
    """

    def __init__(self, node: Node, main_server: Any):
        """
        Initialize the inference service manager.

        Args:
            node: ROS2 node instance
            main_server: Reference to the main PhysicalAIServer instance
        """
        super().__init__(node)
        self.main_server = main_server

    def initialize_services(self) -> None:
        """Initialize inference related services."""
        self.logger.info('Initializing inference services...')

        service_definitions = [
            ('/get_policy_list', GetPolicyList, self.get_policy_list_callback),
            ('/get_saved_policies', GetSavedPolicyList, self.get_saved_policies_callback),
        ]

        self.register_services(service_definitions)
        self.logger.info('Inference services initialized successfully')

    def get_policy_list_callback(self, request, response):
        """
        Handle policy list retrieval requests.

        Args:
            request: Service request
            response: Service response

        Returns:
            Service response
        """
        policy_list = InferenceManager.get_available_policies()
        if not policy_list:
            self.logger.warning('No policies available')
            response.success = False
            response.message = 'No policies available'
        else:
            self.logger.info(f'Available policies: {policy_list}')
            response.success = True
            response.message = 'Policy list retrieved successfully'
        response.policy_list = policy_list
        return response

    def get_saved_policies_callback(self, request, response):
        """
        Handle saved policies retrieval requests.

        Args:
            request: Service request
            response: Service response

        Returns:
            Service response
        """
        saved_policy_path, saved_policy_type = InferenceManager.get_saved_policies()
        if not saved_policy_path and not saved_policy_type:
            self.logger.warning('No saved policies found')
            response.saved_policy_path = []
            response.saved_policy_type = []
            response.success = False
            response.message = 'No saved policies found'
        else:
            self.logger.info(f'Saved policies path: {saved_policy_path}')
            response.saved_policy_path = saved_policy_path
            response.saved_policy_type = saved_policy_type
            response.success = True
            response.message = 'Saved policies retrieved successfully'
        return response
