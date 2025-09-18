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
from typing import Any
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from physical_ai_interfaces.srv import GetRobotTypeList
from physical_ai_server.service_managers.base_service_manager import BaseServiceManager


class SystemInfoServiceManager(BaseServiceManager):
    """
    Service manager for system information related operations.
    
    Manages services for:
    - Robot type information
    - System configuration details
    """
    
    def __init__(self, node: Node, main_server: Any):
        """
        Initialize the system info service manager.
        
        Args:
            node: ROS2 node instance
            main_server: Reference to the main PhysicalAIServer instance
        """
        super().__init__(node)
        self.main_server = main_server
        self.robot_type_list = self._get_robot_type_list()
    
    def initialize_services(self) -> None:
        """Initialize system info related services."""
        self.logger.info('Initializing system info services...')
        
        service_definitions = [
            ('/get_robot_types', GetRobotTypeList, self.get_robot_types_callback),
        ]
        
        self.register_services(service_definitions)
        self.logger.info('System info services initialized successfully')
    
    def _get_robot_type_list(self):
        """
        Get the list of available robot types from configuration files.
        
        Returns:
            List of available robot types
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
        Handle robot type list retrieval requests.
        
        Args:
            request: Service request
            response: Service response
            
        Returns:
            Service response
        """
        if self.robot_type_list is None:
            self.logger.error('Robot type list is not set')
            response.robot_types = []
            response.success = False
            response.message = 'Robot type list is not set'
            return response

        self.logger.info(f'Available robot types: {self.robot_type_list}')
        response.robot_types = self.robot_type_list
        response.success = True
        response.message = 'Robot type list retrieved successfully'
        return response
