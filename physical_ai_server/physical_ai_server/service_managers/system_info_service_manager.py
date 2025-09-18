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

from ament_index_python.packages import get_package_share_directory
from physical_ai_interfaces.msg import BrowserItem
from physical_ai_interfaces.srv import BrowseFile, GetRobotTypeList
from physical_ai_server.service_managers.base_service_manager import BaseServiceManager
from physical_ai_server.utils.file_browse_utils import FileBrowseUtils

from rclpy.node import Node


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
        self.file_browse_utils = FileBrowseUtils(
            max_workers=8,
            logger=self.node.get_logger())

    def initialize_services(self) -> None:
        """Initialize system info related services."""
        self.logger.info('Initializing system info services...')

        service_definitions = [
            ('/get_robot_types', GetRobotTypeList, self.get_robot_types_callback),
            ('/browse_file', BrowseFile, self.browse_file_callback),
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

    def browse_file_callback(self, request, response):
        """
        Handle file browsing requests.

        Args:
            request: Service request
            response: Service response

        Returns:
            Service response
        """
        try:
            if request.action == 'get_path':
                result = self.file_browse_utils.handle_get_path_action(
                    request.current_path)
            elif request.action == 'go_parent':
                # Check if target_files or target_folders are provided
                target_files = None
                target_folders = None

                if hasattr(request, 'target_files') and request.target_files:
                    target_files = set(request.target_files)
                if hasattr(request, 'target_folders') and request.target_folders:
                    target_folders = set(request.target_folders)

                if target_files or target_folders:
                    # Use parallel target checking for go_parent
                    result = self.file_browse_utils.handle_go_parent_with_target_check(
                        request.current_path,
                        target_files,
                        target_folders)
                else:
                    # Use standard go_parent (no targets specified)
                    result = self.file_browse_utils.handle_go_parent_action(
                        request.current_path)
            elif request.action == 'browse':
                # Check if target_files or target_folders are provided
                target_files = None
                target_folders = None

                if hasattr(request, 'target_files') and request.target_files:
                    target_files = set(request.target_files)
                if hasattr(request, 'target_folders') and request.target_folders:
                    target_folders = set(request.target_folders)

                if target_files or target_folders:
                    # Use parallel target checking
                    result = self.file_browse_utils.handle_browse_with_target_check(
                        request.current_path,
                        request.target_name,
                        target_files,
                        target_folders)
                else:
                    # Use standard browsing (no targets specified)
                    result = self.file_browse_utils.handle_browse_action(
                        request.current_path, request.target_name)
            else:
                result = {
                    'success': False,
                    'message': f'Unknown action: {request.action}',
                    'current_path': '',
                    'parent_path': '',
                    'selected_path': '',
                    'items': []
                }

            # Convert result dict to response object
            response.success = result['success']
            response.message = result['message']
            response.current_path = result['current_path']
            response.parent_path = result['parent_path']
            response.selected_path = result['selected_path']

            # Convert item dicts to BrowserItem objects
            response.items = []
            for item_dict in result['items']:
                item = BrowserItem()
                item.name = item_dict['name']
                item.full_path = item_dict['full_path']
                item.is_directory = item_dict['is_directory']
                item.size = item_dict['size']
                item.modified_time = item_dict['modified_time']
                # Set has_target_file field (default False for files)
                item.has_target_file = item_dict.get('has_target_file', False)
                response.items.append(item)

        except Exception as e:
            self.node.get_logger().error(f'Error in browse file handler: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
            response.current_path = ''
            response.parent_path = ''
            response.selected_path = ''
            response.items = []

        return response
