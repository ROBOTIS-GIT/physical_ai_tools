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
# Author: Dongyun Kim, Kiwoong Park

from typing import Any

from physical_ai_interfaces.msg import DatasetInfo
from physical_ai_interfaces.srv import EditDataset, GetDatasetInfo
from physical_ai_server.data_processing.data_editor import DataEditor
from physical_ai_server.service_managers.base_service_manager import BaseServiceManager

from rclpy.node import Node


class EditDataServiceManager(BaseServiceManager):
    """
    Service manager for edit data related operations.

    Manages services for:
    - Edit data
    """

    def __init__(self, node: Node, main_server: Any):
        """
        Initialize the edit data service manager.

        Args:
            node: ROS2 node instance
            main_server: Reference to the main PhysicalAIServer instance
        """
        super().__init__(node)
        self.main_server = main_server

        # Initialize DataEditor for dataset editing
        self.data_editor = DataEditor()

    def initialize_services(self) -> None:
        """Initialize edit data related services."""
        self.logger.info('Initializing edit data services...')

        service_definitions = [
            ('/dataset/edit', EditDataset, self.dataset_edit_callback),
            ('/dataset/get_info', GetDatasetInfo, self.get_dataset_info_callback),
        ]

        self.register_services(service_definitions)
        self.logger.info('Edit data services initialized successfully')

    def dataset_edit_callback(self, request, response):
        try:
            if request.mode == EditDataset.Request.MERGE:
                merge_dataset_list = request.merge_dataset_list
                output_path = request.output_path
                # TODO: Implement HuggingFace upload functionality if needed
                # upload_huggingface = request.upload_huggingface
                self.data_editor.merge_datasets(
                    merge_dataset_list, output_path)

            elif request.mode == EditDataset.Request.DELETE:
                delete_dataset_path = request.delete_dataset_path
                delete_episode_num = sorted(request.delete_episode_num, reverse=True)
                # TODO: Implement HuggingFace upload functionality if needed
                # upload_huggingface = request.upload_huggingface
                for episode_num in delete_episode_num:
                    self.data_editor.delete_episode(
                        delete_dataset_path, episode_num)
            else:
                response.success = False
                response.message = f'Unknown edit mode: {request.mode}'
                return response

            response.success = True
            response.message = f'Successfully processed edit mode: {request.mode}'
            return response

        except Exception as e:
            self.node.get_logger().error(f'Error in dataset_edit_callback: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'

        return response

    def get_dataset_info_callback(self, request, response):
        try:
            dataset_path = request.dataset_path
            dataset_info = self.data_editor.get_dataset_info(dataset_path)

            info = DatasetInfo()
            info.codebase_version = dataset_info.get('codebase_version', 'unknown') if isinstance(
                dataset_info.get('codebase_version'), str) else 'unknown'
            info.robot_type = dataset_info.get('robot_type', 'unknown') if isinstance(
                dataset_info.get('robot_type'), str) else 'unknown'
            info.total_episodes = dataset_info.get('total_episodes', 0) if isinstance(
                dataset_info.get('total_episodes'), int) else 0
            info.total_tasks = dataset_info.get('total_tasks', 0) if isinstance(
                dataset_info.get('total_tasks'), int) else 0
            info.fps = dataset_info.get('fps', 0) if isinstance(
                dataset_info.get('fps'), int) else 0

            response.dataset_info = info
            response.success = True
            response.message = 'Dataset info retrieved successfully'
            return response

        except Exception as e:
            self.node.get_logger().error(f'Error in get_dataset_info_callback: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
            response.dataset_info = DatasetInfo()
            return response
