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

from physical_ai_interfaces.msg import BrowserItem, DatasetInfo
from physical_ai_interfaces.srv import EditDataset

from .base_handler import BaseServiceHandler


class DatasetServiceHandler(BaseServiceHandler):
    """
    Service handler for dataset operations.

    Manages dataset editing (merge/delete), file browsing,
    and dataset information retrieval.
    """

    def __init__(self, node, data_editor, file_browse_utils):
        """
        Initialize dataset service handler.

        Args:
            node: ROS2 node instance
            data_editor: DataEditor instance for dataset operations
            file_browse_utils: FileBrowseUtils instance for file operations
        """
        super().__init__(node)
        self.data_editor = data_editor
        self.file_browse_utils = file_browse_utils

    def dataset_edit_callback(self, request, response):
        """
        Handle dataset editing operations (merge/delete).

        Args:
            request: EditDataset service request
            response: EditDataset service response

        Returns:
            response: Modified response with operation status
        """
        try:
            if request.mode == EditDataset.Request.MERGE:
                merge_dataset_list = request.merge_dataset_list
                output_path = request.output_path
                self.data_editor.merge_datasets(
                    merge_dataset_list,
                    output_path
                )

            elif request.mode == EditDataset.Request.DELETE:
                delete_dataset_path = request.delete_dataset_path
                delete_episode_num = sorted(
                    request.delete_episode_num,
                    reverse=True
                )
                for episode_num in delete_episode_num:
                    self.data_editor.delete_episode(
                        delete_dataset_path,
                        episode_num
                    )
            else:
                return self._create_error_response(
                    response,
                    f'Unknown edit mode: {request.mode}'
                )

            return self._create_success_response(
                response,
                f'Successfully processed edit mode: {request.mode}'
            )

        except Exception as e:
            self.logger.error(f'Error in dataset_edit_callback: {str(e)}')
            return self._create_error_response(
                response,
                f'Error: {str(e)}'
            )

    def get_dataset_info_callback(self, request, response):
        """
        Retrieve dataset information.

        Args:
            request: GetDatasetInfo service request
            response: GetDatasetInfo service response

        Returns:
            response: Modified response with dataset information
        """
        try:
            dataset_path = request.dataset_path
            dataset_info = self.data_editor.get_dataset_info(dataset_path)

            info = DatasetInfo()
            info.codebase_version = (
                dataset_info.get('codebase_version', 'unknown')
                if isinstance(dataset_info.get('codebase_version'), str)
                else 'unknown'
            )
            info.robot_type = (
                dataset_info.get('robot_type', 'unknown')
                if isinstance(dataset_info.get('robot_type'), str)
                else 'unknown'
            )
            info.total_episodes = (
                dataset_info.get('total_episodes', 0)
                if isinstance(dataset_info.get('total_episodes'), int)
                else 0
            )
            info.total_tasks = (
                dataset_info.get('total_tasks', 0)
                if isinstance(dataset_info.get('total_tasks'), int)
                else 0
            )
            info.fps = (
                dataset_info.get('fps', 0)
                if isinstance(dataset_info.get('fps'), int)
                else 0
            )

            response.dataset_info = info
            return self._create_success_response(
                response,
                'Dataset info retrieved successfully'
            )

        except Exception as e:
            self.logger.error(f'Error in get_dataset_info_callback: {str(e)}')
            response.dataset_info = DatasetInfo()
            return self._create_error_response(
                response,
                f'Error: {str(e)}'
            )

    def browse_file_callback(self, request, response):
        """
        Handle file browsing operations.

        Args:
            request: BrowseFile service request
            response: BrowseFile service response

        Returns:
            response: Modified response with browsing results
        """
        try:
            if request.action == 'get_path':
                result = self.file_browse_utils.handle_get_path_action(
                    request.current_path
                )

            elif request.action == 'go_parent':
                # Check if target_files or target_folders are provided
                target_files = None
                target_folders = None

                if hasattr(request, 'target_files') and request.target_files:
                    target_files = set(request.target_files)
                if (hasattr(request, 'target_folders') and
                        request.target_folders):
                    target_folders = set(request.target_folders)

                if target_files or target_folders:
                    # Use parallel target checking for go_parent
                    result = (
                        self.file_browse_utils.
                        handle_go_parent_with_target_check(
                            request.current_path,
                            target_files,
                            target_folders
                        )
                    )
                else:
                    # Use standard go_parent (no targets specified)
                    result = self.file_browse_utils.handle_go_parent_action(
                        request.current_path
                    )

            elif request.action == 'browse':
                # Check if target_files or target_folders are provided
                target_files = None
                target_folders = None

                if hasattr(request, 'target_files') and request.target_files:
                    target_files = set(request.target_files)
                if (hasattr(request, 'target_folders') and
                        request.target_folders):
                    target_folders = set(request.target_folders)

                if target_files or target_folders:
                    # Use parallel target checking
                    result = (
                        self.file_browse_utils.
                        handle_browse_with_target_check(
                            request.current_path,
                            request.target_name,
                            target_files,
                            target_folders
                        )
                    )
                else:
                    # Use standard browsing (no targets specified)
                    result = self.file_browse_utils.handle_browse_action(
                        request.current_path,
                        request.target_name
                    )
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
            self.logger.error(f'Error in browse file handler: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
            response.current_path = ''
            response.parent_path = ''
            response.selected_path = ''
            response.items = []

        return response
