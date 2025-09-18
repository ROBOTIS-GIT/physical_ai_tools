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

from physical_ai_interfaces.srv import ControlHfServer, GetHFUser, SetHFUser
from physical_ai_server.data_processing.data_manager import DataManager
from physical_ai_server.service_managers.base_service_manager import BaseServiceManager

from rclpy.node import Node


class HuggingFaceServiceManager(BaseServiceManager):
    """
    Service manager for HuggingFace related operations.

    Manages services for:
    - HF server control (upload/download/cancel)
    - HF user registration and management
    """

    def __init__(self, node: Node, main_server: Any):
        """
        Initialize the HuggingFace service manager.

        Args:
            node: ROS2 node instance
            main_server: Reference to the main PhysicalAIServer instance
        """
        super().__init__(node)
        self.main_server = main_server

    def initialize_services(self) -> None:
        """Initialize HuggingFace related services."""
        self.logger.info('Initializing HuggingFace services...')

        service_definitions = [
            ('/huggingface/control', ControlHfServer, self.control_hf_server_callback),
            ('/register_hf_user', SetHFUser, self.set_hf_user_callback),
            ('/get_registered_hf_user', GetHFUser, self.get_hf_user_callback),
        ]

        self.register_services(service_definitions)
        self.logger.info('HuggingFace services initialized successfully')

    def control_hf_server_callback(self, request, response):
        """
        Handle HuggingFace server control requests.

        Args:
            request: Service request
            response: Service response

        Returns:
            Service response
        """
        try:
            mode = request.mode
            repo_id = request.repo_id
            local_dir = request.local_dir
            repo_type = request.repo_type
            author = request.author

            if self.main_server.hf_cancel_on_progress:
                response.success = False
                response.message = 'HF API Worker is currently canceling'
                return response

            if mode == 'cancel':
                # Immediate cleanup - force stop the worker
                try:
                    self.main_server.hf_cancel_on_progress = True
                    self.main_server._cleanup_hf_api_worker_with_threading()
                    response.success = True
                    response.message = 'Cancellation started.'
                except Exception as e:
                    self.logger.error(f'Error during cancel: {e}')
                finally:
                    self.main_server.hf_cancel_on_progress = False
                    return response

            # Restart HF API Worker if it does not exist or is not running
            if self.main_server.hf_api_worker is None or \
               not self.main_server.hf_api_worker.is_alive():
                self.logger.info('HF API Worker not running, restarting...')
                self.main_server._init_hf_api_worker()
            # Return error if the worker is busy
            if self.main_server.hf_api_worker.is_busy():
                self.logger.warning('HF API Worker is currently busy with another task')
                response.success = False
                response.message = 'HF API Worker is currently busy with another task'
                return response
            # Prepare request data for the worker
            request_data = {
                'mode': mode,
                'repo_id': repo_id,
                'local_dir': local_dir,
                'repo_type': repo_type,
                'author': author
            }
            # Send request to HF API Worker
            if self.main_server.hf_api_worker.send_request(request_data):
                self.logger.info(f'HF API request sent successfully: {mode} for {repo_id}')
                response.success = True
                response.message = f'HF API request started: {mode} for {repo_id}'
            else:
                self.logger.error('Failed to send request to HF API Worker')
                response.success = False
                response.message = 'Failed to send request to HF API Worker'
            return response
        except Exception as e:
            self.logger.error(f'Error in HF server callback: {str(e)}')
            response.success = False
            response.message = f'Error in HF server callback: {str(e)}'
            return response

    def set_hf_user_callback(self, request, response):
        """
        Handle HuggingFace user registration requests.

        Args:
            request: Service request
            response: Service response

        Returns:
            Service response
        """
        request_hf_token = request.token
        if DataManager.register_huggingface_token(request_hf_token):
            self.logger.info('Hugging Face user token registered successfully')
            response.user_id_list = DataManager.get_huggingface_user_id()
            response.success = True
            response.message = 'Hugging Face user token registered successfully'
        else:
            self.logger.error('Failed to register Hugging Face user token')
            response.user_id_list = []
            response.success = False
            response.message = 'Failed to register token, Please check your token'
        return response

    def get_hf_user_callback(self, request, response):
        """
        Handle HuggingFace user retrieval requests.

        Args:
            request: Service request
            response: Service response

        Returns:
            Service response
        """
        user_ids = DataManager.get_huggingface_user_id()
        if user_ids is not None:
            response.user_id_list = user_ids
            self.logger.info(f'Hugging Face user IDs: {user_ids}')
            response.success = True
            response.message = 'Hugging Face user IDs retrieved successfully'

        else:
            self.logger.error('Failed to retrieve Hugging Face user ID')
            response.user_id_list = []
            response.success = False
            response.message = 'Failed to retrieve Hugging Face user ID'

        return response
