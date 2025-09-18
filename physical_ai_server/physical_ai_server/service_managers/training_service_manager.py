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

import os
from pathlib import Path
from typing import Any
from rclpy.node import Node

from physical_ai_interfaces.srv import (
    SendTrainingCommand,
    GetPolicyList,
    GetUserList,
    GetDatasetList,
    GetModelWeightList
)
from physical_ai_server.service_managers.base_service_manager import BaseServiceManager
from physical_ai_server.training.training_manager import TrainingManager


class TrainingServiceManager(BaseServiceManager):
    """
    Service manager for training related operations.
    
    Manages services for:
    - Training commands (start/stop training)
    - Training resource management (users, datasets, model weights)
    - Available training resources listing
    """
    
    def __init__(self, node: Node, main_server: Any):
        """
        Initialize the training service manager.
        
        Args:
            node: ROS2 node instance
            main_server: Reference to the main PhysicalAIServer instance
        """
        super().__init__(node)
        self.main_server = main_server
        self.DEFAULT_SAVE_ROOT_PATH = Path.home() / '.cache/huggingface/lerobot'
    
    def initialize_services(self) -> None:
        """Initialize training related services."""
        self.logger.info('Initializing training services...')
        
        service_definitions = [
            ('/training/command', SendTrainingCommand, self.user_training_interaction_callback),
            ('/training/get_available_policy', GetPolicyList, self.get_available_list_callback),
            ('/training/get_user_list', GetUserList, self.get_user_list_callback),
            ('/training/get_dataset_list', GetDatasetList, self.get_dataset_list_callback),
            ('/training/get_model_weight_list', GetModelWeightList, self.get_model_weight_list_callback),
        ]
        
        self.register_services(service_definitions)
        self.logger.info('Training services initialized successfully')
    
    def user_training_interaction_callback(self, request, response):
        """
        Handle training command requests.
        
        Args:
            request: Service request
            response: Service response
            
        Returns:
            Service response
        """
        try:
            if request.command == SendTrainingCommand.Request.START:
                self.main_server.training_manager = TrainingManager()
                from physical_ai_server.timer.timer_manager import TimerManager
                self.main_server.training_timer = TimerManager(node=self.main_server)
                self.main_server.training_timer.set_timer(
                    timer_name='training_status',
                    timer_frequency=self.main_server.TRAINING_STATUS_TIMER_FREQUENCY,
                    callback_function=lambda: self.main_server.communicator.publish_training_status(
                        self.main_server.get_training_status()
                    )
                )
                self.main_server.training_timer.start(timer_name='training_status')

                if self.main_server.training_thread and self.main_server.training_thread.is_alive():
                    response.success = False
                    response.message = 'Training is already in progress'
                    return response

                output_folder_name = request.training_info.output_folder_name
                weight_save_root_path = TrainingManager.get_weight_save_root_path()
                self.logger.info(
                    f'Weight save root path: {weight_save_root_path}, '
                    f'Output folder name: {output_folder_name}'
                )
                output_path = weight_save_root_path / output_folder_name
                if output_path.exists():
                    response.success = False
                    response.message = f'Output folder already exists: {output_path}'
                    self.main_server.is_training = False
                    self.main_server.communicator.publish_training_status(
                        self.main_server.get_training_status()
                    )

                    self.main_server.training_manager.stop_event.set()
                    self.main_server.training_timer.stop('training_status')
                    return response

                self.main_server.training_manager.training_info = request.training_info

                def run_training():
                    try:
                        self.main_server.training_manager.train()
                    finally:
                        self.main_server.is_training = False
                        self.logger.info('Training completed.')
                        self.main_server.communicator.publish_training_status(
                            self.main_server.get_training_status()
                        )
                        self.main_server.training_manager.stop_event.set()
                        self.main_server.training_timer.stop('training_status')

                import threading
                self.main_server.training_thread = threading.Thread(target=run_training, daemon=True)
                self.main_server.training_thread.start()
                self.main_server.is_training = True

                response.success = True
                response.message = 'Training started successfully'

            else:
                if request.command == SendTrainingCommand.Request.FINISH:
                    self.main_server.is_training = False
                    self.main_server.communicator.publish_training_status(
                        self.main_server.get_training_status()
                    )
                    self.main_server.training_timer.stop('training_status')
                    if self.main_server.training_thread and self.main_server.training_thread.is_alive():
                        self.main_server.training_manager.stop_event.set()
                        self.main_server.training_thread.join()
                        response.success = True
                        response.message = 'Training stopped successfully'
                    else:
                        response.success = False
                        response.message = 'No training in progress to stop'

        except Exception as e:
            self.logger.error(f'Error in user_training_interaction: {str(e)}')
            response.success = False
            response.message = f'Error in user_training_interaction: {str(e)}'
            return response
        return response

    def get_available_list_callback(self, request, response):
        """
        Handle available training resources listing requests.
        
        Args:
            request: Service request
            response: Service response
            
        Returns:
            Service response
        """
        response.success = True
        response.message = 'Policy and device lists retrieved successfully'
        response.policy_list, response.device_list = TrainingManager.get_available_list()
        return response

    def get_user_list_callback(self, request, response):
        """
        Handle user folder list retrieval requests.
        
        Args:
            request: Service request
            response: Service response
            
        Returns:
            Service response
        """
        try:
            if not self.DEFAULT_SAVE_ROOT_PATH.exists():
                response.user_list = []
                response.success = False
                response.message = f'Path {self.DEFAULT_SAVE_ROOT_PATH} does not exist.'
                return response

            folder_names = [
                name for name in os.listdir(self.DEFAULT_SAVE_ROOT_PATH)
                if (self.DEFAULT_SAVE_ROOT_PATH / name).is_dir()
            ]

            response.user_list = folder_names
            response.success = True
            response.message = f'Found {len(folder_names)} user(s).'

        except Exception as e:
            response.user_list = []
            response.success = False
            response.message = f'Error: {str(e)}'

        return response

    def get_dataset_list_callback(self, request, response):
        """
        Handle dataset list retrieval requests.
        
        Args:
            request: Service request
            response: Service response
            
        Returns:
            Service response
        """
        user_id = request.user_id
        user_path = self.DEFAULT_SAVE_ROOT_PATH / user_id

        try:
            if not user_path.exists() or not user_path.is_dir():
                response.dataset_list = []
                response.success = False
                response.message = f"User ID '{user_id}' does not exist at path: {user_path}"
                return response

            dataset_names = [
                name for name in os.listdir(user_path)
                if (user_path / name).is_dir()
            ]

            response.dataset_list = dataset_names
            response.success = True
            response.message = f"Found {len(dataset_names)} dataset(s) for user '{user_id}'."

        except Exception as e:
            response.dataset_list = []
            response.success = False
            response.message = f'Error: {str(e)}'

        return response

    def get_model_weight_list_callback(self, request, response):
        """
        Handle model weight list retrieval requests.
        
        Args:
            request: Service request
            response: Service response
            
        Returns:
            Service response
        """
        save_root_path = TrainingManager.get_weight_save_root_path()
        try:
            if not save_root_path.exists():
                response.success = False
                response.message = f'Path does not exist: {save_root_path}'
                response.model_weight_list = []
                return response

            model_folders = [
                f.name for f in save_root_path.iterdir()
                if f.is_dir()
            ]

            response.success = True
            response.message = f'Found {len(model_folders)} model weights'
            response.model_weight_list = model_folders

        except Exception as e:
            response.success = False
            response.message = f'Error: {str(e)}'
            response.model_weight_list = []

        return response
