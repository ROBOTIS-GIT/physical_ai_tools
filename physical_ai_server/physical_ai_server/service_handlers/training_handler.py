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
import threading
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rclpy.node import Node
    from rclpy.publisher import Publisher
    from physical_ai_server.timer.timer_manager import TimerManager
    from physical_ai_server.training.training_manager import TrainingManager

from physical_ai_interfaces.msg import TrainingStatus
from physical_ai_interfaces.srv import SendTrainingCommand

from .base_handler import BaseServiceHandler


class TrainingServiceHandler(BaseServiceHandler):
    """
    Service handler for training operations.

    Manages training start, stop, and status monitoring for policy training.
    """

    TRAINING_STATUS_TIMER_FREQUENCY = 0.5  # seconds
    DEFAULT_SAVE_ROOT_PATH = Path.home() / '.cache/huggingface/lerobot'

    def __init__(self,
                 node: 'Node',
                 training_status_publisher: 'Publisher',
                 training_manager: 'TrainingManager',
                 training_timer: 'TimerManager'):
        """
        Initialize training service handler.

        Args:
            node: ROS2 node instance
            training_status_publisher: Publisher for training status updates
            training_manager: TrainingManager instance
            training_timer: TimerManager instance for training status updates
        """
        super().__init__(node)
        self.training_status_publisher = training_status_publisher
        self.training_manager = training_manager
        self.training_timer = training_timer
        self.training_thread = None
        self.is_training = False

    def user_training_interaction_callback(self, request, response):
        """
        Handle training control commands (start/stop).

        Args:
            request: SendTrainingCommand service request
            response: SendTrainingCommand service response

        Returns:
            response: Modified response with command execution status
        """
        try:
            if request.command == SendTrainingCommand.Request.START:
                return self._handle_training_start(request, response)
            elif request.command == SendTrainingCommand.Request.FINISH:
                return self._handle_training_finish(request, response)
            else:
                return self._create_error_response(
                    response,
                    f'Unknown training command: {request.command}'
                )

        except Exception as e:
            self.logger.error(f'Error in user_training_interaction: {str(e)}')
            return self._create_error_response(
                response,
                f'Error in user_training_interaction: {str(e)}'
            )

    def _handle_training_start(self, request, response):
        """
        Handle training start command.

        Args:
            request: SendTrainingCommand service request
            response: SendTrainingCommand service response

        Returns:
            response: Modified response with start status
        """
        if self.training_thread and self.training_thread.is_alive():
            return self._create_error_response(
                response,
                'Training is already in progress'
            )

        # Setup training status timer
        self.training_timer.set_timer(
            timer_name='training_status',
            timer_frequency=self.TRAINING_STATUS_TIMER_FREQUENCY,
            callback_function=lambda: self.training_status_publisher.publish(
                self._get_training_status()
            )
        )
        self.training_timer.start(timer_name='training_status')

        output_folder_name = request.training_info.output_folder_name
        weight_save_root_path = TrainingManager.get_weight_save_root_path()
        self.logger.info(
            f'Weight save root path: {weight_save_root_path}, '
            f'Output folder name: {output_folder_name}'
        )

        output_path = weight_save_root_path / output_folder_name
        if output_path.exists():
            self.is_training = False
            training_status = self._get_training_status()
            self.training_status_publisher.publish(training_status)
            self.training_manager.stop_event.set()
            self.training_timer.stop('training_status')
            return self._create_error_response(
                response,
                f'Output folder already exists: {output_path}'
            )

        self.training_manager.training_info = request.training_info

        def run_training():
            try:
                self.training_manager.train()
            finally:
                self.is_training = False
                self.logger.info('Training completed.')
                training_status = self._get_training_status()
                self.training_status_publisher.publish(training_status)
                self.training_manager.stop_event.set()
                self.training_timer.stop('training_status')

        self.training_thread = threading.Thread(
            target=run_training,
            daemon=True
        )
        self.training_thread.start()
        self.is_training = True

        return self._create_success_response(
            response,
            'Training started successfully'
        )

    def _handle_training_finish(self, request, response):
        """
        Handle training finish/stop command.

        Args:
            request: SendTrainingCommand service request
            response: SendTrainingCommand service response

        Returns:
            response: Modified response with stop status
        """
        self.is_training = False
        training_status = self._get_training_status()
        self.training_status_publisher.publish(training_status)
        self.training_timer.stop('training_status')

        if self.training_thread and self.training_thread.is_alive():
            self.training_manager.stop_event.set()
            self.training_thread.join()
            return self._create_success_response(
                response,
                'Training stopped successfully'
            )
        else:
            return self._create_error_response(
                response,
                'No training in progress to stop'
            )

    def _get_training_status(self):
        """
        Get current training status.

        Returns:
            TrainingStatus: Current training status message
        """
        msg = TrainingStatus()
        if self.training_manager is None:
            return msg

        try:
            current_status = (
                self.training_manager.get_current_training_status()
            )
            training_info = current_status.training_info
            current_step = current_status.current_step
            current_loss = current_status.current_loss
            msg.training_info = training_info
            msg.current_step = current_step
            msg.current_loss = current_loss
            msg.is_training = self.is_training
            msg.error = ''
        except Exception as e:
            msg.current_step = 0
            msg.current_loss = float('nan')
            msg.error = str(e)
            self.logger.error(f'Error getting training status: {msg.error}')

        return msg

    def get_available_list_callback(self, request, response):
        """
        Get available policy and device lists for training.

        Args:
            request: GetPolicyList service request
            response: GetPolicyList service response

        Returns:
            response: Modified response with available lists
        """
        response.policy_list, response.device_list = (
            TrainingManager.get_available_list()
        )
        return self._create_success_response(
            response,
            'Policy and device lists retrieved successfully'
        )

    def get_user_list_callback(self, request, response):
        """
        Get list of users from dataset directory.

        Args:
            request: GetUserList service request
            response: GetUserList service response

        Returns:
            response: Modified response with user list
        """
        try:
            if not self.DEFAULT_SAVE_ROOT_PATH.exists():
                response.user_list = []
                return self._create_error_response(
                    response,
                    f'Path {self.DEFAULT_SAVE_ROOT_PATH} does not exist.'
                )

            folder_names = [
                name for name in os.listdir(self.DEFAULT_SAVE_ROOT_PATH)
                if (self.DEFAULT_SAVE_ROOT_PATH / name).is_dir()
            ]

            response.user_list = folder_names
            return self._create_success_response(
                response,
                f'Found {len(folder_names)} user(s).'
            )

        except Exception as e:
            response.user_list = []
            return self._create_error_response(
                response,
                f'Error: {str(e)}'
            )

    def get_dataset_list_callback(self, request, response):
        """
        Get list of datasets for a specific user.

        Args:
            request: GetDatasetList service request
            response: GetDatasetList service response

        Returns:
            response: Modified response with dataset list
        """
        user_id = request.user_id
        user_path = self.DEFAULT_SAVE_ROOT_PATH / user_id

        try:
            if not user_path.exists() or not user_path.is_dir():
                response.dataset_list = []
                return self._create_error_response(
                    response,
                    f"User ID '{user_id}' does not exist at path: {user_path}"
                )

            dataset_names = [
                name for name in os.listdir(user_path)
                if (user_path / name).is_dir()
            ]

            response.dataset_list = dataset_names
            return self._create_success_response(
                response,
                f"Found {len(dataset_names)} dataset(s) for user '{user_id}'."
            )

        except Exception as e:
            response.dataset_list = []
            return self._create_error_response(
                response,
                f'Error: {str(e)}'
            )

    def get_model_weight_list_callback(self, request, response):
        """
        Get list of saved model weights.

        Args:
            request: GetModelWeightList service request
            response: GetModelWeightList service response

        Returns:
            response: Modified response with model weight list
        """
        save_root_path = TrainingManager.get_weight_save_root_path()
        try:
            if not save_root_path.exists():
                response.model_weight_list = []
                return self._create_error_response(
                    response,
                    f'Path does not exist: {save_root_path}'
                )

            model_folders = [
                f.name for f in save_root_path.iterdir()
                if f.is_dir()
            ]

            response.model_weight_list = model_folders
            return self._create_success_response(
                response,
                f'Found {len(model_folders)} model weights'
            )

        except Exception as e:
            response.model_weight_list = []
            return self._create_error_response(
                response,
                f'Error: {str(e)}'
            )

    def cleanup(self):
        """Cleanup handler resources on shutdown."""
        if self.is_training and self.training_manager:
            self.training_manager.stop_event.set()
        if self.training_timer:
            self.training_timer.stop('training_status')
