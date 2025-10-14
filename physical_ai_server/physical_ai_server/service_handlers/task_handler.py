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

import time
from typing import TYPE_CHECKING, Optional

from physical_ai_interfaces.srv import SendCommand

from .base_handler import BaseServiceHandler

if TYPE_CHECKING:
    from physical_ai_server.communication.communicator import Communicator
    from physical_ai_server.data_processing.data_manager import DataManager
    from physical_ai_server.inference.inference_manager import InferenceManager
    from physical_ai_server.timer.timer_manager import TimerManager
    from rclpy.node import Node


class TaskServiceHandler(BaseServiceHandler):
    """
    Service handler for task control operations.

    Manages recording, inference, and robot type configuration.
    """

    def __init__(self, node: 'Node'):
        """
        Initialize task service handler.

        Args:
            node: ROS2 node instance (for logger and state access)
        """
        super().__init__(node)

    def set_managers(
        self,
        data_manager: Optional['DataManager'],
        inference_manager: Optional['InferenceManager'],
        communicator: Optional['Communicator'],
        timer_manager: Optional['TimerManager']
    ):
        """
        Set manager instances after initialization.

        Args:
            data_manager: Data manager instance
            inference_manager: Inference manager instance
            communicator: Communicator instance
            timer_manager: Timer manager instance
        """
        self.data_manager = data_manager
        self.inference_manager = inference_manager
        self.communicator = communicator
        self.timer_manager = timer_manager

    def user_interaction_callback(self, request, response):
        """
        Handle user task control commands.

        Args:
            request: SendCommand service request
            response: SendCommand service response

        Returns:
            response: Modified response with command execution status
        """
        try:
            if request.command == SendCommand.Request.START_RECORD:
                return self._handle_start_record(request, response)
            elif request.command == SendCommand.Request.START_INFERENCE:
                return self._handle_start_inference(request, response)
            else:
                return self._handle_recording_control(request, response)

        except Exception as e:
            self.logger.error(f'Error in user interaction: {str(e)}')
            return self._create_error_response(
                response,
                f'Error in user interaction: {str(e)}'
            )

    def _handle_start_record(self, request, response):
        """
        Handle start recording command.

        Args:
            request: SendCommand service request
            response: SendCommand service response

        Returns:
            response: Modified response with start status
        """
        if self.node.on_recording:
            self.logger.info('Restarting the recording.')
            self.data_manager.re_record()
            return self._create_success_response(
                response,
                'Restarting the recording.'
            )

        self.logger.info('Start recording')
        self.node.operation_mode = 'collection'
        task_info = request.task_info

        # Initialize robot control parameters
        self._init_robot_control_parameters(task_info)

        self.node.start_recording_time = time.perf_counter()
        self.node.on_recording = True

        return self._create_success_response(
            response,
            'Recording started'
        )

    def _handle_start_inference(self, request, response):
        """
        Handle start inference command.

        Args:
            request: SendCommand service request
            response: SendCommand service response

        Returns:
            response: Modified response with start status
        """
        self.node.joint_topic_types = (
            self.communicator.get_publisher_msg_types()
        )
        self.node.operation_mode = 'inference'
        task_info = request.task_info
        self.node.task_instruction = task_info.task_instruction

        valid_result, result_message = (
            self.inference_manager.validate_policy(
                policy_path=task_info.policy_path
            )
        )

        if not valid_result:
            self.logger.error(result_message)
            return self._create_error_response(response, result_message)

        self._init_robot_control_parameters(task_info)

        if task_info.record_inference_mode:
            self.node.on_recording = True

        self.node.on_inference = True
        self.node.start_recording_time = time.perf_counter()

        return self._create_success_response(
            response,
            'Inference started'
        )

    def _handle_recording_control(self, request, response):
        """
        Handle recording control commands (stop, next, rerecord, etc).

        Args:
            request: SendCommand service request
            response: SendCommand service response

        Returns:
            response: Modified response with command status
        """
        if not self.node.on_recording and not self.node.on_inference:
            return self._create_error_response(
                response,
                'Not currently recording'
            )

        if request.command == SendCommand.Request.STOP:
            self.logger.info('Stopping recording')
            self.data_manager.record_stop()
            return self._create_success_response(
                response,
                'Recording stopped'
            )

        elif request.command == SendCommand.Request.MOVE_TO_NEXT:
            self.logger.info('Moving to next episode')
            if len(request.task_info.task_instruction) > 1:
                self.data_manager.record_next_episode()
            else:
                self.data_manager.record_early_save()
            return self._create_success_response(
                response,
                'Moved to next episode'
            )

        elif request.command == SendCommand.Request.RERECORD:
            self.logger.info('Re-recording current episode')
            self.data_manager.re_record()
            return self._create_success_response(
                response,
                'Re-recording current episode'
            )

        elif request.command == SendCommand.Request.FINISH:
            self.logger.info('Terminating all operations')
            self.data_manager.record_finish()
            self.node.on_inference = False
            return self._create_success_response(
                response,
                'All operations terminated'
            )

        elif request.command == SendCommand.Request.SKIP_TASK:
            self.logger.info('Skipping task')
            self.data_manager.record_skip_task()
            return self._create_success_response(
                response,
                'Task skipped successfully'
            )

        return self._create_error_response(
            response,
            f'Unknown command: {request.command}'
        )

    def set_robot_type_callback(self, request, response):
        """
        Set robot type and initialize parameters.

        Args:
            request: SetRobotType service request
            response: SetRobotType service response

        Returns:
            response: Modified response with set status
        """
        try:
            self.logger.info(f'Setting robot type to: {request.robot_type}')
            self.node.operation_mode = 'collection'
            self.node.robot_type = request.robot_type

            # Clear and reinitialize parameters
            self._clear_parameters()
            self._init_ros_params(request.robot_type)

            return self._create_success_response(
                response,
                f'Robot type set to {request.robot_type}'
            )

        except Exception as e:
            self.logger.error(f'Failed to set robot type: {str(e)}')
            return self._create_error_response(
                response,
                f'Failed to set robot type: {str(e)}'
            )

    def _init_robot_control_parameters(self, task_info):
        """
        Initialize robot control parameters from task info.

        This method is called by the main server through a callback.
        """
        # This will be set by the main server
        if hasattr(self, 'init_robot_control_callback'):
            self.init_robot_control_callback(task_info)

    def _clear_parameters(self):
        """
        Clear robot parameters.

        This method is called by the main server through a callback.
        """
        # This will be set by the main server
        if hasattr(self, 'clear_parameters_callback'):
            self.clear_parameters_callback()

    def _init_ros_params(self, robot_type):
        """
        Initialize ROS parameters for robot type.

        This method is called by the main server through a callback.
        """
        # This will be set by the main server
        if hasattr(self, 'init_ros_params_callback'):
            self.init_ros_params_callback(robot_type)
