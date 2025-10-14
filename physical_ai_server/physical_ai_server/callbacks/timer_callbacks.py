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

from physical_ai_interfaces.msg import TaskStatus

if TYPE_CHECKING:
    from physical_ai_server.communication.communicator import Communicator
    from physical_ai_server.data_processing.data_manager import DataManager
    from physical_ai_server.inference.inference_manager import InferenceManager
    from physical_ai_server.timer.timer_manager import TimerManager
    from rclpy.node import Node


class TimerCallbacks:
    """
    Timer callback manager for periodic operations.

    Handles data collection, inference, and joystick trigger callbacks.
    """

    DEFAULT_TOPIC_TIMEOUT = 5.0  # seconds

    def __init__(
        self,
        node: 'Node',
        params: dict,
        total_joint_order: list,
        joint_order: dict
    ):
        """
        Initialize timer callbacks.

        Args:
            node: ROS2 node instance (for logger and state access)
            params: Robot configuration parameters
            total_joint_order: Complete joint order list
            joint_order: Joint order dictionary
        """
        self.node = node
        self.logger = node.get_logger()
        self.params = params
        self.total_joint_order = total_joint_order
        self.joint_order = joint_order

    def set_managers(
        self,
        data_manager: Optional['DataManager'],
        communicator: Optional['Communicator'],
        inference_manager: Optional['InferenceManager'],
        timer_manager: Optional['TimerManager']
    ):
        """
        Set manager instances after initialization.

        Args:
            data_manager: Data manager instance
            communicator: Communicator instance
            inference_manager: Inference manager instance
            timer_manager: Timer manager instance
        """
        self.data_manager = data_manager
        self.communicator = communicator
        self.inference_manager = inference_manager
        self.timer_manager = timer_manager

    def data_collection_timer_callback(self):
        """Timer callback for data collection during recording."""
        error_msg = ''
        current_status = TaskStatus()

        camera_msgs, follower_msgs, leader_msgs = (
            self.communicator.get_latest_data()
        )

        # Check for camera data
        if camera_msgs is None:
            if (time.perf_counter() -
                    self.node.start_recording_time >
                    self.DEFAULT_TOPIC_TIMEOUT):
                error_msg = (
                    'Camera data not received within timeout period'
                )
                self.logger.error(error_msg)
            else:
                self.logger.info('Waiting for camera data...')
                return

        # Check for follower data
        elif follower_msgs is None:
            if (time.perf_counter() -
                    self.node.start_recording_time >
                    self.DEFAULT_TOPIC_TIMEOUT):
                error_msg = (
                    'Follower data not received within timeout period'
                )
                self.logger.error(error_msg)
            else:
                self.logger.info('Waiting for follower data...')
                return

        # Check for leader data
        elif leader_msgs is None:
            if (time.perf_counter() -
                    self.node.start_recording_time >
                    self.DEFAULT_TOPIC_TIMEOUT):
                error_msg = (
                    'Leader data not received within timeout period'
                )
                self.logger.error(error_msg)
            else:
                self.logger.info('Waiting for leader data...')
                return

        try:
            camera_data, follower_data, leader_data = (
                self.data_manager.convert_msgs_to_raw_datas(
                    camera_msgs,
                    follower_msgs,
                    self.total_joint_order,
                    leader_msgs,
                    self.joint_order
                )
            )

        except Exception as e:
            error_msg = (
                f'Failed to convert messages: {str(e)}, '
                f'please check the robot type again!'
            )
            self.node.on_recording = False
            current_status.phase = TaskStatus.READY
            current_status.error = error_msg
            self.communicator.publish_status(status=current_status)
            self.timer_manager.stop(timer_name=self.node.operation_mode)
            return

        if not self.data_manager.check_lerobot_dataset(
                camera_data,
                self.total_joint_order):
            error_msg = (
                'Invalid repository name, '
                'Please change the repository name'
            )
            self.logger.info(error_msg)

        if error_msg:
            self.node.on_recording = False
            current_status.phase = TaskStatus.READY
            current_status.error = error_msg
            self.communicator.publish_status(status=current_status)
            self.timer_manager.stop(timer_name=self.node.operation_mode)
            return

        # Handle joystick triggers
        if self.communicator.joystick_state['updated']:
            self.handle_joystick_trigger(
                joystick_mode=self.communicator.joystick_state['mode']
            )
            self.communicator.joystick_state['updated'] = False

        # Record data
        record_completed = self.data_manager.record(
            images=camera_data,
            state=follower_data,
            action=leader_data
        )

        current_status = self.data_manager.get_current_record_status()
        self.communicator.publish_status(status=current_status)

        if record_completed:
            self.logger.info('Recording completed')
            current_status.phase = TaskStatus.READY
            current_status.proceed_time = int(0)
            current_status.total_time = int(0)
            self.communicator.publish_status(status=current_status)
            self.node.on_recording = False
            self.timer_manager.stop(timer_name=self.node.operation_mode)

    def inference_timer_callback(self):
        """Timer callback for inference operations."""
        error_msg = ''
        current_status = TaskStatus()

        camera_msgs, follower_msgs, _ = self.communicator.get_latest_data()

        if (camera_msgs is None or
                len(camera_msgs) != len(self.params['camera_topic_list'])):
            self.logger.info('Waiting for camera data...')
            return
        elif follower_msgs is None:
            self.logger.info('Waiting for follower data...')
            return

        try:
            camera_data, follower_data, _ = (
                self.data_manager.convert_msgs_to_raw_datas(
                    camera_msgs,
                    follower_msgs,
                    self.total_joint_order
                )
            )
        except Exception as e:
            error_msg = (
                f'Failed to convert messages: {str(e)}, '
                f'please check the robot type again!'
            )
            self.node.on_inference = False
            current_status.phase = TaskStatus.READY
            current_status.error = error_msg
            self.communicator.publish_status(status=current_status)
            self.inference_manager.clear_policy()
            self.timer_manager.stop(timer_name=self.node.operation_mode)
            return

        if self.inference_manager.policy is None:
            if not self.inference_manager.load_policy():
                self.logger.error('Failed to load policy')
                return

        try:
            if not self.node.on_inference:
                self.logger.info('Inference mode is not active')
                current_status = self.data_manager.get_current_record_status()
                current_status.phase = TaskStatus.READY
                self.communicator.publish_status(status=current_status)
                self.inference_manager.clear_policy()
                self.timer_manager.stop(timer_name=self.node.operation_mode)
                return

            action = self.inference_manager.predict(
                images=camera_data,
                state=follower_data,
                task_instruction=self.node.task_instruction[0]
            )

            self.logger.info(f'Action data: {action}')
            action_pub_msgs = (
                self.data_manager.data_converter.tensor_array2joint_msgs(
                    action,
                    self.node.joint_topic_types,
                    self.joint_order
                )
            )

            self.communicator.publish_action(joint_msg_datas=action_pub_msgs)
            current_status = self.data_manager.get_current_record_status()
            current_status.phase = TaskStatus.INFERENCING
            self.communicator.publish_status(status=current_status)

        except Exception as e:
            self.logger.error(f'Inference failed, please check : {str(e)}')
            error_msg = f'Inference failed, please check : {str(e)}'
            self.node.on_recording = False
            self.node.on_inference = False
            current_status = self.data_manager.get_current_record_status()
            current_status.phase = TaskStatus.READY
            current_status.error = error_msg
            self.communicator.publish_status(status=current_status)
            self.inference_manager.clear_policy()
            self.timer_manager.stop(timer_name=self.node.operation_mode)

    def handle_joystick_trigger(self, joystick_mode: str):
        """
        Handle joystick trigger events.

        Args:
            joystick_mode: Joystick mode string (left, right, etc.)
        """
        self.logger.info(f'Joystick mode updated: {joystick_mode}')

        if self.data_manager is None:
            self.logger.warning('Data manager is not initialized')
            return

        if not self.node.on_recording:
            self.logger.warning('Not currently recording')
            return

        if joystick_mode == 'right':
            self.logger.info('Right tact triggered - Moving to next episode')
            task_info = self.data_manager.get_task_info()
            if len(task_info.task_instruction) > 1:
                self.data_manager.record_next_episode()
            else:
                self.data_manager.record_early_save()

        elif joystick_mode == 'left':
            self.logger.info('Left tact triggered - Re-record current episode')
            self.data_manager.re_record()

        elif joystick_mode == 'right_long_time':
            self.logger.info('Right long tact triggered - Custom')
            # Custom functionality can be added here

        elif joystick_mode == 'left_long_time':
            self.logger.info('Left long tact triggered - Custom')
            # Custom functionality can be added here

        else:
            self.logger.info(f'Received joystick trigger: {joystick_mode}')
