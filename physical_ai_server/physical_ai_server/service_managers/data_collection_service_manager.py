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

from typing import Any, Optional
from rclpy.node import Node

from physical_ai_interfaces.srv import SendCommand, SetRobotType
from physical_ai_server.service_managers.base_service_manager import BaseServiceManager


class DataCollectionServiceManager(BaseServiceManager):
    """
    Service manager for data collection related operations.
    
    Manages services for:
    - Task commands (start/stop recording, inference, etc.)
    - Robot type configuration
    """
    
    def __init__(self, node: Node, main_server: Any):
        """
        Initialize the data collection service manager.
        
        Args:
            node: ROS2 node instance
            main_server: Reference to the main PhysicalAIServer instance
        """
        super().__init__(node)
        self.main_server = main_server
    
    def initialize_services(self) -> None:
        """Initialize data collection related services."""
        self.logger.info('Initializing data collection services...')
        
        service_definitions = [
            ('/task/command', SendCommand, self.user_interaction_callback),
            ('/set_robot_type', SetRobotType, self.set_robot_type_callback),
        ]
        
        self.register_services(service_definitions)
        self.logger.info('Data collection services initialized successfully')
    
    def user_interaction_callback(self, request, response):
        """
        Handle user interaction commands for data collection and inference.
        
        Args:
            request: Service request
            response: Service response
            
        Returns:
            Service response
        """
        try:
            if request.command == SendCommand.Request.START_RECORD:
                if self.main_server.on_recording:
                    self.logger.info('Restarting the recording.')
                    self.main_server.data_manager.re_record()
                    response.success = True
                    response.message = 'Restarting the recording.'
                    return response

                self.logger.info('Start recording')
                self.main_server.operation_mode = 'collection'
                task_info = request.task_info
                self.main_server.init_robot_control_parameters_from_user_task(
                    task_info
                )

                import time
                self.main_server.start_recording_time = time.perf_counter()
                self.main_server.on_recording = True
                response.success = True
                response.message = 'Recording started'

            elif request.command == SendCommand.Request.START_INFERENCE:
                self.main_server.joint_topic_types = self.main_server.communicator.get_publisher_msg_types()
                self.main_server.operation_mode = 'inference'
                task_info = request.task_info
                self.main_server.task_instruction = task_info.task_instruction

                valid_result, result_message = self.main_server.inference_manager.validate_policy(
                    policy_path=task_info.policy_path)

                if not valid_result:
                    response.success = False
                    response.message = result_message
                    self.logger.error(response.message)
                    return response

                self.main_server.init_robot_control_parameters_from_user_task(
                    task_info
                )
                if task_info.record_inference_mode:
                    self.main_server.on_recording = True
                self.main_server.on_inference = True
                import time
                self.main_server.start_recording_time = time.perf_counter()
                response.success = True
                response.message = 'Inference started'

            else:
                if not self.main_server.on_recording and not self.main_server.on_inference:
                    response.success = False
                    response.message = 'Not currently recording'
                else:
                    if request.command == SendCommand.Request.STOP:
                        self.logger.info('Stopping recording')
                        self.main_server.data_manager.record_stop()
                        response.success = True
                        response.message = 'Recording stopped'

                    elif request.command == SendCommand.Request.MOVE_TO_NEXT:
                        self.logger.info('Moving to next episode')
                        if len(request.task_info.task_instruction) > 1:
                            self.main_server.data_manager.record_next_episode()
                        else:
                            self.main_server.data_manager.record_early_save()
                        response.success = True
                        response.message = 'Moved to next episode'

                    elif request.command == SendCommand.Request.RERECORD:
                        self.logger.info('Re-recording current episode')
                        self.main_server.data_manager.re_record()
                        response.success = True
                        response.message = 'Re-recording current episode'

                    elif request.command == SendCommand.Request.FINISH:
                        self.logger.info('Terminating all operations')
                        self.main_server.data_manager.record_finish()
                        self.main_server.on_inference = False
                        response.success = True
                        response.message = 'All operations terminated'

                    elif request.command == SendCommand.Request.SKIP_TASK:
                        self.logger.info('Skipping task')
                        self.main_server.data_manager.record_skip_task()
                        response.success = True
                        response.message = 'Task skipped successfully'

        except Exception as e:
            self.logger.error(f'Error in user interaction: {str(e)}')
            response.success = False
            response.message = f'Error in user interaction: {str(e)}'
            return response
        return response

    def set_robot_type_callback(self, request, response):
        """
        Handle robot type setting requests.
        
        Args:
            request: Service request
            response: Service response
            
        Returns:
            Service response
        """
        try:
            self.logger.info(f'Setting robot type to: {request.robot_type}')
            self.main_server.operation_mode = 'collection'
            self.main_server.robot_type = request.robot_type
            self.main_server.clear_parameters()
            self.main_server.init_ros_params(self.main_server.robot_type)
            response.success = True
            response.message = f'Robot type set to {self.main_server.robot_type}'
            return response

        except Exception as e:
            self.logger.error(f'Failed to set robot type: {str(e)}')
            response.success = False
            response.message = f'Failed to set robot type: {str(e)}'
            return response
