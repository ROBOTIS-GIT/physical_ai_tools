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
# Author: Dongyun Kim

import glob
import os
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from physical_ai_interfaces.msg import TaskStatus
from physical_ai_interfaces.srv import (
    GetHFUser,
    GetRobotTypeList,
    SendCommand,
    SetHFUser,
    SetRobotType,
    )
from physical_ai_server.communication.communicator import Communicator
from physical_ai_server.data_processing.data_manager import DataManager
from physical_ai_server.timer.timer_manager import TimerManager
from physical_ai_server.utils.parameter_utils import (
    declare_parameters,
    load_parameters,
    log_parameters,
)

import rclpy
from rclpy.node import Node


class PhysicalAIServer(Node):
    # Define operation modes (constants taken from Communicator)

    def __init__(self):
        super().__init__('physical_ai_server')
        self.get_logger().info('Start Physical AI Server')

        self.communicator = None
        self.data_manager = None
        self.timer_manager = None

        self.params = None
        self.total_joint_order = None

        self.on_recording = False
        self.default_save_root_path = Path.home() / '.cache/huggingface/lerobot'

        self.init_ros_service()
        self.robot_type_list = self.get_robot_type_list()
        self.timer_callback_dict = {
            'collection': self.data_collection_timer_callback,
            'inference': self.inference_timer_callback
        }
        self.topic_timeout = 5  # seconds
        self.start_recording_time = 0

    def init_ros_service(self):
        self.get_logger().info('Initializing ROS services...')
        self.recording_cmd_service = self.create_service(
            SendCommand,
            '/task/command',
            self.user_interaction_callback
        )

        self.get_robot_types_service = self.create_service(
            GetRobotTypeList,
            '/get_robot_types',
            self.get_robot_types_callback
        )

        self.set_robot_type_service = self.create_service(
            SetRobotType,
            '/set_robot_type',
            self.set_robot_type_callback
        )

        self.set_hf_user_service = self.create_service(
            SetHFUser,
            '/register_hf_user',
            self.set_hf_user_callback
        )

        self.get_hf_user_service = self.create_service(
            GetHFUser,
            '/get_registered_hf_user',
            self.get_hf_user_callback
        )
        self.get_logger().info('ROS services initialized successfully')

    def init_ros_params(self, robot_type):
        self.get_logger().info(f'Initializing ROS parameters for robot type: {robot_type}')
        param_names = [
            'camera_topic_list',
            'joint_topic_list',
            'observation_list',
            'joint_list'
        ]

        # Declare parameters
        declare_parameters(
            node=self,
            robot_type=robot_type,
            param_names=param_names,
            default_value=['']
        )

        # Load parameters
        self.params = load_parameters(
            node=self,
            robot_type=robot_type,
            param_names=param_names
        )

        self.joint_order_list = [
            f'joint_order.{joint_name}' for joint_name in self.params['joint_list']
        ]

        declare_parameters(
            node=self,
            robot_type=robot_type,
            param_names=self.joint_order_list,
            default_value=['']
        )

        self.joint_order = load_parameters(
            node=self,
            robot_type=robot_type,
            param_names=self.joint_order_list
        )

        self.total_joint_order = []
        for joint_list in self.joint_order.values():
            self.total_joint_order.extend(joint_list)

        # Log loaded parameters
        log_parameters(self, self.params)
        log_parameters(self, self.joint_order)

        # Initialize observation manager
        self.communicator = Communicator(
            node=self,
            operation_mode=self.operation_mode,
            params=self.params
        )
        self.get_logger().info(
            f'ROS parameters initialized successfully for robot type: {robot_type}')

    def init_robot_control_parameters_from_user_task(
            self,
            task_info):
        self.get_logger().info(
            'Initializing robot control parameters from user task...')
        self.data_manager = DataManager(
            save_root_path=self.default_save_root_path,
            robot_type=self.robot_type,
            task_info=task_info
        )

        self.timer_manager = TimerManager(node=self)
        self.timer_manager.set_timer(
            timer_name=self.operation_mode,
            timer_frequency=task_info.fps,
            callback_function=self.timer_callback_dict[self.operation_mode]
        )
        self.timer_manager.start(timer_name=self.operation_mode)
        self.get_logger().info(
            'Robot control parameters initialized successfully')

    def clear_parameters(self):
        if self.communicator is not None:
            self.communicator.cleanup()
            self.communicator = None

        if self.timer_manager is not None:
            self.timer_manager = None

        self.params = None
        self.total_joint_order = None
        self.joint_order = None

    def set_hf_user_callback(self, request, response):
        request_hf_token = request.token
        if DataManager.register_huggingface_token(request_hf_token):
            self.get_logger().info('Hugging Face user token registered successfully')
            response.user_id_list = DataManager.get_huggingface_user_id()
            response.success = True
            response.message = 'Hugging Face user token registered successfully'
        else:
            self.get_logger().error('Failed to register Hugging Face user token')
            response.user_id_list = []
            response.success = False
            response.message = 'Failed to register token, Please check your token'
        return response

    def get_hf_user_callback(self, request, response):
        user_ids = DataManager.get_huggingface_user_id()
        if user_ids is not None:
            response.user_id_list = user_ids
            self.get_logger().info(f'Hugging Face user IDs: {user_ids}')
            response.success = True
            response.message = 'Hugging Face user IDs retrieved successfully'

        else:
            self.get_logger().error('Failed to retrieve Hugging Face user ID')
            response.user_id_list = []
            response.success = False
            response.message = 'Failed to retrieve Hugging Face user ID'

        return response

    def get_robot_type_list(self):
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

        self.get_logger().info(f'Available robot types: {robot_type_list}')
        return robot_type_list

    def data_collection_timer_callback(self):
        error_msg = ''
        current_status = TaskStatus()
        camera_msgs, follower_msgs, leader_msgs = self.communicator.get_latest_data()
        camera_data, follower_data, leader_data = self.data_manager.convert_msgs_to_raw_datas(
            camera_msgs,
            follower_msgs,
            leader_msgs,
            self.total_joint_order,
            self.joint_order)

        if (not camera_data or
                len(camera_data) != len(self.params['camera_topic_list'])):
            if time.perf_counter() - self.start_recording_time > self.topic_timeout:
                error_msg = 'Camera data not received within timeout period'
                self.get_logger().error(error_msg)
            else:
                self.get_logger().info('Waiting for camera data...')
                return

        elif not follower_data or len(follower_data) != len(self.total_joint_order):
            if time.perf_counter() - self.start_recording_time > self.topic_timeout:
                error_msg = 'Follower data not received within timeout period'
                self.get_logger().error(error_msg)
            else:
                self.get_logger().info('Waiting for follower data...')
                return

        elif not leader_data or len(leader_data) != len(self.total_joint_order):
            if time.perf_counter() - self.start_recording_time > self.topic_timeout:
                error_msg = 'Leader data not received within timeout period'
                self.get_logger().error(error_msg)
            else:
                self.get_logger().info('Waiting for leader data...')
                return

        elif not self.data_manager.check_lerobot_dataset(
                camera_data,
                self.total_joint_order):
            error_msg = 'Invalid repository name, Please change the repository name'
            self.get_logger().info(error_msg)

        if error_msg:
            self.on_recording = False
            current_status.phase = TaskStatus.READY
            current_status.error = error_msg
            self.communicator.publish_status(status=current_status)
            self.timer_manager.stop(timer_name=self.operation_mode)
            return

        record_completed = self.data_manager.record(
            images=camera_data,
            state=follower_data,
            action=leader_data)

        current_status = self.data_manager.get_current_record_status()
        self.communicator.publish_status(status=current_status)

        if record_completed:
            self.get_logger().info('Recording completed')
            current_status.phase = TaskStatus.READY
            current_status.proceed_time = int(0)
            current_status.total_time = int(0)
            self.communicator.publish_status(status=current_status)
            self.on_recording = False
            self.timer_manager.stop(timer_name=self.operation_mode)
            return

    def inference_timer_callback(self):
        # TODO: Implement inference timer callback logic
        camera_data = {}
        follower_data = []
        leader_data = []
        print(camera_data, follower_data, leader_data)

    def user_interaction_callback(self, request, response):
        try:
            if request.command == SendCommand.Request.START_RECORD:
                if self.on_recording:
                    self.get_logger().info('Restarting the recording.')
                    self.data_manager.re_record()
                    response.success = True
                    response.message = 'Restarting the recording.'
                    return response

                self.get_logger().info('Start recording')
                self.operation_mode = 'collection'
                task_info = request.task_info
                self.init_robot_control_parameters_from_user_task(
                    task_info
                )
                self.start_recording_time = time.perf_counter()
                self.on_recording = True
                response.success = True
                response.message = 'Recording started'

            elif request.command == SendCommand.Request.START_INFERENCE:
                # TODO: This is currently hardcoded to 'collection',
                # and support for 'inference' mode will be added in a future PR.
                self.operation_mode = 'inference'
                self.on_recording = False
                response.success = True
                response.message = 'Inference started'

            else:
                if not self.on_recording:
                    response.success = False
                    response.message = 'Not currently recording'
                else:
                    if request.command == SendCommand.Request.STOP:
                        self.get_logger().info('Stopping recording')
                        self.data_manager.record_stop()
                        response.success = True
                        response.message = 'Recording stopped'

                    elif request.command == SendCommand.Request.MOVE_TO_NEXT:
                        self.get_logger().info('Moving to next episode')
                        self.data_manager.record_early_save()
                        response.success = True
                        response.message = 'Moved to next episode'

                    elif request.command == SendCommand.Request.RERECORD:
                        self.get_logger().info('Re-recording current episode')
                        self.data_manager.re_record()
                        response.success = True
                        response.message = 'Re-recording current episode'

                    elif request.command == SendCommand.Request.FINISH:
                        self.get_logger().info('Terminating all operations')
                        self.data_manager.record_finish()
                        response.success = True
                        response.message = 'All operations terminated'

        except Exception as e:
            self.get_logger().error(f'Error in user interaction: {str(e)}')
            response.success = False
            response.message = f'Error in user interaction: {str(e)}'
            return response
        return response

    def get_robot_types_callback(self, request, response):
        if self.robot_type_list is None:
            self.get_logger().error('Robot type list is not set')
            response.robot_types = []
            response.success = False
            response.message = 'Robot type list is not set'
            return response

        self.get_logger().info(f'Available robot types: {self.robot_type_list}')
        response.robot_types = self.robot_type_list
        response.success = True
        response.message = 'Robot type list retrieved successfully'
        return response

    def set_robot_type_callback(self, request, response):
        try:
            self.get_logger().info(f'Setting robot type to: {request.robot_type}')
            self.operation_mode = 'collection'
            self.robot_type = request.robot_type
            self.clear_parameters()
            self.init_ros_params(self.robot_type)
            response.success = True
            response.message = f'Robot type set to {self.robot_type}'
            return response

        except Exception as e:
            self.get_logger().error(f'Failed to set robot type: {str(e)}')
            response.success = False
            response.message = f'Failed to set robot type: {str(e)}'
            return response


def main(args=None):
    rclpy.init(args=args)
    node = PhysicalAIServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
