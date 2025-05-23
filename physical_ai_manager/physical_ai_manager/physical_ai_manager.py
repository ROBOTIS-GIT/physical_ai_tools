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

import cv2
from pathlib import Path

from physical_ai_manager.communication.communicator import Communicator
from physical_ai_manager.data_processing.data_converter import DataConverter
from physical_ai_manager.utils.parameter_utils import (
    declare_parameters,
    load_parameters,
    log_parameters,
)
from physical_ai_manager.data_processing.data_saver import DataSaver
import rclpy
from rclpy.node import Node
from physical_ai_interfaces.srv import SendRecordingCommand
from physical_ai_manager.timer.timer_manager import TimerManager


class PhysicalAIManager(Node):
    # Define operation modes (constants taken from Communicator)

    def __init__(self):
        super().__init__('physical_ai_manager')

        # Create service
        self.recording_cmd_service = self.create_service(
            SendRecordingCommand,
            'recording/command',
            self.user_interaction_callback
        )

        self.communicator = None
        self.timer_manager = None
        self.data_converter = None
        self.data_collection_config = None
        self.params = None
        self.joint_order = None
        self.total_joint_order = None
        self.default_save_root_path = Path.home() / '.cache/huggingface/lerobot'
        
    def init_robot_control_parameters_from_user_task(
            self,
            robot_type,
            operation_mode,
            timer_frequency):
        self.get_ros_params(robot_type)

        # Initialize observation manager
        self.communicator = Communicator(
            node=self,
            operation_mode=operation_mode,
            params=self.params
        )

        # Create data_collection_timer for periodic data collection with specified frequency
        self.timer_manager = TimerManager(
            node=self)

        self.timer_manager.set_timer(
            timer_name=operation_mode,
            timer_frequency=timer_frequency,
            callback_function=self.data_collection_timer_callback
        )

        self.data_converter = DataConverter()
        self.data_collection_config = None
    
    def clear_robot_control_parameters(self):
        self.communicator = None
        self.timer_manager = None
        self.data_converter = None
        self.data_collection_config = None
        self.params = None
        self.joint_order = None
        self.total_joint_order = None

    def get_ros_params(self, robot_type):
        # Define parameter names to load
        param_names = [
            'camera_topic_list',
            'joint_sub_topic_list',
            'joint_pub_topic_list',
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

    def update_latest_data(self):
        image_data = {}
        follower_data = []
        leader_data = []

        image_msgs, follower_msgs, leader_msgs = self.communicator.get_latest_data()

        for key, value in image_msgs.items():
            image_data[key] = cv2.cvtColor(
                self.data_converter.compressed_image2cvmat(value),
                cv2.COLOR_BGR2RGB)

        for key, value in follower_msgs.items():
            if value is not None:
                follower_data.extend(self.data_converter.joint_state2tensor_array(
                    value, self.total_joint_order))

        for key, value in leader_msgs.items():
            leader_data.extend(self.data_converter.joint_trajectory2tensor_array(
                value, self.joint_order[f'joint_order.{key}']))

        return image_data, follower_data, leader_data

    def send_action(self, action):
        joint_msgs = self.data_converter.tensor_array2joint_trajectory(
            action,
            self.total_joint_order)
        self.communicator.send_action(joint_msgs)

    def data_collection_timer_callback(self):
        import time
        if self.data_saver.status == 'stop':
            self.get_logger().info('Recording stopped')
            return

        start_time = time.perf_counter()
        camera_data, follower_data, leader_data = self.update_latest_data()

        if len(camera_data) != len(self.params['camera_topic_list']):
            return

        if camera_data and follower_data and leader_data:
            record_completed = self.data_saver.record(
                images=camera_data,
                state=follower_data,
                action=leader_data,
                joint_list=self.total_joint_order)

            if record_completed:
                self.get_logger().info('Recording stopped')
                self.timer_manager.stop(timer_name=self.operation_mode)
                return

            end_time = time.perf_counter()
            fps = round(1 / (end_time - start_time), 1)
            self.get_logger().info(f'Record FPS: {fps}')
        else:
            self.get_logger().info('No data to save')

    def inference_timer_callback(self):
        camera_data, follower_data, _ = self.update_latest_data()

        if camera_data and follower_data:
            self.latest_camera_data = camera_data
            self.latest_joint_data = follower_data

    def user_interaction_callback(self, request, response):
        robot_type = request.robot_type
        repo_id = request.repo_id
        task_instruction = request.task_instruction
        
        timer_frequency = request.frequency
        episode_time = request.episode_time
        episode_num = request.episode_num
        warmup_time = request.warmup_time
        reset_time = request.reset_time
        
        save_path = self.default_save_root_path / repo_id
        self.get_logger().info(f'Save path: {save_path}')
            
        if request.command == SendRecordingCommand.Request.START_RECORD:
            self.get_logger().info('Starting recording with task: ' + request.task_name)
            self.operation_mode = 'collection'
            self.init_robot_control_parameters_from_user_task(
                robot_type,
                self.operation_mode,
                timer_frequency
            )

            self.data_saver = DataSaver(
                repo_id=repo_id,
                save_fps=timer_frequency,
                save_path=save_path,
                task_instruction=task_instruction
            )

            self.timer_manager.start(timer_name=self.operation_mode)
            response.success = True
            response.message = "Recording started"

        elif request.command == SendRecordingCommand.Request.STOP:
            self.get_logger().info('Stopping recording')
            if self.timer_manager is not None:
                self.timer_manager.stop(timer_name=self.operation_mode)
                self.data_saver.record_stop()
            response.success = True
            response.message = "Recording stopped"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PhysicalAIManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
