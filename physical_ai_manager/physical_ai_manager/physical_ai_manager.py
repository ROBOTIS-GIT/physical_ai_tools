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
    MODE_COLLECTION = Communicator.MODE_COLLECTION
    MODE_INFERENCE = Communicator.MODE_INFERENCE

    def __init__(self):
        super().__init__('physical_ai_manager')

        self.get_ros_params()

        # Initialize latest data storage
        self.latest_observation = {}
        self.latest_action = {}

        # Initialize observation manager
        self.communicator = Communicator(
            node=self,
            operation_mode=self.operation_mode,
            params=self.params
        )

        # Initialize ROS2 Communication Manager
        self.communicator.init_subscribers()
        self.communicator.init_publishers()
        
        # Create service
        self.recording_cmd_service = self.create_service(
            SendRecordingCommand,
            'recording/command',
            self.user_interaction_callback
        )

        # Create data_collection_timer for periodic data collection with specified frequency
        self.timer_manager = TimerManager(
            node=self)

        self.timer_manager.set_timer(
            timer_name='data_collection',
            timer_frequency=self.data_collection_timer_frequency,
            callback_function=self.data_collection_timer_callback
        )
        self.timer_manager.set_timer(
            timer_name='data_inference',
            timer_frequency=self.data_inference_timer_frequency,
            callback_function=self.data_inference_timer_callback
        )

        self.data_converter = DataConverter()
        self.data_collection_config = None

        self.get_logger().info('PhysicalAIManager initialization completed')

    def get_ros_params(self):
        # Declare and get robot type and operation mode parameters
        self.declare_parameter('robot_type', 'ai_worker')
        self.declare_parameter('operation_mode', self.MODE_COLLECTION)
        self.declare_parameter('data_collection_timer_frequency', 30.0)  # Hz
        self.declare_parameter('data_inference_timer_frequency', 30.0)  # Hz

        self.robot_type = self.get_parameter('robot_type').value
        self.operation_mode = self.get_parameter('operation_mode').value
        self.data_collection_timer_frequency = self.get_parameter('data_collection_timer_frequency').value
        self.data_inference_timer_frequency = self.get_parameter('data_inference_timer_frequency').value

        self.get_logger().info(f'Robot type: {self.robot_type}')
        self.get_logger().info(f'Operation mode: {self.operation_mode}')
        self.get_logger().info(f'Data collection timer frequency: {self.data_collection_timer_frequency} Hz')
        self.get_logger().info(f'Data inference timer frequency: {self.data_inference_timer_frequency} Hz')

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
            robot_type=self.robot_type,
            param_names=param_names,
            default_value=['']
        )

        # Load parameters
        self.params = load_parameters(
            node=self,
            robot_type=self.robot_type,
            param_names=param_names
        )

        self.collect_joint_order_list = [
            f'collect_joint_order.{joint_name}' for joint_name in self.params['joint_list']
        ]

        self.inference_joint_order_list = [
            f'inference_joint_order.{joint_name}' for joint_name in self.params['joint_list']
        ]

        declare_parameters(
            node=self,
            robot_type=self.robot_type,
            param_names=self.collect_joint_order_list,
            default_value=['']
        )

        declare_parameters(
            node=self,
            robot_type=self.robot_type,
            param_names=self.inference_joint_order_list,
            default_value=['']
        )

        self.collect_joint_order_param = load_parameters(
            node=self,
            robot_type=self.robot_type,
            param_names=self.collect_joint_order_list
        )

        self.inference_joint_order_param = load_parameters(
            node=self,
            robot_type=self.robot_type,
            param_names=self.inference_joint_order_list
        )
        # Log loaded parameters
        log_parameters(self, self.params)
        log_parameters(self, self.collect_joint_order_param)
        log_parameters(self, self.inference_joint_order_param)

    def update_latest_data(self):
        image_data = {}
        follower_data = {}
        leader_data = {}

        image_msgs, follower_msgs, leader_msgs = self.communicator.get_latest_data()
        if image_msgs is None or follower_msgs is None:
            return None, None, None

        for key, value in image_msgs.items():
            image_data[key] = self.data_converter.compressed_image2cvmat(value)
            
        for key, value in follower_msgs.items():
            # self.get_logger().info(f'follower_msgs[key]: {key}, {self.collect_joint_order_param}')
            if value is not None:
                follower_data[key] = self.data_converter.joint_state2tensor_array(
                    value, self.collect_joint_order_param[f'collect_joint_order.{key}'])
                
        for key, value in leader_msgs.items():
            leader_data[key] = self.data_converter.joint_trajectory2tensor_array(
                value, self.collect_joint_order_param[f'collect_joint_order.{key}'])

        return image_data, follower_data, leader_data

    def send_action(self, action):
        joint_msgs = self.data_converter.tensor_array2joint_trajectory(
            action,
            self.inference_joint_order_param)
        self.communicator.send_action(joint_msgs)
        

    def data_collection_timer_callback(self):
        if self.data_saver.status == 'stop':
            self.get_logger().info('Recording stopped')
            self.status
            return
        camera_data, follower_data, leader_data = self.update_latest_data()
        
        self.get_logger().info(f'follower_data: {follower_data}')
        self.get_logger().info(f'leader_data: {leader_data}')

        if camera_data and follower_data and leader_data:
            self.data_saver.record(
                images=camera_data,
                state=follower_data,
                action=leader_data,
                joint_list=self.collect_joint_order_param[
                    'collect_joint_order.follower'])
        else:
            self.get_logger().info('No data to save')
    def data_inference_timer_callback(self):
        camera_data, follower_data, _ = self.update_latest_data()

        if camera_data is not None and follower_data is not None:
            self.latest_camera_data = camera_data
            self.latest_joint_data = follower_data
            
            
    def user_interaction_callback(self, request, response):
        if request.command == SendRecordingCommand.Request.START:
            self.get_logger().info('Starting recording with task: ' + request.task_name)
            self.data_saver = DataSaver(
                repo_id='ROBOTIS/aiworker_wholebody5',
                save_path=Path('/home/dongyun/.cache/huggingface/lerobot/ROBOTIS/aiworker_wholebody5'),
                task_instruction='TEST'
            )

            self.timer_manager.start(timer_name='data_collection')
            response.success = True
            response.message = "Recording started"
        elif request.command == SendRecordingCommand.Request.STOP:
            self.get_logger().info('Stopping recording')
            self.timer_manager.stop(timer_name='data_collection')
            self.data_saver.status = 'stop'
            response.success = True
            response.message = "Recording stopped"
        # 다른 명령 처리
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PhysicalAIManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
