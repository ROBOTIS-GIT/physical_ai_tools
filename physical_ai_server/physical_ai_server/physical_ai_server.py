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

from physical_ai_server.communication.communicator import Communicator
from physical_ai_server.data_processing.data_converter import DataConverter
from physical_ai_server.utils.parameter_utils import (
    declare_parameters,
    load_parameters,
    log_parameters,
)
from physical_ai_server.data_processing.data_manager import DataManager
import rclpy
from rclpy.node import Node
from physical_ai_interfaces.srv import SendRecordingCommand
from physical_ai_server.timer.timer_manager import TimerManager


class PhysicalAIServer(Node):
    # Define operation modes (constants taken from Communicator)

    def __init__(self):
        super().__init__('physical_ai_server')

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

    def update_latest_data(
            self,
            camera_data: dict,
            follower_data: list,
            leader_data: list) -> bool:

        image_msgs, follower_msgs, leader_msgs = self.communicator.get_latest_data()
        
        if image_msgs is not None and follower_msgs is not None:
            for key, value in image_msgs.items():
                camera_data[key] = cv2.cvtColor(
                    self.data_converter.compressed_image2cvmat(value),
                    cv2.COLOR_BGR2RGB)

            for key, value in follower_msgs.items():
                if value is not None:
                    follower_data.extend(self.data_converter.joint_state2tensor_array(
                        value, self.total_joint_order))

            if self.operation_mode == 'collection':
                if leader_msgs is not None:
                    for key, value in leader_msgs.items():
                        leader_data.extend(self.data_converter.joint_trajectory2tensor_array(
                            value, self.joint_order[f'joint_order.{key}']))
                else:
                    self.get_logger().error('Leader topic is not found')
                    return False
        else:
            self.get_logger().error('Camera or Follower topic is not found')
            return False

        if len(camera_data) != len(self.params['camera_topic_list']):
            self.get_logger().error(
                f'Camera data length does not match the number of camera topics: {len(camera_data)} != {len(self.params["camera_topic_list"])}')
            return False
        
        if len(self.total_joint_order) != len(follower_data):
            self.get_logger().error(
                'Follower data length does not match the number of joints')
            return False
        
        if len(self.total_joint_order) != len(leader_data):
            self.get_logger().error(
                'Leader data length does not match the number of joints')
            return False

        return True

    def send_action(self, action):
        joint_msgs = self.data_converter.tensor_array2joint_trajectory(
            action,
            self.total_joint_order)
        self.communicator.send_action(joint_msgs)

    def data_collection_timer_callback(self):
        camera_data = {}
        follower_data = []
        leader_data = []

        if not self.update_latest_data(
            camera_data,
            follower_data,
            leader_data):
            return

        if not self.data_manager.check_lerobot_dataset(
                camera_data,
                self.total_joint_order):
            self.get_logger().info(
                'Invalid Repository Folder, Please check the repository folder')
            self.timer_manager.stop(timer_name=self.operation_mode)
            return

        record_completed = self.data_manager.record(
            images=camera_data,
            state=follower_data,
            action=leader_data)

        if record_completed:
            self.get_logger().info('Recording stopped')
            self.timer_manager.stop(timer_name=self.operation_mode)
            return


    def inference_timer_callback(self):
        camera_data, follower_data, _ = self.update_latest_data()

        if camera_data and follower_data:
            self.latest_camera_data = camera_data
            self.latest_joint_data = follower_data

    def user_interaction_callback(self, request, response):
        save_path = self.default_save_root_path / request.repo_id
        self.get_logger().info(f'Save path: {save_path}')

        if request.command == SendRecordingCommand.Request.START_RECORD:
            self.get_logger().info('Starting recording with task: ' + request.task_name)
            self.operation_mode = 'collection'
            self.init_robot_control_parameters_from_user_task(
                request.robot_type,
                self.operation_mode,
                request.frequency
            )

            self.data_manager = DataManager(
                repo_id=request.repo_id,
                save_path=save_path,
                task_instruction=request.task_instruction,
                tags=['ROBOTIS', request.robot_type],
                use_image_buffer=request.use_image_buffer,
                save_fps=request.frequency,
                reset_time_s=request.reset_time,
                episode_time_s=request.episode_time,
                warmup_time_s=request.warmup_time,
                num_episodes=request.episode_num,
                push_to_hub=request.push_to_hub,
                private=request.private_mode)

            self.timer_manager.start(timer_name=self.operation_mode)
            response.success = True
            response.message = "Recording started"

        elif request.command == SendRecordingCommand.Request.STOP:
            self.get_logger().info('Stopping recording')
            if self.timer_manager is not None:
                self.timer_manager.stop(timer_name=self.operation_mode)
                self.data_manager.record_stop()
            response.success = True
            response.message = "Recording stopped"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PhysicalAIServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
