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

from pathlib import Path
from typing import Optional

from physical_ai_interfaces.msg import HFOperationStatus, TrainingStatus
from physical_ai_interfaces.srv import (
    BrowseFile,
    ControlHfServer,
    EditDataset,
    GetDatasetInfo,
    GetDatasetList,
    GetHFUser,
    GetModelWeightList,
    GetPolicyList,
    GetRobotTypeList,
    GetSavedPolicyList,
    GetUserList,
    SendCommand,
    SendTrainingCommand,
    SetHFUser,
    SetRobotType,
)

from physical_ai_server.callbacks.timer_callbacks import TimerCallbacks
from physical_ai_server.communication.communicator import Communicator
from physical_ai_server.data_processing.data_editor import DataEditor
from physical_ai_server.data_processing.data_manager import DataManager
from physical_ai_server.inference.inference_manager import InferenceManager
from physical_ai_server.service_handlers.dataset_handler import (
    DatasetServiceHandler
)
from physical_ai_server.service_handlers.hf_handler import HFServiceHandler
from physical_ai_server.service_handlers.robot_handler import (
    RobotServiceHandler
)
from physical_ai_server.service_handlers.task_handler import TaskServiceHandler
from physical_ai_server.service_handlers.training_handler import (
    TrainingServiceHandler
)
from physical_ai_server.timer.timer_manager import TimerManager
from physical_ai_server.utils.file_browse_utils import FileBrowseUtils
from physical_ai_server.utils.parameter_utils import (
    declare_parameters,
    load_parameters,
    log_parameters,
)

import rclpy
from rclpy.node import Node


class PhysicalAIServer(Node):
    """
    Main ROS2 node for Physical AI Server.

    Manages robot control, data collection, training, and inference operations
    through modular service handlers and timer callbacks.
    """

    DEFAULT_SAVE_ROOT_PATH = Path.home() / '.cache/huggingface/lerobot'
    PUB_QOS_SIZE = 10

    def __init__(self):
        """Initialize Physical AI Server node."""
        super().__init__('physical_ai_server')
        self.get_logger().info('Start Physical AI Server')

        # Initialize state variables
        self._init_state_variables()

        # Initialize core components
        self._init_core_components()

        # Initialize ROS publishers
        self._init_ros_publisher()

        # Initialize service handlers
        self._init_service_handlers()

        # Initialize timer callbacks
        self._init_timer_callbacks()

        # Initialize ROS services
        self._init_ros_service()

    def _init_state_variables(self):
        """Initialize server state variables."""
        self.params = None
        self.total_joint_order = None
        self.joint_order = None
        self.on_recording = False
        self.on_inference = False
        self.robot_type = None
        self.operation_mode = None
        self.start_recording_time: float = 0.0
        self.task_instruction = None
        self.joint_topic_types = None

    def _init_core_components(self):
        """Initialize core components."""
        self.communicator: Optional[Communicator] = None
        self.data_manager: Optional[DataManager] = None
        self.timer_manager: Optional[TimerManager] = None
        self.heartbeat_timer: Optional[TimerManager] = None
        self.inference_manager: Optional[InferenceManager] = None

        # Initialize data editor
        self.data_editor = DataEditor()

        # Initialize file browse utils
        self.file_browse_utils = FileBrowseUtils(
            max_workers=8,
            logger=self.get_logger()
        )

        # Initialize inference manager
        self.inference_manager = InferenceManager()

    def _init_ros_publisher(self):
        """Initialize ROS publishers."""
        self.get_logger().info('Initializing ROS publishers...')

        self.training_status_publisher = self.create_publisher(
            TrainingStatus,
            '/training/status',
            100
        )

        self.hf_status_publisher = self.create_publisher(
            HFOperationStatus,
            '/huggingface/status',
            self.PUB_QOS_SIZE
        )

    def _init_service_handlers(self):
        """Initialize service handlers."""
        self.get_logger().info('Initializing service handlers...')

        # Initialize HF handler with worker
        from physical_ai_server.data_processing.hf_api_worker import (
            HfApiWorker
        )
        hf_api_worker = HfApiWorker()
        self.hf_handler = HFServiceHandler(
            self,
            hf_api_worker,
            self.hf_status_publisher
        )
        self.hf_handler._init_hf_api_worker()

        # Initialize robot handler
        self.robot_handler = RobotServiceHandler(
            self,
            self.inference_manager
        )

        # Initialize training handler
        self.training_handler = TrainingServiceHandler(
            self,
            self.training_status_publisher
        )

        # Initialize dataset handler
        self.dataset_handler = DatasetServiceHandler(
            self,
            self.data_editor,
            self.file_browse_utils
        )

        # Initialize task handler
        self.task_handler = TaskServiceHandler(self)

        # Set callbacks for task handler
        self.task_handler.init_robot_control_callback = (
            self.init_robot_control_parameters_from_user_task
        )
        self.task_handler.clear_parameters_callback = (
            self.clear_parameters
        )
        self.task_handler.init_ros_params_callback = (
            self.init_ros_params
        )

    def _init_timer_callbacks(self):
        """Initialize timer callbacks."""
        self.timer_callbacks = TimerCallbacks(
            self,
            self.params,
            self.total_joint_order,
            self.joint_order
        )

    def _init_ros_service(self):
        """Initialize ROS services."""
        self.get_logger().info('Initializing ROS services...')

        service_definitions = [
            # Task control services
            ('/task/command', SendCommand,
             self.task_handler.user_interaction_callback),

            # Robot configuration services
            ('/get_robot_types', GetRobotTypeList,
             self.robot_handler.get_robot_types_callback),
            ('/set_robot_type', SetRobotType,
             self.task_handler.set_robot_type_callback),
            ('/get_policy_list', GetPolicyList,
             self.robot_handler.get_policy_list_callback),
            ('/get_saved_policies', GetSavedPolicyList,
             self.robot_handler.get_saved_policies_callback),

            # HuggingFace services
            ('/register_hf_user', SetHFUser,
             self.hf_handler.set_hf_user_callback),
            ('/get_registered_hf_user', GetHFUser,
             self.hf_handler.get_hf_user_callback),
            ('/huggingface/control', ControlHfServer,
             self.hf_handler.control_hf_server_callback),

            # Training services
            ('/training/command', SendTrainingCommand,
             self.training_handler.user_training_interaction_callback),
            ('/training/get_available_policy', GetPolicyList,
             self.training_handler.get_available_list_callback),
            ('/training/get_user_list', GetUserList,
             self.training_handler.get_user_list_callback),
            ('/training/get_dataset_list', GetDatasetList,
             self.training_handler.get_dataset_list_callback),
            ('/training/get_model_weight_list', GetModelWeightList,
             self.training_handler.get_model_weight_list_callback),

            # Dataset services
            ('/browse_file', BrowseFile,
             self.dataset_handler.browse_file_callback),
            ('/dataset/edit', EditDataset,
             self.dataset_handler.dataset_edit_callback),
            ('/dataset/get_info', GetDatasetInfo,
             self.dataset_handler.get_dataset_info_callback),
        ]

        for service_name, service_type, callback in service_definitions:
            self.create_service(service_type, service_name, callback)

        self.get_logger().info('ROS services initialized successfully')

    def init_ros_params(self, robot_type):
        """
        Initialize ROS parameters for specified robot type.

        Args:
            robot_type: Robot type identifier
        """
        self.get_logger().info(
            f'Initializing ROS parameters for robot type: {robot_type}'
        )

        param_names = [
            'camera_topic_list',
            'joint_topic_list',
            'observation_list',
            'joint_list',
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
            f'joint_order.{joint_name}'
            for joint_name in self.params['joint_list']
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

        # Initialize communicator
        self.communicator = Communicator(
            node=self,
            operation_mode=self.operation_mode,
            params=self.params
        )

        # Initialize heartbeat timer
        if self.heartbeat_timer is None:
            self.heartbeat_timer = TimerManager(node=self)
            self.heartbeat_timer.set_timer(
                timer_name='heartbeat',
                timer_frequency=1.0,
                callback_function=(
                    self.communicator.heartbeat_timer_callback
                )
            )
            self.heartbeat_timer.start(timer_name='heartbeat')

        self.get_logger().info(
            f'ROS parameters initialized successfully for '
            f'robot type: {robot_type}'
        )

    def init_robot_control_parameters_from_user_task(self, task_info):
        """
        Initialize robot control parameters from user task.

        Args:
            task_info: Task information message
        """
        self.get_logger().info(
            'Initializing robot control parameters from user task...'
        )

        self.data_manager = DataManager(
            save_root_path=self.DEFAULT_SAVE_ROOT_PATH,
            robot_type=self.robot_type,
            task_info=task_info
        )

        self.communicator.clear_latest_data()

        # Setup timer callback
        timer_callback_dict = {
            'collection': (
                self.timer_callbacks.data_collection_timer_callback
            ),
            'inference': (
                self.timer_callbacks.inference_timer_callback
            )
        }

        self.timer_manager = TimerManager(node=self)
        self.timer_manager.set_timer(
            timer_name=self.operation_mode,
            timer_frequency=task_info.fps,
            callback_function=timer_callback_dict[self.operation_mode]
        )
        self.timer_manager.start(timer_name=self.operation_mode)

        # Set managers for handlers and callbacks
        self.task_handler.set_managers(
            self.data_manager,
            self.inference_manager,
            self.communicator,
            self.timer_manager
        )
        self.timer_callbacks.set_managers(
            self.data_manager,
            self.communicator,
            self.inference_manager,
            self.timer_manager
        )

        self.get_logger().info(
            'Robot control parameters initialized successfully'
        )

    def clear_parameters(self):
        """Clear robot parameters and cleanup resources."""
        if self.communicator is not None:
            self.communicator.cleanup()
            self.communicator = None

        if self.timer_manager is not None:
            self.timer_manager = None

        if self.heartbeat_timer is not None:
            self.heartbeat_timer.stop(timer_name='heartbeat')
            self.heartbeat_timer = None

        self.params = None
        self.total_joint_order = None
        self.joint_order = None

    def shutdown(self):
        """Cleanup resources on shutdown."""
        self.get_logger().info('Shutting down Physical AI Server...')

        # Cleanup handlers
        if hasattr(self, 'hf_handler'):
            self.hf_handler.cleanup()

        if hasattr(self, 'training_handler'):
            self.training_handler.cleanup()

        # Cleanup timers
        if self.heartbeat_timer is not None:
            self.heartbeat_timer.stop(timer_name='heartbeat')

        if self.timer_manager is not None:
            if self.operation_mode:
                self.timer_manager.stop(timer_name=self.operation_mode)

        # Cleanup communicator
        if self.communicator is not None:
            self.communicator.cleanup()

        self.get_logger().info('Physical AI Server shutdown complete')


def main(args=None):
    """Main entry point for Physical AI Server."""
    rclpy.init(args=args)
    node = PhysicalAIServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
