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

from datetime import datetime
import glob
import json
import os
from pathlib import Path
import threading
import time
import traceback
from typing import Optional

import yaml

from ament_index_python.packages import get_package_share_directory
from physical_ai_interfaces.msg import (
    BrowserItem,
    HFOperationStatus,
    TaskInfo,
    TaskStatus,
    TrainingStatus,
)
from physical_ai_interfaces.srv import (
    BrowseFile,
    ControlHfServer,
    GetDatasetList,
    GetHFUser,
    GetModelWeightList,
    GetPolicyList,
    GetReplayData,
    GetRobotTypeList,
    GetSavedPolicyList,
    GetTrainingInfo,
    GetUserList,
    SendCommand,
    SendTrainingCommand,
    SetHFUser,
    SetRobotType,
)

from physical_ai_server.communication.communicator import Communicator
from physical_ai_server.data_processing.data_manager import DataManager
from physical_ai_server.data_processing.hf_api_worker import HfApiWorker
from physical_ai_server.data_processing.mp4_conversion_worker import Mp4ConversionWorker
from physical_ai_server.data_processing.replay_data_handler import ReplayDataHandler
from physical_ai_server.inference.zenoh_inference_manager import ZenohInferenceManager
from physical_ai_server.timer.timer_manager import TimerManager
from physical_ai_server.training.zenoh_training_manager import ZenohTrainingManager
from physical_ai_server.utils.file_browse_utils import FileBrowseUtils
from physical_ai_server.utils.parameter_utils import (
    declare_parameters,
    load_parameters,
    log_parameters,
)
from physical_ai_server.utils.video_file_server import VideoFileServer

import rclpy
from rclpy.node import Node


class PhysicalAIServer(Node):
    # Define operation modes (constants taken from Communicator)

    DEFAULT_SAVE_ROOT_PATH = Path.home() / '.cache/huggingface/lerobot'
    DEFAULT_TOPIC_TIMEOUT = 5.0  # seconds
    PUB_QOS_SIZE = 10
    TRAINING_STATUS_TIMER_FREQUENCY = 0.5  # seconds
    VIDEO_SERVER_PORT = 8082  # Port for video file server

    class RosbagNotReadyException(Exception):
        """Exception raised when rosbag recording cannot start yet."""

        pass

    def __init__(self):
        super().__init__('physical_ai_server')
        self.get_logger().info('Start Physical AI Server')

        self.params = None
        self.total_joint_order = None
        self.on_recording = False
        self.on_inference = False

        self.hf_cancel_on_progress = False

        self.robot_type_list = self.get_robot_type_list()
        self.start_recording_time: float = 0.0

        self.training_thread = None
        self.is_training = False
        self.training_status_timer = None

        self._init_core_components()

        self._init_ros_publisher()
        self._init_ros_service()

        self._setup_timer_callbacks()

        self.previous_data_manager_status = None

        self.goal_repo_id = None

    def _init_core_components(self):
        self.communicator: Optional[Communicator] = None
        self.data_manager: Optional[DataManager] = None
        self.timer_manager: Optional[TimerManager] = None
        self.heartbeat_timer: Optional[TimerManager] = None
        self.training_timer: Optional[TimerManager] = None
        # Zenoh managers for inference/training (Docker container communication)
        self.inference_manager: Optional[ZenohInferenceManager] = None
        self.training_manager: Optional[ZenohTrainingManager] = None

        # Initialize HF API Worker
        self.hf_api_worker: Optional[HfApiWorker] = None
        self.hf_status_timer: Optional[TimerManager] = None
        self._init_hf_api_worker()

        # Initialize MP4 Conversion Worker
        self.mp4_conversion_worker: Optional[Mp4ConversionWorker] = None
        self.mp4_status_timer: Optional[TimerManager] = None

        # Initialize ReplayDataHandler for replay viewer
        self.replay_data_handler = ReplayDataHandler(logger=self.get_logger())

        # Initialize FileBrowseUtils for file browsing
        self.file_browse_utils = FileBrowseUtils(
            max_workers=8,
            logger=self.get_logger()
        )

        # Initialize Video File Server for replay viewer
        self._init_video_server()

    def _init_video_server(self):
        """Initialize the video file server for replay viewer."""
        try:
            # Allow serving files from home directory and workspace
            allowed_paths = [
                str(Path.home()),
                '/workspace',  # Docker workspace directory
            ]
            self.video_server = VideoFileServer(
                port=self.VIDEO_SERVER_PORT,
                allowed_paths=allowed_paths,
                replay_data_handler=self.replay_data_handler
            )
            self.video_server.start()
            self.get_logger().info(
                f'Video file server started on port {self.VIDEO_SERVER_PORT}'
            )
            self.get_logger().info(
                f'Replay data API available at http://0.0.0.0:{self.VIDEO_SERVER_PORT}/replay-data/'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to start video server: {e}')
            self.video_server = None

    def _init_ros_publisher(self):
        self.get_logger().info('Initializing ROS publishers...')
        pub_qos_size = 100
        self.training_status_publisher = self.create_publisher(
            TrainingStatus,
            '/training/status',
            pub_qos_size
        )

    def _init_ros_service(self):
        self.get_logger().info('Initializing ROS services...')
        service_definitions = [
            ('/task/command', SendCommand, self.user_interaction_callback),
            ('/get_robot_types', GetRobotTypeList, self.get_robot_types_callback),
            ('/set_robot_type', SetRobotType, self.set_robot_type_callback),
            ('/register_hf_user', SetHFUser, self.set_hf_user_callback),
            ('/get_registered_hf_user', GetHFUser, self.get_hf_user_callback),
            ('/get_policy_list', GetPolicyList, self.get_policy_list_callback),
            ('/get_saved_policies', GetSavedPolicyList, self.get_saved_policies_callback),
            ('/training/command', SendTrainingCommand, self.user_training_interaction_callback),
            ('/training/get_available_policy', GetPolicyList, self.get_available_list_callback),
            ('/training/get_user_list', GetUserList, self.get_user_list_callback),
            ('/training/get_dataset_list', GetDatasetList, self.get_dataset_list_callback),
            (
                '/training/get_model_weight_list',
                GetModelWeightList,
                self.get_model_weight_list_callback
            ),
            ('/huggingface/control', ControlHfServer, self.control_hf_server_callback),
            ('/training/get_training_info', GetTrainingInfo, self.get_training_info_callback),
            ('/replay/get_data', GetReplayData, self.get_replay_data_callback),
            ('/browse_file', BrowseFile, self.browse_file_callback),
        ]

        for service_name, service_type, callback in service_definitions:
            self.create_service(service_type, service_name, callback)

        self.get_logger().info('ROS services initialized successfully')

    def _setup_timer_callbacks(self):
        self.timer_callback_dict = {
            'collection': self._data_collection_timer_callback,
            'inference': self._inference_timer_callback
        }

    def init_ros_params(self, robot_type):
        self.get_logger().info(f'Initializing ROS parameters for robot type: {robot_type}')
        # List parameters (STRING_ARRAY type)
        list_param_names = [
            'camera_topic_list',
            'joint_topic_list',
            'observation_list',
            'joint_list',
            'rosbag_extra_topic_list',
        ]

        # Declare list parameters
        declare_parameters(
            node=self,
            robot_type=robot_type,
            param_names=list_param_names,
            default_value=['']
        )

        # Declare string parameters separately (urdf_path is a single string)
        urdf_param_name = f'{robot_type}.urdf_path'
        if not self.has_parameter(urdf_param_name):
            self.declare_parameter(urdf_param_name, '')

        # Load list parameters
        self.params = load_parameters(
            node=self,
            robot_type=robot_type,
            param_names=list_param_names
        )

        # Load urdf_path separately
        self.params['urdf_path'] = self.get_parameter(urdf_param_name).value

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

        if self.heartbeat_timer is None:
            self.heartbeat_timer = TimerManager(node=self)
            self.heartbeat_timer.set_timer(
                timer_name='heartbeat',
                timer_frequency=1.0,
                callback_function=self.communicator.heartbeat_timer_callback
            )
            self.heartbeat_timer.start(timer_name='heartbeat')

        # Register joystick handler for immediate processing
        self.communicator.register_joystick_handler(self.handle_joystick_trigger)

        self.inference_manager = ZenohInferenceManager(node=self)
        self.get_logger().info(
            f'ROS parameters initialized successfully for robot type: {robot_type}')

    def get_training_status(self):
        msg = TrainingStatus()
        if self.training_manager is None:
            return
        try:
            current_status = self.training_manager.get_current_training_status()
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
            self.get_logger().error(f'Error publishing training status: {msg.error}')
            return msg
        return msg

    def init_robot_control_parameters_from_user_task(
            self,
            task_info):
        self.get_logger().info(
            'Initializing robot control parameters from user task...')
        self.data_manager = DataManager(
            save_root_path=self.DEFAULT_SAVE_ROOT_PATH,
            robot_type=self.robot_type,
            task_info=task_info
        )

        self.timer_manager = TimerManager(node=self)
        # Use 10 Hz for status publishing (rosbag2-only mode, no data collection needed)
        # TODO: Consider moving to event-based approach instead of timer
        self.timer_manager.set_timer(
            timer_name=self.operation_mode,
            timer_frequency=10,
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

        if self.heartbeat_timer is not None:
            self.heartbeat_timer.stop(timer_name='heartbeat')
            self.heartbeat_timer = None

        if self.training_timer is not None:
            self.training_timer.stop(timer_name='training_status')
            self.training_timer = None

        self.params = None
        self.total_joint_order = None
        self.joint_order = None

    def set_hf_user_callback(self, request, response):
        request_hf_token = request.token
        try:
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
        except Exception as e:
            self.get_logger().error(f'Error in set_hf_user_callback: {str(e)}')
            response.user_id_list = []
            response.success = False
            response.message = f'Error in set_hf_user_callback:\n{str(e)}'

        return response

    def get_hf_user_callback(self, request, response):
        try:
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
        except Exception as e:
            self.get_logger().error(f'Error in get_hf_user_callback: {str(e)}')
            response.user_id_list = []
            response.success = False
            response.message = f'Failed to retrieve Hugging Face user ID:\n{str(e)}'

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

    def handle_rosbag_recording(self):
        """
        Handle rosbag recording state transitions (simplified mode).

        States: idle -> recording -> idle
        Transitions:
        - idle -> recording: START rosbag
        - recording -> idle (finish): STOP rosbag (save)
        - recording -> idle (cancel): STOP_AND_DELETE rosbag
        """
        try:
            current = self.data_manager.get_status()
            previous = self.previous_data_manager_status

            # Early return if no status change
            if current == previous:
                return

            self.get_logger().info(f'Rosbag state transition: {previous} -> {current}')

            # Simplified state transitions
            if current == 'recording' and previous in ('idle', None):
                # Start recording
                rosbag_path = self.data_manager.get_save_rosbag_path()
                self.get_logger().info(
                    f'Rosbag state: idle->recording, path={rosbag_path}, '
                    f'episode={self.data_manager._record_episode_count}')
                if rosbag_path:
                    self.communicator.start_rosbag(rosbag_uri=rosbag_path)
                    self.get_logger().info(f'Rosbag recording started: {rosbag_path}')
                else:
                    self.get_logger().error('Rosbag path is None - cannot start recording!')

            elif current == 'idle' and previous == 'recording':
                # This is handled by stop_recording_and_save or cancel_recording
                # The actual rosbag command is sent from those methods
                pass

            # Legacy state transitions (for backward compatibility)
            elif current == 'warmup':
                rosbag_topics = self.communicator.get_all_topics()
                self.communicator.prepare_rosbag(topics=rosbag_topics)

            elif current == 'run' and previous in ('warmup', 'reset', 'skip_task'):
                rosbag_path = self.data_manager.get_save_rosbag_path()
                if rosbag_path:
                    self.communicator.start_rosbag(rosbag_uri=rosbag_path)

            elif current == 'save' and previous == 'run':
                urdf_path = self.params.get('urdf_path', '')
                if urdf_path:
                    self.data_manager.save_robotis_metadata(urdf_path=urdf_path)
                self.communicator.stop_rosbag()

            elif current == 'stop' and previous == 'run':
                self.communicator.stop_rosbag()

            elif current == 'finish':
                self.communicator.finish_rosbag()

            elif current == 'reset' and previous == 'run':
                self.communicator.stop_and_delete_rosbag()

            self.previous_data_manager_status = current

        except Exception as e:
            error_msg = f'Error in rosbag recording: {str(e)}'
            self.get_logger().error(traceback.format_exc())
            self.get_logger().error(error_msg)

    def stop_recording_and_save(self):
        """Stop recording and save the rosbag (simplified mode)."""
        if self.data_manager is None:
            return

        self.get_logger().info(
            f'Stopping recording: episode={self.data_manager._record_episode_count}, '
            f'status={self.data_manager.get_status()}')

        # Save metadata before stopping
        urdf_path = self.params.get('urdf_path', '')
        if urdf_path:
            self.data_manager.save_robotis_metadata(urdf_path=urdf_path)

        # Stop rosbag
        self.communicator.stop_rosbag()

        # Update data manager state
        self.data_manager.stop_recording()

        # IMPORTANT: Update previous status to 'idle' so next recording can detect transition
        # This is needed because timer might be stopped before callback can update it
        self.previous_data_manager_status = 'idle'

        self.get_logger().info(
            f'Recording stopped and saved: next_episode={self.data_manager._record_episode_count}')

    def cancel_current_recording(self):
        """Cancel recording and save with review flag (simplified mode).

        Data is saved instead of deleted, with needs_review=True in metadata.
        This allows partially useful data to be reviewed later.
        """
        if self.data_manager is None:
            return

        # Save metadata with review flag BEFORE stopping (status must be 'recording')
        urdf_path = self.params.get('urdf_path', '')
        if urdf_path:
            self.data_manager.save_robotis_metadata(
                urdf_path=urdf_path, needs_review=True)

        # Stop rosbag (save, not delete)
        self.communicator.stop_rosbag()

        # Update data manager state (increment episode since data is saved)
        self.data_manager.stop_recording()

        # IMPORTANT: Update previous status to 'idle' so next recording can detect transition
        self.previous_data_manager_status = 'idle'

        self.get_logger().info('Recording cancelled - data saved with review flag')

    def _data_collection_timer_callback(self):
        """
        Timer callback for rosbag2-only data collection (simplified mode).

        This callback manages:
        - Joystick trigger handling (right=toggle Start/Finish, left=Cancel)
        - Status publishing

        Note: No warmup, no episode timing, no reset cycle.
        Recording starts/stops instantly on user command.
        """
        # Handle joystick trigger for recording control
        if self.communicator.joystick_state['updated']:
            self.handle_joystick_trigger(
                joystick_mode=self.communicator.joystick_state['mode'])
            self.communicator.joystick_state['updated'] = False

        # Get current status and publish
        current_status = self.data_manager.get_current_record_status()
        self.communicator.publish_status(status=current_status)

        # Handle rosbag2 recording state transitions
        if self.data_manager.should_record_rosbag2():
            self.handle_rosbag_recording()

    def _inference_timer_callback(self):
        """
        Timer callback for inference mode.

        NOTE: Inference mode is currently disabled in rosbag2-only data acquisition mode.
        Data subscribers were removed to simplify the architecture for rosbag2 recording.
        To re-enable inference, data subscribers need to be added back to communicator.py.
        """
        error_msg = 'Inference mode is not supported in rosbag2-only mode. ' \
                    'Data subscribers are required for inference.'
        self.get_logger().error(error_msg)

        current_status = TaskStatus()
        current_status.phase = TaskStatus.READY
        current_status.error = error_msg

        self.on_inference = False
        self.communicator.publish_status(status=current_status)
        if self.inference_manager:
            self.inference_manager.clear_policy()
        self.timer_manager.stop(timer_name=self.operation_mode)

    def user_training_interaction_callback(self, request, response):
        """
        Handle training command requests (START/FINISH).

        Supports both new training and resume functionality with proper validation.
        """
        try:
            if request.command == SendTrainingCommand.Request.START:
                # Initialize training components (ROS2 services with rmw_zenoh)
                self.training_manager = ZenohTrainingManager(node=self)
                self.training_timer = TimerManager(node=self)
                self._setup_training_status_timer()

                # Validate training state
                if self.training_thread and self.training_thread.is_alive():
                    response.success = False
                    response.message = 'Training is already in progress'
                    return response

                # Extract resume parameters
                resume = getattr(request, 'resume', False)
                resume_model_path = getattr(request, 'resume_model_path', '').strip()

                # Log training request details
                output_folder_name = request.training_info.output_folder_name
                weight_save_root_path = ZenohTrainingManager.get_weight_save_root_path()
                self.get_logger().info(
                    f'Training request - Output: {output_folder_name}, '
                    f'Resume: {resume}, Model path: {resume_model_path}'
                )

                # Validate training configuration
                validation_result = self._validate_training_request(
                    resume, resume_model_path, output_folder_name, weight_save_root_path
                )
                if not validation_result['success']:
                    response.success = False
                    response.message = validation_result['message']
                    self._cleanup_training_on_error()
                    return response

                # Configure and start training
                self._configure_training_manager(request, resume, resume_model_path)
                self._start_training_thread()

                response.success = True
                response.message = 'Training started successfully'

            else:
                # Handle FINISH command
                if request.command == SendTrainingCommand.Request.FINISH:
                    self._stop_training()
                    response.success = True
                    response.message = 'Training stopped successfully'
                else:
                    response.success = False
                    response.message = f'Unknown command: {request.command}'

        except Exception as e:
            self.get_logger().error(f'Error in training callback: {str(e)}')
            response.success = False
            response.message = f'Training error: {str(e)}'
            self._cleanup_training_on_error()

        return response

    def _setup_training_status_timer(self):
        """Set up timer for publishing training status updates."""
        self.training_timer.set_timer(
            timer_name='training_status',
            timer_frequency=self.TRAINING_STATUS_TIMER_FREQUENCY,
            callback_function=lambda: self.training_status_publisher.publish(
                self.get_training_status()
            )
        )
        self.training_timer.start(timer_name='training_status')

    def _validate_training_request(
            self,
            resume,
            resume_model_path,
            output_folder_name,
            weight_save_root_path
    ):
        """
        Validate training request parameters.

        Returns
        -------
        dict
            {'success': bool, 'message': str}

        """
        # Check output folder conflicts for new training
        if not resume:
            output_path = weight_save_root_path / output_folder_name
            if output_path.exists():
                return {
                    'success': False,
                    'message': f'Output folder already exists: {output_path}'
                }

        # Validate resume configuration
        if resume:
            if not resume_model_path:
                return {
                    'success': False,
                    'message': 'Resume model path is required when resume=True'
                }

            # Check if resume config file exists
            full_config_path = weight_save_root_path / resume_model_path
            if not full_config_path.exists():
                return {
                    'success': False,
                    'message': f'Resume config file not found: {full_config_path}'
                }

        return {'success': True, 'message': 'Validation passed'}

    def _configure_training_manager(self, request, resume, resume_model_path):
        """Configure training manager with request parameters."""
        self.training_manager.training_info = request.training_info
        self.training_manager.resume = resume
        self.training_manager.resume_model_path = resume_model_path

    def _start_training_thread(self):
        """Start training in a separate thread."""
        def run_training():
            try:
                self.training_manager.train()
            except Exception as e:
                self.get_logger().error(f'Training error: {str(e)}')
            finally:
                self._cleanup_training_on_completion()

        self.training_thread = threading.Thread(target=run_training, daemon=True)
        self.training_thread.start()
        self.is_training = True

    def _stop_training(self):
        """Stop training gracefully."""
        self.is_training = False
        if self.training_manager:
            self.training_manager.stop_event.set()
        if self.training_thread and self.training_thread.is_alive():
            self.training_thread.join(timeout=self.DEFAULT_TOPIC_TIMEOUT)
        self._cleanup_training_on_completion()

    def _cleanup_training_on_completion(self):
        """Cleanup training resources on normal completion."""
        self.is_training = False
        self.get_logger().info('Training completed.')
        training_status = self.get_training_status()
        self.training_status_publisher.publish(training_status)
        if self.training_manager:
            self.training_manager.stop_event.set()
        if hasattr(self, 'training_timer'):
            self.training_timer.stop('training_status')

    def _cleanup_training_on_error(self):
        """Cleanup training resources on error."""
        self.is_training = False
        training_status = self.get_training_status()
        self.training_status_publisher.publish(training_status)
        if self.training_manager:
            self.training_manager.stop_event.set()
        if hasattr(self, 'training_timer'):
            self.training_timer.stop('training_status')

    def user_interaction_callback(self, request, response):
        """
        Handle user commands for recording control (simplified mode).

        Commands:
        - START_RECORD: Initialize and start recording
        - STOP / FINISH: Stop and save recording
        - RERECORD: Cancel current recording (discard)
        """
        try:
            if request.command == SendCommand.Request.START_RECORD:
                # Initialize data manager only if it doesn't exist or task changed
                task_info = request.task_info
                task_name = f'{self.robot_type}_{task_info.task_name}'

                # Check if we need to create a new DataManager
                need_new_manager = (
                    self.data_manager is None or
                    self.data_manager._save_repo_name != task_name
                )

                if need_new_manager:
                    self.get_logger().info('Initializing new recording session')
                    self.operation_mode = 'collection'
                    self.init_robot_control_parameters_from_user_task(task_info)
                else:
                    episode = self.data_manager._record_episode_count
                    self.get_logger().info(
                        f'Continuing recording session - Episode {episode}')
                    # Restart timer if it was stopped
                    if self.timer_manager:
                        self.timer_manager.start(timer_name=self.operation_mode)

                self.on_recording = True

                # Start recording (simplified mode)
                self.get_logger().info('Starting recording')
                self.data_manager.start_recording()
                self.start_recording_time = time.perf_counter()
                self.communicator.publish_action_event('start')
                response.success = True
                response.message = 'Recording started'

            elif request.command == SendCommand.Request.START_INFERENCE:
                self.joint_topic_types = self.communicator.get_publisher_msg_types()
                self.operation_mode = 'inference'
                task_info = request.task_info
                self.task_instruction = task_info.task_instruction

                valid_result, result_message = self.inference_manager.validate_policy(
                    policy_path=task_info.policy_path)

                if not valid_result:
                    response.success = False
                    response.message = result_message
                    self.get_logger().error(response.message)
                    return response

                self.init_robot_control_parameters_from_user_task(task_info)
                if task_info.record_inference_mode:
                    self.on_recording = True
                self.on_inference = True
                self.start_recording_time = time.perf_counter()
                response.success = True
                response.message = 'Inference started'

            elif request.command == SendCommand.Request.CONVERT_MP4:
                # Handle MP4 conversion command
                task_info = request.task_info
                task_name = f'{self.robot_type}_{task_info.task_name}'
                source_folders = [s for s in task_info.task_instruction if s.strip()]

                base_path = Path('/workspace/rosbag2')

                # Initialize MP4 conversion worker if needed
                if self.mp4_conversion_worker is None or not self.mp4_conversion_worker.is_alive():
                    self._init_mp4_conversion_worker()

                if self.mp4_conversion_worker is None:
                    response.success = False
                    response.message = 'Failed to initialize MP4 conversion worker'
                    return response

                if self.mp4_conversion_worker.is_busy():
                    response.success = False
                    response.message = 'MP4 conversion is already in progress'
                    return response

                if source_folders:
                    # === Merge & Convert mode ===
                    dataset_path = base_path / task_name

                    # Validate source folders
                    source_paths = []
                    for folder_name in source_folders:
                        src = Path(folder_name) if folder_name.startswith('/') \
                            else base_path / folder_name
                        if not src.exists():
                            response.success = False
                            response.message = f'Source folder not found: {src}'
                            return response
                        source_paths.append(str(src))

                    if dataset_path.exists():
                        response.success = False
                        response.message = \
                            f'Output folder already exists: {dataset_path}'
                        return response

                    request_data = {
                        'dataset_path': str(dataset_path),
                        'robot_type': self.robot_type,
                        'robot_config_path': os.path.join(
                            get_package_share_directory('physical_ai_server'),
                            'config', f'{self.robot_type}_config.yaml'
                        ),
                        'source_folders': source_paths,
                    }
                else:
                    # === Single Convert mode (existing behavior) ===
                    dataset_path = base_path / task_name

                    if not dataset_path.exists():
                        response.success = False
                        response.message = \
                            f'Dataset path does not exist: {dataset_path}'
                        return response

                    request_data = {
                        'dataset_path': str(dataset_path),
                        'robot_type': self.robot_type,
                        'robot_config_path': os.path.join(
                            get_package_share_directory('physical_ai_server'),
                            'config', f'{self.robot_type}_config.yaml'
                        ),
                    }

                # Send conversion request
                if self.mp4_conversion_worker.send_request(request_data):
                    self.get_logger().info(f'MP4 conversion started for: {dataset_path}')
                    response.success = True
                    response.message = f'MP4 conversion started for: {task_name}'
                else:
                    response.success = False
                    response.message = 'Failed to start MP4 conversion'

            elif request.command == 10:  # UPLOAD_ROSBAG
                # Handle rosbag upload to HuggingFace Hub
                try:
                    # Extract parameters from request
                    import json
                    data = json.loads(request.data) if isinstance(request.data, str) else request.data

                    folder_paths = data.get('folder_paths', [])
                    hf_token = data.get('hf_token', '')
                    hf_endpoint = data.get('hf_endpoint', 'http://192.168.60.152:1000')

                    if not folder_paths:
                        response.success = False
                        response.message = 'No folders specified for upload'
                        return response

                    # If no token provided, use saved token (pass None to use saved token)
                    if not hf_token:
                        hf_token = None
                        self.get_logger().info('No token provided, will use saved token from server')

                    self.get_logger().info(
                        f'Starting upload of {len(folder_paths)} rosbag folder(s) to {hf_endpoint}'
                    )

                    # Define progress callback
                    def progress_callback(folder_name, percentage, status, error=None):
                        """Callback for upload progress updates."""
                        self.get_logger().info(
                            f'Upload progress: {folder_name} - {percentage}% ({status})'
                        )
                        # TODO: Publish to ROS topic for UI updates

                    # Start upload in background thread
                    def upload_worker():
                        try:
                            result = DataManager.upload_rosbag_folders(
                                folder_paths=folder_paths,
                                hf_token=hf_token,
                                hf_endpoint=hf_endpoint,
                                progress_callback=progress_callback
                            )

                            if result['success']:
                                self.get_logger().info(
                                    f"Upload completed successfully: {len(result['results'])} folders"
                                )
                            else:
                                self.get_logger().error(
                                    f"Upload failed: {result.get('error', 'Unknown error')}"
                                )
                        except Exception as e:
                            self.get_logger().error(f'Upload worker error: {e}')
                            import traceback
                            self.get_logger().error(traceback.format_exc())

                    upload_thread = threading.Thread(target=upload_worker, daemon=True)
                    upload_thread.start()

                    response.success = True
                    response.message = f'Started uploading {len(folder_paths)} folder(s)'

                except Exception as e:
                    self.get_logger().error(f'Error starting upload: {e}')
                    response.success = False
                    response.message = f'Error starting upload: {str(e)}'

            else:
                if not self.on_recording and not self.on_inference:
                    # Not recording - handle cancel to toggle previous episode review
                    if request.command in (
                        SendCommand.Request.CANCEL,
                        SendCommand.Request.RERECORD,
                    ) and self.data_manager is not None:
                        result = self.data_manager \
                            .toggle_previous_episode_needs_review()
                        if result is not None:
                            event = 'review_on' if result else 'review_off'
                            self.communicator.publish_action_event(event)
                            response.success = True
                            response.message = \
                                f'Previous episode needs_review: {result}'
                        else:
                            response.success = False
                            response.message = 'No previous episode to toggle'
                    else:
                        response.success = False
                        response.message = 'Not currently recording'
                else:
                    if request.command == SendCommand.Request.STOP:
                        # Stop and save (simplified mode)
                        self.get_logger().info('Stopping and saving recording')
                        self.stop_recording_and_save()
                        self.communicator.publish_action_event('finish')
                        response.success = True
                        response.message = 'Recording stopped and saved'

                    elif request.command == SendCommand.Request.MOVE_TO_NEXT:
                        # In simplified mode, this saves current and prepares for next
                        self.get_logger().info('Saving current episode')
                        self.stop_recording_and_save()
                        self.communicator.publish_action_event('finish')
                        response.success = True
                        response.message = 'Episode saved'

                    elif request.command == SendCommand.Request.RERECORD:
                        # Cancel current recording (save with review flag)
                        self.get_logger().info('Cancelling current recording')
                        self.cancel_current_recording()
                        self.communicator.publish_action_event('cancel')
                        self.on_recording = False
                        self.on_inference = False
                        if self.timer_manager:
                            self.timer_manager.stop(timer_name=self.operation_mode)
                        # Publish final READY status
                        if self.data_manager:
                            final_status = self.data_manager.get_current_record_status()
                            self.communicator.publish_status(status=final_status)
                        response.success = True
                        response.message = 'Recording cancelled'

                    elif request.command == SendCommand.Request.FINISH:
                        # Finish all operations
                        self.get_logger().info('Finishing all operations')
                        if self.data_manager and self.data_manager.is_recording():
                            self.stop_recording_and_save()
                        self.communicator.publish_action_event('finish')
                        self.communicator.finish_rosbag()
                        self.on_recording = False
                        self.on_inference = False
                        if self.timer_manager:
                            self.timer_manager.stop(timer_name=self.operation_mode)
                        # Publish final READY status after stopping timer
                        if self.data_manager:
                            final_status = self.data_manager.get_current_record_status()
                            self.communicator.publish_status(status=final_status)
                        response.success = True
                        response.message = 'All operations terminated'

                    elif request.command == SendCommand.Request.SKIP_TASK:
                        # In simplified mode, skip = cancel (save with review flag)
                        self.get_logger().info('Skipping (cancelling) current recording')
                        self.cancel_current_recording()
                        self.communicator.publish_action_event('cancel')
                        self.on_recording = False
                        self.on_inference = False
                        if self.timer_manager:
                            self.timer_manager.stop(timer_name=self.operation_mode)
                        # Publish final READY status
                        if self.data_manager:
                            final_status = self.data_manager.get_current_record_status()
                            self.communicator.publish_status(status=final_status)
                        response.success = True
                        response.message = 'Recording cancelled'

                    elif request.command == SendCommand.Request.CANCEL:
                        # Cancel current recording (save with review flag)
                        self.get_logger().info('Cancelling current recording')
                        self.cancel_current_recording()
                        self.communicator.publish_action_event('cancel')
                        self.on_recording = False
                        self.on_inference = False
                        if self.timer_manager:
                            self.timer_manager.stop(timer_name=self.operation_mode)
                        # Publish final READY status
                        if self.data_manager:
                            final_status = self.data_manager.get_current_record_status()
                            self.communicator.publish_status(status=final_status)
                        response.success = True
                        response.message = 'Recording cancelled'

                    else:
                        response.success = False
                        response.message = f'Unknown command: {request.command}'

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

    def get_policy_list_callback(self, request, response):
        policy_list = ZenohInferenceManager.get_available_policies()
        if not policy_list:
            self.get_logger().warning('No policies available')
            response.success = False
            response.message = 'No policies available'
        else:
            self.get_logger().info(f'Available policies: {policy_list}')
            response.success = True
            response.message = 'Policy list retrieved successfully'
        response.policy_list = policy_list
        return response

    def get_available_list_callback(self, request, response):
        response.success = True
        response.message = 'Policy and device lists retrieved successfully'
        response.policy_list, response.device_list = ZenohTrainingManager.get_available_list()
        return response

    def get_user_list_callback(self, request, response):
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
        save_root_path = ZenohTrainingManager.get_weight_save_root_path()
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

    def get_saved_policies_callback(self, request, response):
        saved_policy_path, saved_policy_type = ZenohInferenceManager.get_saved_policies()
        if not saved_policy_path and not saved_policy_type:
            self.get_logger().warning('No saved policies found')
            response.saved_policy_path = []
            response.saved_policy_type = []
            response.success = False
            response.message = 'No saved policies found'
        else:
            self.get_logger().info(f'Saved policies path: {saved_policy_path}')
            response.saved_policy_path = saved_policy_path
            response.saved_policy_type = saved_policy_type
            response.success = True
            response.message = 'Saved policies retrieved successfully'
        return response

    def get_training_info_callback(self, request, response):
        """
        Retrieve training configuration from a saved model.

        Loads configuration from train_config.json and populates TrainingInfo message.
        """
        try:
            # Validate request
            if not request.train_config_path:
                response.success = False
                response.message = 'train_config_path is required'
                return response

            # Clean up path (remove leading/trailing whitespace)
            train_config_path = request.train_config_path.strip()
            weight_save_root_path = ZenohTrainingManager.get_weight_save_root_path()
            config_path = weight_save_root_path / train_config_path

            # Check if config file exists
            if not config_path.exists():
                response.success = False
                response.message = f'Model config file not found: {config_path}'
                return response

            # Load and parse configuration
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    config_data = json.load(f)

                self.get_logger().info(f'Successfully loaded config from: {config_path}')

                # Populate TrainingInfo message from config
                training_info = response.training_info

                # Dataset configuration
                dataset_config = config_data.get('dataset', {})
                training_info.dataset = dataset_config.get('repo_id', '')

                # Policy configuration
                policy_config = config_data.get('policy', {})
                training_info.policy_type = policy_config.get('type', '')
                training_info.policy_device = policy_config.get('device', 'cuda')

                # Output directory (extract folder name)
                output_dir = config_data.get('output_dir', '')
                if output_dir:
                    training_info.output_folder_name = Path(output_dir).name
                else:
                    training_info.output_folder_name = ''

                # Training parameters with defaults
                training_info.seed = config_data.get('seed', 1000)
                training_info.num_workers = config_data.get('num_workers', 4)
                training_info.batch_size = config_data.get('batch_size', 8)
                training_info.steps = config_data.get('steps', 100000)
                training_info.eval_freq = config_data.get('eval_freq', 20000)
                training_info.log_freq = config_data.get('log_freq', 200)
                training_info.save_freq = config_data.get('save_freq', 1000)

                response.success = True
                response.message = \
                    f'Training configuration loaded successfully from {train_config_path}'

            except json.JSONDecodeError as e:
                response.success = False
                response.message = f'Invalid JSON in config file: {str(e)}'
                return response
            except KeyError as e:
                response.success = False
                response.message = f'Missing required field in config: {str(e)}'
                return response

        except Exception as e:
            self.get_logger().error(f'Error in get_training_info_callback: {str(e)}')
            response.success = False
            response.message = f'Failed to retrieve training info: {str(e)}'

        return response

    def set_robot_type_callback(self, request, response):
        try:
            self.get_logger().info(f'Setting robot type to: {request.robot_type}')
            self.operation_mode = 'collection'
            self.robot_type = request.robot_type
            self.clear_parameters()
            self.init_ros_params(self.robot_type)

            # Prepare rosbag subscriptions after communicator is initialized
            # This allows instant recording start without warmup delay
            if self.communicator is not None and self.communicator.rosbag_service_available:
                rosbag_topics = self.communicator.get_all_topics()
                self.communicator.prepare_rosbag(topics=rosbag_topics)
                topic_count = len(rosbag_topics)
                self.get_logger().info(
                    f'Rosbag prepared with {topic_count} topics - ready for recording')
            else:
                self.get_logger().warn('Rosbag service not available - prepare skipped')

            response.success = True
            response.message = f'Robot type set to {self.robot_type}'
            return response

        except Exception as e:
            self.get_logger().error(f'Failed to set robot type: {str(e)}')
            response.success = False
            response.message = f'Failed to set robot type: {str(e)}'
            return response

    def _init_hf_api_worker(self):
        """Initialize HF API Worker and status monitoring timer."""
        try:
            self.hf_api_worker = HfApiWorker()
            if self.hf_api_worker.start():
                self.get_logger().info('HF API Worker started successfully')
                # Initialize idle count
                self._hf_idle_count = 0
                # Initialize status monitoring timer
                self.hf_status_timer = TimerManager(node=self)
                self.hf_status_timer.set_timer(
                    timer_name='hf_status',
                    timer_frequency=2.0,
                    callback_function=self._hf_status_timer_callback
                )
                self.hf_status_timer.start(timer_name='hf_status')
                # Create publisher for HF status
                self.hf_status_publisher = self.create_publisher(
                    HFOperationStatus,
                    '/huggingface/status',
                    self.PUB_QOS_SIZE
                )
            else:
                self.get_logger().error('Failed to start HF API Worker')
        except Exception as e:
            self.get_logger().error(f'Error initializing HF API Worker: {str(e)}')

    def _hf_status_timer_callback(self):
        """Timer callback to check HF API Worker status and publish updates."""
        if self.hf_api_worker is None:
            return
        try:
            status = self.hf_api_worker.check_task_status()
            self._publish_hf_operation_status_msg(status)

            # Log status changes (avoid spamming logs)
            last_status = self._last_hf_status.get('status', 'Unknown') \
                if hasattr(self, '_last_hf_status') else 'Unknown'
            current_status = status.get('status', 'Unknown')

            if hasattr(self, '_last_hf_status') and last_status != current_status:
                self.get_logger().info(f'HF API Status changed: {last_status} -> {current_status}')

            self._last_hf_status = status
            # Idle status count and automatic shutdown
            if status.get('status', 'Unknown') == 'Idle':
                self._hf_idle_count = getattr(self, '_hf_idle_count', 0) + 1
                if self._hf_idle_count >= 5:
                    self.get_logger().info(
                        'HF API Worker idle for 5 cycles, shutting down worker and timer.')
                    self._cleanup_hf_api_worker()
            else:
                self._hf_idle_count = 0
        except Exception as e:
            self.get_logger().error(f'Error in HF status timer callback: {str(e)}')

    def _publish_hf_operation_status_msg(self, status):
        status_msg = HFOperationStatus()
        status_msg.operation = status.get('operation', 'Unknown')
        status_msg.status = status.get('status', 'Unknown')
        status_msg.repo_id = status.get('repo_id', '')
        status_msg.local_path = status.get('local_path', '')
        status_msg.message = status.get('message', '')

        progress_progress = status.get('progress', {})

        status_msg.progress_current = progress_progress.get('current', 0)
        status_msg.progress_total = progress_progress.get('total', 0)
        status_msg.progress_percentage = progress_progress.get('percentage', 0.0)

        # self.get_logger().info(f'HF API Status: {status_msg}')
        self.hf_status_publisher.publish(status_msg)

    def control_hf_server_callback(self, request, response):
        try:
            mode = request.mode
            repo_id = request.repo_id
            local_dir = request.local_dir
            repo_type = request.repo_type
            author = request.author

            if self.hf_cancel_on_progress:
                response.success = False
                response.message = 'HF API Worker is currently canceling'
                return response

            if mode == 'cancel':
                # Immediate cleanup - force stop the worker
                try:
                    self.hf_cancel_on_progress = True
                    self._cleanup_hf_api_worker_with_threading()
                    response.success = True
                    response.message = 'Cancellation started.'
                except Exception as e:
                    self.get_logger().error(f'Error during cancel: {e}')
                finally:
                    self.hf_cancel_on_progress = False
                    return response

            # Restart HF API Worker if it does not exist or is not running
            if self.hf_api_worker is None or not self.hf_api_worker.is_alive():
                self.get_logger().info('HF API Worker not running, restarting...')
                self._init_hf_api_worker()
            # Return error if the worker is busy
            if self.hf_api_worker.is_busy():
                self.get_logger().warning('HF API Worker is currently busy with another task')
                response.success = False
                response.message = 'HF API Worker is currently busy with another task'
                return response
            # Prepare request data for the worker
            request_data = {
                'mode': mode,
                'repo_id': repo_id,
                'local_dir': local_dir,
                'repo_type': repo_type,
                'author': author
            }
            # Send request to HF API Worker
            if self.hf_api_worker.send_request(request_data):
                self.get_logger().info(f'HF API request sent successfully: {mode} for {repo_id}')
                response.success = True
                response.message = f'HF API request started: {mode} for {repo_id}'
            else:
                self.get_logger().error('Failed to send request to HF API Worker')
                response.success = False
                response.message = 'Failed to send request to HF API Worker'
            return response
        except Exception as e:
            self.get_logger().error(f'Error in HF server callback: {str(e)}')
            response.success = False
            response.message = f'Error in HF server callback: {str(e)}'
            return response

    def get_replay_data_callback(self, request, response):
        """Handle replay data request for viewing recorded ROSbag data."""
        try:
            bag_path = request.bag_path
            self.get_logger().info(f'Getting replay data for: {bag_path}')

            result = self.replay_data_handler.get_replay_data(bag_path)

            response.success = result.get('success', False)
            response.message = result.get('message', '')

            if response.success:
                response.video_files = result.get('video_files', [])
                response.video_topics = result.get('video_topics', [])
                response.video_fps = result.get('video_fps', [])
                response.frame_indices = result.get('frame_indices', [])
                response.frame_timestamps = result.get('frame_timestamps', [])
                response.joint_timestamps = result.get('joint_timestamps', [])
                response.joint_names = result.get('joint_names', [])
                response.joint_positions = result.get('joint_positions', [])
                response.action_timestamps = result.get('action_timestamps', [])
                response.action_names = result.get('action_names', [])
                response.action_values = result.get('action_values', [])
                response.start_time = result.get('start_time', 0.0)
                response.end_time = result.get('end_time', 0.0)
                response.duration = result.get('duration', 0.0)
                response.video_server_port = self.VIDEO_SERVER_PORT
                response.bag_path = bag_path

            return response

        except Exception as e:
            self.get_logger().error(f'Error in get_replay_data_callback: {str(e)}')
            response.success = False
            response.message = f'Error getting replay data: {str(e)}'
            return response

    def browse_file_callback(self, request, response):
        """Handle file browsing requests."""
        try:
            if request.action == 'get_path':
                result = self.file_browse_utils.handle_get_path_action(
                    request.current_path)
            elif request.action == 'go_parent':
                target_files = None
                target_folders = None

                if hasattr(request, 'target_files') and request.target_files:
                    target_files = set(request.target_files)
                if hasattr(request, 'target_folders') and request.target_folders:
                    target_folders = set(request.target_folders)

                if target_files or target_folders:
                    result = self.file_browse_utils.handle_go_parent_with_target_check(
                        request.current_path,
                        target_files,
                        target_folders)
                else:
                    result = self.file_browse_utils.handle_go_parent_action(
                        request.current_path)
            elif request.action == 'browse':
                target_files = None
                target_folders = None

                if hasattr(request, 'target_files') and request.target_files:
                    target_files = set(request.target_files)
                if hasattr(request, 'target_folders') and request.target_folders:
                    target_folders = set(request.target_folders)

                if target_files or target_folders:
                    result = self.file_browse_utils.handle_browse_with_target_check(
                        request.current_path,
                        request.target_name,
                        target_files,
                        target_folders)
                else:
                    result = self.file_browse_utils.handle_browse_action(
                        request.current_path, request.target_name)
            else:
                result = {
                    'success': False,
                    'message': f'Unknown action: {request.action}',
                    'current_path': '',
                    'parent_path': '',
                    'selected_path': '',
                    'items': []
                }

            response.success = result['success']
            response.message = result['message']
            response.current_path = result['current_path']
            response.parent_path = result['parent_path']
            response.selected_path = result['selected_path']

            response.items = []
            for item_dict in result['items']:
                item = BrowserItem()
                item.name = item_dict['name']
                item.full_path = item_dict['full_path']
                item.is_directory = item_dict['is_directory']
                item.size = item_dict['size']
                item.modified_time = item_dict['modified_time']
                item.has_target_file = item_dict.get('has_target_file', False)
                response.items.append(item)

            return response

        except Exception as e:
            self.get_logger().error(f'Error in browse_file_callback: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
            response.current_path = ''
            response.parent_path = ''
            response.selected_path = ''
            response.items = []
            return response

    def handle_joystick_trigger(self, joystick_mode: str):
        """
        Handle joystick trigger for simplified recording control.

        - Right button: Toggle Start/Finish
          - If no session: Auto-create session and start recording
          - If idle: Start recording
          - If recording: Finish and save
        - Left button: Cancel (only during recording)
          - Discards current recording
        """
        self.get_logger().info(f'Joystick trigger: {joystick_mode}')

        if joystick_mode == 'right':
            # Toggle Start/Finish
            if self.data_manager is None:
                # Auto-create recording session with timestamp-based task name
                self.get_logger().info(
                    'Right button: No session exists, auto-creating...')
                if not self._auto_create_recording_session():
                    # Failed to create session (e.g., robot_type not set)
                    return
                # Start recording after session creation
                self.data_manager.start_recording()
                self.start_recording_time = time.perf_counter()
                self.communicator.publish_action_event('start')
            elif self.data_manager.is_recording():
                # Currently recording -> Finish and save
                self.get_logger().info('Right button: Finishing recording')
                self.stop_recording_and_save()
                self.communicator.publish_action_event('finish')
            else:
                # Not recording -> Start recording
                self.get_logger().info('Right button: Starting recording')
                # Restart timer if it was stopped (e.g., by UI Finish command)
                if self.timer_manager:
                    self.timer_manager.start(timer_name=self.operation_mode)
                self.on_recording = True
                self.data_manager.start_recording()
                self.start_recording_time = time.perf_counter()
                self.communicator.publish_action_event('start')

        elif joystick_mode == 'left':
            # Cancel during recording, or mark previous episode in idle
            if self.data_manager is None:
                self.get_logger().info('Left button ignored - no session')
            elif self.data_manager.is_recording():
                self.get_logger().info('Left button: Cancelling recording')
                self.cancel_current_recording()
                self.communicator.publish_action_event('cancel')
            else:
                # Idle state: toggle previous episode's needs_review
                result = self.data_manager.toggle_previous_episode_needs_review()
                if result is not None:
                    event = 'review_on' if result else 'review_off'
                    self.communicator.publish_action_event(event)
                else:
                    self.get_logger().info(
                        'Left button: No previous episode to toggle')

        elif joystick_mode == 'right_long_time':
            self.get_logger().info('Right long press - reserved for future use')

        elif joystick_mode == 'left_long_time':
            self.get_logger().info('Left long press - reserved for future use')

        else:
            self.get_logger().info(f'Unknown joystick trigger: {joystick_mode}')

    def _auto_create_recording_session(self) -> bool:
        """
        Auto-create a recording session with timestamp-based task name.

        Task name format: task_YYMMDDHHMMSS (e.g., task_260204181101)

        Returns:
            bool: True if session created successfully, False otherwise
        """
        # Check if robot_type is set
        if not hasattr(self, 'robot_type') or self.robot_type is None:
            self.get_logger().error(
                'Cannot auto-create session: robot_type is not set. '
                'Please set robot type from UI first.')
            return False

        # Generate timestamp-based task name
        timestamp = datetime.now().strftime('%y%m%d%H%M%S')
        task_name = f'task_{timestamp}'

        self.get_logger().info(f'Auto-creating recording session: {task_name}')

        # Create TaskInfo with timestamp-based values
        task_info = TaskInfo()
        task_info.task_name = task_name
        task_info.task_type = ''
        task_info.task_instruction = [task_name]
        task_info.policy_path = ''
        task_info.tags = []
        task_info.record_inference_mode = False

        # Initialize recording session
        self.operation_mode = 'collection'
        self.init_robot_control_parameters_from_user_task(task_info)
        self.on_recording = True

        self.get_logger().info(
            f'Auto-created recording session: {task_name}')
        return True

    def _cleanup_hf_api_worker_with_threading(self):
        """
        Non-blocking cleanup of HF API Worker using threading.

        This method starts a separate thread to run the existing
        _cleanup_hf_api_worker method, preventing the main process.
        from blocking during shutdown.
        """
        import threading
        import time

        def cleanup_worker_thread():
            """Worker thread to run _cleanup_hf_api_worker."""
            try:
                # Call the existing cleanup method
                self._cleanup_hf_api_worker()
            except Exception as e:
                self.get_logger().error(f'Error in cleanup worker thread: {e}')

        try:
            if self.hf_status_timer is None and self.hf_api_worker is None:
                self.get_logger().info('No HF API components to cleanup')
                return

            self.get_logger().info('Starting non-blocking HF API Worker cleanup...')

            # Start cleanup thread
            cleanup_thread = threading.Thread(target=cleanup_worker_thread, daemon=True)
            cleanup_thread.start()

            # Reset references immediately (don't wait for cleanup to complete)
            self.hf_status_timer = None
            self.hf_api_worker = None

            self.get_logger().info('HF API Worker cleanup thread started')

            # Publish cancel status messages
            for i in range(3):
                self._publish_hf_operation_status_msg({
                    'status': 'Idle',
                    'operation': 'stop',
                    'repo_id': '',
                    'local_path': '',
                    'message': 'Canceled by stop command',
                    'progress': {
                        'current': 0,
                        'total': 0,
                        'percentage': 0.0,
                    }
                })
                time.sleep(0.5)

        except Exception as e:
            self.get_logger().error(
                f'Error starting non-blocking HF API Worker cleanup: {str(e)}'
            )
            # Fallback to blocking cleanup if threading fails
            self._cleanup_hf_api_worker()
        finally:
            self.hf_cancel_on_progress = False

    def _cleanup_hf_api_worker(self):
        """Cleanup HF API Worker and related timers."""
        try:
            if self.hf_status_timer is not None:
                self.hf_status_timer.stop(timer_name='hf_status')
                self.hf_status_timer = None

            if self.hf_api_worker is not None:
                self.hf_api_worker.stop()
                self.hf_api_worker = None

            self.get_logger().info('HF API Worker cleaned up successfully')
        except Exception as e:
            self.get_logger().error(f'Error cleaning up HF API Worker: {str(e)}')

    def _init_mp4_conversion_worker(self):
        """Initialize MP4 Conversion Worker and status monitoring timer."""
        try:
            self.mp4_conversion_worker = Mp4ConversionWorker()
            if self.mp4_conversion_worker.start():
                self.get_logger().info('MP4 Conversion Worker started successfully')
                # Initialize idle count
                self._mp4_idle_count = 0
                # Initialize status monitoring timer
                self.mp4_status_timer = TimerManager(node=self)
                self.mp4_status_timer.set_timer(
                    timer_name='mp4_status',
                    timer_frequency=2.0,
                    callback_function=self._mp4_status_timer_callback
                )
                self.mp4_status_timer.start(timer_name='mp4_status')
            else:
                self.get_logger().error('Failed to start MP4 Conversion Worker')
        except Exception as e:
            self.get_logger().error(f'Error initializing MP4 Conversion Worker: {str(e)}')

    def _mp4_status_timer_callback(self):
        """Timer callback to check MP4 Conversion Worker status."""
        if self.mp4_conversion_worker is None:
            return
        try:
            status = self.mp4_conversion_worker.check_task_status()

            # Log status changes
            last_status = self._last_mp4_status.get('status', 'Unknown') \
                if hasattr(self, '_last_mp4_status') else 'Unknown'
            current_status = status.get('status', 'Unknown')

            if hasattr(self, '_last_mp4_status') and last_status != current_status:
                self.get_logger().info(
                    f'MP4 Conversion Status changed: {last_status} -> {current_status}'
                )

            self._last_mp4_status = status

            # Update task status with conversion progress if converting
            if current_status == 'Converting' and self.communicator is not None:
                progress = status.get('progress', {})
                percentage = progress.get('percentage', 0.0)
                if self.data_manager is not None:
                    current_record_status = self.data_manager.get_current_record_status()
                else:
                    current_record_status = TaskStatus()
                    if hasattr(self, 'robot_type'):
                        current_record_status.robot_type = self.robot_type
                current_record_status.phase = TaskStatus.CONVERTING
                current_record_status.encoding_progress = percentage
                self.communicator.publish_status(status=current_record_status)

            # Handle completion or failure - publish final status to frontend
            if current_status in ('Success', 'Failed'):
                if current_status == 'Success':
                    self.get_logger().info(
                        f'MP4 conversion completed: {status.get("message", "")}'
                    )
                else:
                    self.get_logger().error(
                        f'MP4 conversion failed: {status.get("message", "")}'
                    )
                # Publish READY status so frontend can detect conversion end
                if self.communicator is not None:
                    if self.data_manager is not None:
                        final_status = self.data_manager.get_current_record_status()
                    else:
                        final_status = TaskStatus()
                        if hasattr(self, 'robot_type'):
                            final_status.robot_type = self.robot_type
                    final_status.phase = TaskStatus.READY
                    self.communicator.publish_status(status=final_status)

            # Idle status count and automatic shutdown
            if current_status == 'Idle':
                self._mp4_idle_count = getattr(self, '_mp4_idle_count', 0) + 1
                if self._mp4_idle_count >= 5:
                    self.get_logger().info(
                        'MP4 Conversion Worker idle for 5 cycles, shutting down.'
                    )
                    self._cleanup_mp4_conversion_worker()
            else:
                self._mp4_idle_count = 0
        except Exception as e:
            self.get_logger().error(f'Error in MP4 status timer callback: {str(e)}')

    def _cleanup_mp4_conversion_worker(self):
        """Cleanup MP4 Conversion Worker and related timers."""
        try:
            if self.mp4_status_timer is not None:
                self.mp4_status_timer.stop(timer_name='mp4_status')
                self.mp4_status_timer = None

            if self.mp4_conversion_worker is not None:
                self.mp4_conversion_worker.stop()
                self.mp4_conversion_worker = None

            self.get_logger().info('MP4 Conversion Worker cleaned up successfully')
        except Exception as e:
            self.get_logger().error(f'Error cleaning up MP4 Conversion Worker: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    node = PhysicalAIServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup workers before destroying node
        node._cleanup_hf_api_worker()
        node._cleanup_mp4_conversion_worker()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
