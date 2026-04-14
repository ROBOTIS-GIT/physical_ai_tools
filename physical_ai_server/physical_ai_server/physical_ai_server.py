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
    GetTrainingInfo,
    GetUserList,
    HFEndpointList,
    SelectHFEndpoint,
    SendCommand,
    SendTrainingCommand,
    SetHFUser,
    SetRobotType,
)

from physical_ai_server.communication.communicator import Communicator
from physical_ai_server.data_processing.data_manager import DataManager
from physical_ai_server.data_processing.hf_api_worker import HfApiWorker
from physical_ai_server.data_processing.hf_endpoint_store import HFEndpointStore
from physical_ai_server.data_processing.mp4_conversion_worker import Mp4ConversionWorker
from physical_ai_server.data_processing.replay_data_handler import ReplayDataHandler
from physical_ai_server.inference.inference_manager import InferenceManager
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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
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

        # Callback groups for MultiThreadedExecutor.
        # Separating service servers from service clients prevents deadlock
        # when a service callback needs to call another service (e.g., FINISH
        # calling /groot/stop).
        self._service_cb_group = MutuallyExclusiveCallbackGroup()
        self._client_cb_group = ReentrantCallbackGroup()

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
        # Zenoh managers for training (Docker container communication)
        self.training_manager: Optional[ZenohTrainingManager] = None
        # Action chunk inference manager (GR00T, LeRobot, etc.)
        self.inference_manager: Optional[InferenceManager] = None

        # Initialize HF API Worker
        self.hf_endpoint_store = HFEndpointStore()
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
            ('/huggingface/list_endpoints', HFEndpointList, self.list_hf_endpoints_callback),
            ('/huggingface/select_endpoint', SelectHFEndpoint, self.select_hf_endpoint_callback),
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
            self.create_service(
                service_type, service_name, callback,
                callback_group=self._service_cb_group,
            )

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
            'command_topic_list',
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

        control_hz = getattr(task_info, 'control_hz', 0) or 10
        self._control_hz = control_hz

        # Stop previous timer before creating a new one
        if hasattr(self, 'timer_manager') and self.timer_manager is not None:
            self.timer_manager.stop_all()

        self.timer_manager = TimerManager(node=self)
        self.timer_manager.set_timer(
            timer_name=self.operation_mode,
            timer_frequency=control_hz,
            callback_function=self.timer_callback_dict[self.operation_mode]
        )
        self.timer_manager.start(timer_name=self.operation_mode)
        self.get_logger().info(
            f'Robot control parameters initialized (control_hz={control_hz})')

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
        """Validate ``token`` against ``endpoint`` and persist on success.

        The previous flow validated against a single global token; this one
        keeps a per-endpoint store so the user can have one token for the
        official hub and another for the internal hub at the same time.
        """
        endpoint = (request.endpoint or '').strip()
        label = (request.label or '').strip()
        token = (request.token or '').strip()

        if not endpoint:
            response.user_id_list = []
            response.success = False
            response.message = 'endpoint is required'
            return response
        if not token:
            response.user_id_list = []
            response.success = False
            response.message = 'token is required'
            return response

        try:
            user_ids = DataManager.whoami_huggingface(endpoint, token)
            if not user_ids:
                response.user_id_list = []
                response.success = False
                response.message = (
                    f'Token validation timed out for endpoint {endpoint}'
                )
                return response

            primary_user = user_ids[0] if user_ids else ''
            self.hf_endpoint_store.set(
                endpoint=endpoint,
                label=label,
                token=token,
                user_id=primary_user,
            )
            self.get_logger().info(
                f'Registered HF token for {endpoint} ({primary_user})'
            )
            response.user_id_list = user_ids
            response.success = True
            response.message = (
                f'Token validated and stored for {endpoint} ({primary_user})'
            )
        except Exception as e:
            self.get_logger().error(f'Error in set_hf_user_callback: {str(e)}')
            response.user_id_list = []
            response.success = False
            response.message = f'Error: {str(e)}'

        return response

    def get_hf_user_callback(self, request, response):
        """Return the user list for ``request.endpoint`` (empty = active)."""
        endpoint = (request.endpoint or '').strip()
        try:
            entry = self.hf_endpoint_store.resolve(endpoint)
            if entry is None:
                response.user_id_list = []
                response.success = False
                response.message = (
                    'No HuggingFace endpoint registered yet — register a token '
                    'from the UI first.'
                )
                return response

            user_ids = DataManager.whoami_huggingface(entry.endpoint, entry.token)
            if not user_ids:
                response.user_id_list = []
                response.success = False
                response.message = (
                    f'Token validation timed out for {entry.endpoint}'
                )
                return response

            response.user_id_list = user_ids
            response.success = True
            response.message = (
                f'Resolved {len(user_ids)} user id(s) for {entry.endpoint}'
            )
        except Exception as e:
            self.get_logger().error(f'Error in get_hf_user_callback: {str(e)}')
            response.user_id_list = []
            response.success = False
            response.message = f'Error: {str(e)}'

        return response

    def list_hf_endpoints_callback(self, request, response):
        """Return every registered endpoint plus the currently active one."""
        try:
            entries = self.hf_endpoint_store.list()
            active = self.hf_endpoint_store.get_active()
            response.endpoints = [e.endpoint for e in entries]
            response.labels = [e.label for e in entries]
            response.user_ids = [e.user_id for e in entries]
            response.active = active.endpoint if active else ''
            response.success = True
            response.message = f'{len(entries)} endpoint(s) registered'
        except Exception as e:
            self.get_logger().error(f'Error in list_hf_endpoints_callback: {str(e)}')
            response.endpoints = []
            response.labels = []
            response.user_ids = []
            response.active = ''
            response.success = False
            response.message = f'Error: {str(e)}'
        return response

    def select_hf_endpoint_callback(self, request, response):
        """Set the active endpoint. Empty string clears the selection."""
        endpoint = (request.endpoint or '').strip()
        try:
            ok = self.hf_endpoint_store.set_active(endpoint)
            if not ok:
                response.success = False
                response.message = (
                    f'Endpoint not registered: {endpoint}. Register a token '
                    f'for it first.'
                )
                return response
            response.success = True
            response.message = (
                f'Active endpoint set to {endpoint or "<none>"}'
            )
        except Exception as e:
            self.get_logger().error(f'Error in select_hf_endpoint_callback: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
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
                # Rosbag is started directly in the service handler
                # (START_RECORD / START_INFERENCE_RECORD).
                # No action needed here.
                pass

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

    def stop_current_segment(self):
        """Stop the active segment, save its rosbag, refresh episode_info.json."""
        if self.data_manager is None:
            return
        self.communicator.stop_rosbag()
        self.data_manager.stop_segment()
        urdf_path = self.params.get('urdf_path', '')
        if urdf_path:
            self.data_manager.save_robotis_metadata(urdf_path=urdf_path)
        else:
            self.data_manager._write_episode_info()
        self.previous_data_manager_status = 'idle'

    def finish_current_episode(self, needs_review: bool = False):
        """Finalize episode (after all segments stopped) and increment counter."""
        if self.data_manager is None:
            return
        self.data_manager.finish_episode(needs_review=needs_review)
        self.previous_data_manager_status = 'idle'

    def _start_finalize_worker(self, task_info):
        """Spawn a daemon thread that merges scratch + moves it to archive."""
        if (getattr(self, '_merge_thread', None) is not None
                and self._merge_thread.is_alive()):
            raise RuntimeError('Merge already in progress')
        if self.data_manager is None:
            raise RuntimeError('Data manager not initialized')

        urdf_path = ''
        if self.params:
            urdf_path = self.params.get('urdf_path', '') or ''

        def _worker():
            try:
                archive_dir = self.data_manager.finalize_to_archive(
                    task_info, urdf_path=urdf_path)
                self.get_logger().info(
                    f'Finalize complete: {archive_dir}')
            except Exception as e:
                self.get_logger().error(f'Finalize failed: {e}')

        self._merge_thread = threading.Thread(target=_worker, daemon=True)
        self._merge_thread.start()

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
        Timer callback for inference mode (runs at control_hz).

        Pops one action from the action buffer and publishes
        joint commands via communicator. Buffer is refilled in
        the background by InferenceManager.
        """
        if not self.on_inference or self.inference_manager is None:
            return

        current_status = TaskStatus()
        if hasattr(self, 'robot_type') and self.robot_type:
            current_status.robot_type = self.robot_type

        # Handle async load error
        load_error = self.inference_manager.load_error
        if load_error:
            self.get_logger().error(f'Inference load failed: {load_error}')
            current_status.phase = TaskStatus.READY
            current_status.error = load_error
            self.communicator.publish_status(status=current_status)
            self._stop_groot_inference()
            self.on_inference = False
            return

        # Still loading policy
        if self.inference_manager.is_loading:
            current_status.phase = TaskStatus.LOADING
            self.communicator.publish_status(status=current_status)
            return

        # Paused — model loaded but not inferencing
        if self.inference_manager.is_paused:
            current_status.phase = TaskStatus.PAUSED
            self.communicator.publish_status(status=current_status)
            return

        # Ready — pop action and publish
        joint_msg_datas = self.inference_manager.pop_action()
        if joint_msg_datas is not None:
            self.communicator.publish_action(joint_msg_datas)
        else:
            if not hasattr(self, '_pop_none_count'):
                self._pop_none_count = 0
            self._pop_none_count += 1
            if self._pop_none_count <= 5 or self._pop_none_count % 100 == 0:
                self.get_logger().warning(
                    f'pop_action() returned None '
                    f'(count={self._pop_none_count}, '
                    f'buffer={len(self.inference_manager._action_buffer)}, '
                    f'requesting={self.inference_manager._requesting})'
                )

        current_status.phase = TaskStatus.INFERENCING
        self.communicator.publish_status(status=current_status)

    def user_training_interaction_callback(self, request, response):
        """
        Handle training command requests (START/FINISH).

        Supports both new training and resume functionality with proper validation.
        """
        try:
            if request.command == SendTrainingCommand.Request.START:
                # Initialize training components (ROS2 services with rmw_zenoh)
                self.training_manager = ZenohTrainingManager(
                    node=self, client_cb_group=self._client_cb_group,
                )
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
            if request.command in (
                SendCommand.Request.START_RECORD,
                SendCommand.Request.START_SEGMENT,
            ):
                # Scratch flow: task_info is irrelevant at segment-start time;
                # only primitive_description is consumed from the request.
                task_info = request.task_info
                self._last_ui_task_info = task_info

                if self.data_manager is None:
                    self.get_logger().info('Initializing new recording session')
                    self.operation_mode = 'collection'
                    self.init_robot_control_parameters_from_user_task(task_info)
                else:
                    # Refresh cached task_info so any later finalize can pick up
                    # the latest user-entered values (though the actual merge
                    # always overrides with the task_info in the merge request).
                    self.data_manager._task_info = task_info
                    if self.timer_manager:
                        self.timer_manager.start(timer_name=self.operation_mode)

                # Ensure rosbag subscriptions are ready before recording
                rosbag_topics = self.communicator.get_all_topics()
                self.communicator.prepare_rosbag(topics=rosbag_topics)

                self.get_logger().info('Starting recording')
                rosbag_path = self.data_manager.get_save_rosbag_path(allow_idle=True)
                if not rosbag_path:
                    response.success = False
                    response.message = 'Failed to resolve rosbag path'
                    return response

                try:
                    self.communicator.start_rosbag(rosbag_uri=rosbag_path)
                except Exception as e:
                    response.success = False
                    response.message = f'Failed to start rosbag: {str(e)}'
                    return response

                primitive = getattr(
                    task_info, 'primitive_description', '') or ''
                self.data_manager.start_segment(primitive)
                self.on_recording = True
                self.start_recording_time = time.perf_counter()
                self.communicator.publish_action_event('start')
                response.success = True
                response.message = (
                    f'Segment {self.data_manager._current_segment_index - 0} '
                    f'started'
                )

            elif request.command == SendCommand.Request.STOP_SEGMENT:
                if (self.data_manager is None
                        or not self.data_manager.is_recording()):
                    response.success = False
                    response.message = 'Not currently recording a segment'
                else:
                    self.get_logger().info('Stopping current segment')
                    self.stop_current_segment()
                    self.on_recording = False
                    self.communicator.publish_action_event('finish')
                    response.success = True
                    response.message = (
                        f'Segment stopped. Total segments: '
                        f'{len(self.data_manager._segments_meta)}'
                    )

            elif request.command == SendCommand.Request.DISCARD_SEGMENT:
                if self.data_manager is None:
                    response.success = False
                    response.message = 'Data manager not initialized'
                else:
                    try:
                        self.data_manager.discard_segment(
                            int(request.segment_index))
                        response.success = True
                        response.message = (
                            f'Segment {request.segment_index} discarded'
                        )
                    except Exception as e:
                        response.success = False
                        response.message = f'Discard failed: {e}'

            elif request.command == SendCommand.Request.FINISH_EPISODE:
                # Deprecated in scratch flow. Kept for backward-compatibility;
                # responds OK without mutating state.
                response.success = True
                response.message = (
                    'FINISH_EPISODE is deprecated; use MERGE_EPISODE '
                    'or DISCARD_EPISODE'
                )
                return response

            elif request.command == SendCommand.Request.DISCARD_EPISODE:
                if self.data_manager is None:
                    response.success = False
                    response.message = 'Data manager not initialized'
                else:
                    try:
                        if self.data_manager.is_recording():
                            self.stop_current_segment()
                            self.on_recording = False
                        self.data_manager.discard_episode()
                        response.success = True
                        response.message = 'Pending episode discarded'
                    except Exception as e:
                        response.success = False
                        response.message = f'Discard episode failed: {e}'

            elif request.command == SendCommand.Request.MERGE_EPISODE:
                if self.data_manager is None:
                    response.success = False
                    response.message = 'Data manager not initialized'
                else:
                    try:
                        self._start_finalize_worker(request.task_info)
                        response.success = True
                        response.message = 'Merge started'
                    except Exception as e:
                        response.success = False
                        response.message = f'Merge failed to start: {e}'

            elif request.command == SendCommand.Request.START_INFERENCE:
                self.operation_mode = 'inference'
                task_info = request.task_info

                task_instruction = (
                    task_info.task_instruction[0]
                    if task_info.task_instruction
                    else ''
                )

                # If model already loaded and paused, just resume
                if (
                    self.inference_manager is not None
                    and self.inference_manager.is_paused
                ):
                    self.get_logger().info(
                        'Model already loaded, resuming inference'
                    )
                    self.inference_manager.resume(task_instruction)
                    self.on_inference = True
                    response.success = True
                    response.message = 'Inference resumed (model already loaded)'
                else:
                    # Clean up existing inference session if any
                    if self.inference_manager is not None:
                        self._stop_groot_inference()

                    self.init_robot_control_parameters_from_user_task(task_info)
                    self.joint_topic_types = self.communicator.get_publisher_msg_types()

                    # Determine service prefix from policy type
                    service_prefix = self._determine_service_prefix(task_info)

                    # Create and start inference manager
                    self.inference_manager = InferenceManager(
                        node=self,
                        joint_topic_types=self.joint_topic_types,
                        joint_order=self.joint_order,
                        service_prefix=service_prefix,
                        on_chunk_received=self.communicator.publish_action_chunk,
                        control_hz=self._control_hz,
                        client_cb_group=self._client_cb_group,
                    )

                    self.inference_manager.start(
                        model_path=task_info.policy_path,
                        embodiment_tag='new_embodiment',
                        robot_type=self.robot_type,
                        task_instruction=task_instruction,
                    )

                    if task_info.record_inference_mode:
                        self.on_recording = True

                    response.success = True
                    response.message = (
                        f'{service_prefix.strip("/").upper()} inference loading'
                    )

                self.on_inference = True
                self.start_recording_time = time.perf_counter()

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

                if len(source_folders) >= 2:
                    # === Merge & Convert mode (requires at least 2 sources) ===
                    dataset_path = base_path / task_name

                    # Validate source folders
                    source_paths = []
                    for folder_name in source_folders:
                        if folder_name.startswith('/'):
                            src = Path(folder_name)
                        else:
                            # Try with robot_type prefix first, then raw name
                            src = base_path / f'{self.robot_type}_{folder_name}'
                            if not src.exists():
                                src = base_path / folder_name
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
                        self._stop_groot_inference()
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

                    elif request.command == SendCommand.Request.STOP_INFERENCE:
                        if self.inference_manager is not None:
                            self.inference_manager.pause()
                            response.success = True
                            response.message = 'Inference paused'
                        else:
                            response.success = False
                            response.message = 'No inference session active'

                    elif request.command == SendCommand.Request.RESUME_INFERENCE:
                        if self.inference_manager is not None:
                            task_instruction = (
                                request.task_info.task_instruction[0]
                                if request.task_info.task_instruction
                                else ''
                            )
                            self._pop_none_count = 0
                            self.inference_manager.resume(
                                task_instruction=task_instruction
                            )
                            response.success = True
                            response.message = 'Inference resumed'
                        else:
                            response.success = False
                            response.message = 'No inference session active'

                    elif request.command == SendCommand.Request.START_INFERENCE_RECORD:
                        self.get_logger().info(
                            'Starting recording during inference'
                        )
                        if self.data_manager is not None:
                            # Prepare rosbag subscriptions before starting
                            rosbag_topics = \
                                self.communicator.get_all_topics()
                            self.communicator.prepare_rosbag(
                                topics=rosbag_topics
                            )
                            rosbag_path = \
                                self.data_manager.get_save_rosbag_path(
                                    allow_idle=True)
                            if not rosbag_path:
                                response.success = False
                                response.message = \
                                    'Failed to resolve rosbag path'
                                return response

                            try:
                                self.communicator.start_rosbag(
                                    rosbag_uri=rosbag_path
                                )
                            except Exception as e:
                                response.success = False
                                response.message = \
                                    f'Failed to start rosbag: {str(e)}'
                                return response

                            self.data_manager.start_recording()
                            self.on_recording = True
                            self.start_recording_time = time.perf_counter()
                            self.communicator.publish_action_event('start')
                            response.success = True
                            response.message = (
                                'Recording started during inference'
                            )
                        else:
                            response.success = False
                            response.message = 'Data manager not initialized'

                    elif request.command == SendCommand.Request.STOP_INFERENCE_RECORD:
                        self.get_logger().info(
                            'Stopping and saving recording'
                        )
                        if (
                            self.data_manager
                            and self.data_manager.is_recording()
                        ):
                            self.stop_recording_and_save()
                            self.on_recording = False
                            self.communicator.publish_action_event('finish')
                            response.success = True
                            response.message = 'Recording saved'
                        else:
                            response.success = False
                            response.message = 'Not currently recording'

                    elif request.command == SendCommand.Request.CANCEL_INFERENCE_RECORD:
                        self.get_logger().info(
                            'Cancelling recording during inference'
                        )
                        if (
                            self.data_manager
                            and self.data_manager.is_recording()
                        ):
                            self.cancel_current_recording()
                            self.on_recording = False
                            self.communicator.publish_action_event('cancel')
                            response.success = True
                            response.message = 'Recording cancelled'
                        else:
                            response.success = False
                            response.message = 'Not currently recording'

                    elif request.command == SendCommand.Request.FINISH:
                        # Finish all operations
                        self.get_logger().info('Finishing all operations')
                        if self.data_manager and self.data_manager.is_recording():
                            self.stop_recording_and_save()
                            self.communicator.publish_action_event('finish')
                            self.communicator.finish_rosbag()
                        self._stop_groot_inference()
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
                        self._stop_groot_inference()
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
                        self._stop_groot_inference()
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

            # Scratch flow: initialize DataManager *and* the collection timer
            # eagerly. The timer is what drives TaskStatus publishing, so
            # without it the UI never sees segment_count updates. Going
            # through init_robot_control_parameters_from_user_task also
            # runs restore_pending() inside DataManager.__init__.
            if self.data_manager is None:
                try:
                    self.operation_mode = 'collection'
                    self.init_robot_control_parameters_from_user_task(
                        task_info=None)
                    restored = len(self.data_manager._segments_meta)
                    if restored:
                        self.get_logger().info(
                            f'Restored {restored} pending segment(s) from '
                            f'{DataManager.PENDING_ROSBAG_PATH}')
                except Exception as e:
                    self.get_logger().warning(
                        f'Failed to eagerly init DataManager: {e}')

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
            request_endpoint = (getattr(request, 'endpoint', '') or '').strip()

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

            # Resolve which endpoint+token to use for this request. ``endpoint``
            # field on the request takes precedence; otherwise we fall back to
            # the active endpoint stored on disk.
            entry = self.hf_endpoint_store.resolve(request_endpoint)
            if entry is None:
                response.success = False
                response.message = (
                    f'No HuggingFace endpoint registered '
                    f'(requested: {request_endpoint or "<active>"}). '
                    f'Register a token from the UI first.'
                )
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
            # Prepare request data for the worker. The endpoint and token
            # travel with the request so the worker process never has to
            # consult global state.
            request_data = {
                'mode': mode,
                'repo_id': repo_id,
                'local_dir': local_dir,
                'repo_type': repo_type,
                'author': author,
                'endpoint': entry.endpoint,
                'token': entry.token,
            }
            # Send request to HF API Worker
            if self.hf_api_worker.send_request(request_data):
                self.get_logger().info(
                    f'HF API request sent: {mode} {repo_id} via {entry.endpoint}'
                )
                response.success = True
                response.message = (
                    f'HF API request started: {mode} for {repo_id} via {entry.endpoint}'
                )
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

    # LeRobot policy types (used for service_prefix detection)
    LEROBOT_POLICIES = {
        'tdmpc', 'diffusion', 'act', 'vqbet', 'pi0', 'pi0_fast', 'pi05',
        'smolvla', 'xvla', 'sac',
    }

    def _determine_service_prefix(self, task_info) -> str:
        """Determine inference service prefix from task_info or policy config.

        1. If task_info has service_type field, use it directly.
        2. Otherwise, read policy_path/config.json to detect policy type.
        3. LeRobot policy types -> "/lerobot", default -> "/groot".
        """
        # Check for explicit service_type in task_info
        service_type = getattr(task_info, 'service_type', None)
        if service_type:
            prefix = f'/{service_type.strip("/")}'
            self.get_logger().info(f'Service prefix from task_info: {prefix}')
            return prefix

        # Detect from policy config
        policy_path = getattr(task_info, 'policy_path', '')
        if policy_path:
            config_path = Path(policy_path) / 'config.json'
            if config_path.exists():
                try:
                    with open(config_path) as f:
                        config = json.load(f)
                    policy_type = config.get('type', '')
                    if policy_type in self.LEROBOT_POLICIES:
                        self.get_logger().info(
                            f'Detected LeRobot policy type: {policy_type}'
                        )
                        return '/lerobot'
                except Exception as e:
                    self.get_logger().warning(
                        f'Failed to read policy config: {e}'
                    )

        # Default to groot for backward compatibility
        return '/groot'

    def _stop_groot_inference(self):
        """Stop GR00T inference and cleanup (non-blocking).

        Clears the inference_manager reference immediately so no new
        actions are dispatched, then runs the blocking cleanup (thread
        join + container stop service call) in a background thread.
        """
        if self.inference_manager is not None:
            manager = self.inference_manager
            self.inference_manager = None

            def _cleanup():
                try:
                    manager.stop()
                except Exception as e:
                    self.get_logger().error(f'Error stopping inference: {e}')

            threading.Thread(target=_cleanup, daemon=True).start()

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
                    return
                # Start rosbag, then recording
                rosbag_path = self.data_manager.get_save_rosbag_path(allow_idle=True)
                if not rosbag_path:
                    self.get_logger().error('Failed to resolve rosbag path')
                    return
                try:
                    self.communicator.start_rosbag(rosbag_uri=rosbag_path)
                except Exception as e:
                    self.get_logger().error(f'Failed to start rosbag: {e}')
                    return
                self.data_manager.start_recording()
                self.on_recording = True
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
                if self.timer_manager:
                    self.timer_manager.start(timer_name=self.operation_mode)
                rosbag_path = self.data_manager.get_save_rosbag_path(allow_idle=True)
                if not rosbag_path:
                    self.get_logger().error('Failed to resolve rosbag path')
                    return
                try:
                    self.communicator.start_rosbag(rosbag_uri=rosbag_path)
                except Exception as e:
                    self.get_logger().error(f'Failed to start rosbag: {e}')
                    return
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
        Auto-create a recording session. If the UI has already sent a
        START_RECORD with task_info (cached in ``_last_ui_task_info``),
        reuse that so the folder name matches what the user typed
        (Task_{num}_{name}_MCAP). Otherwise fall back to a
        timestamp-based name.

        Returns:
            bool: True if session created successfully, False otherwise
        """
        if not hasattr(self, 'robot_type') or self.robot_type is None:
            self.get_logger().error(
                'Cannot auto-create session: robot_type is not set. '
                'Please set robot type from UI first.')
            return False

        cached = getattr(self, '_last_ui_task_info', None)
        if cached is not None and cached.task_name:
            task_info = cached
            self.get_logger().info(
                f'Joystick: reusing UI task_info (task_name={task_info.task_name})')
        else:
            self.get_logger().error(
                'Cannot start recording from joystick: '
                'please start the first episode from the UI so task info '
                '(Task Num / Task Name) is set.')
            return False

        self.operation_mode = 'collection'
        self.init_robot_control_parameters_from_user_task(task_info)
        self.on_recording = True
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
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup workers before destroying node
        node._cleanup_hf_api_worker()
        node._cleanup_mp4_conversion_worker()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
