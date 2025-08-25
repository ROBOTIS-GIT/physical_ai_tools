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

import glob
import multiprocessing
import os
from pathlib import Path
import threading
import time
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from physical_ai_interfaces.msg import TaskStatus, TrainingStatus
from physical_ai_interfaces.srv import (
    GetDatasetList,
    GetHFUser,
    GetModelWeightList,
    GetPolicyList,
    GetRobotTypeList,
    GetUserList,
    SendCommand,
    SendTrainingCommand,
    SetHFUser,
    SetRobotType,
)

from physical_ai_server.communication.communicator import Communicator
from physical_ai_server.data_processing.data_manager import DataManager
from physical_ai_server.inference import InferenceBase, InferenceFactory
from physical_ai_server.inference.action_chunk_processor import ActionChunkProcessor
from physical_ai_server.inference.inference_multi_process import InferenceWorker
from physical_ai_server.inference.visualize_inference_result import InferenceResultVisualizer
from physical_ai_server.timer.timer_manager import TimerManager
from physical_ai_server.training.training_manager import TrainingManager
from physical_ai_server.utils.parameter_utils import (
    declare_parameters,
    load_parameters,
    log_parameters,
)

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class PhysicalAIServer(Node):
    # Define operation modes (constants taken from Communicator)

    DEFAULT_SAVE_ROOT_PATH = Path.home() / '.cache/huggingface/lerobot'
    DEFAULT_TOPIC_TIMEOUT = 5.0  # seconds
    PUB_QOS_SIZE = 10
    TRAINING_STATUS_TIMER_FREQUENCY = 0.5  # seconds
    INFERENCE_WORKER_FREQUENCY = 100.0

    def __init__(self):
        super().__init__('physical_ai_server')
        self.get_logger().info('Start Physical AI Server')

        self.params = None
        self.total_joint_order = None
        self.on_recording = False
        self.on_inference = False

        self.robot_type_list = self.get_robot_type_list()
        self.start_recording_time: float = 0.0

        self.training_thread = None
        self.is_training = False
        self.training_status_timer = None

        self._init_core_components()

        self._init_ros_service()

        self._setup_timer_callbacks()

        # Multi-process inference state
        self._used_action_count = 0
        self.remaining_actions = []
        self.inference_worker: Optional[InferenceWorker] = None
        self.inference_pending = False

        # Track inference timing for proper action offset
        self.inference_start_action_count = 0
        self.last_executed_action = None

        # Observation tracking for debugging stale data issues
        self.last_observation_data = None
        self.observation_change_history = []

        # Initialize inference result visualizer
        self.visualizer = InferenceResultVisualizer(
            logger=self.get_logger(),
            enabled=True
        )

        # Initialize action chunk processor
        self.action_chunk_processor = ActionChunkProcessor(logger=self.get_logger())

    def _init_core_components(self):
        self.communicator: Optional[Communicator] = None
        self.data_manager: Optional[DataManager] = None
        self.timer_manager: Optional[TimerManager] = None
        self.heartbeat_timer: Optional[TimerManager] = None
        self.training_timer: Optional[TimerManager] = None
        self.inference_manager: Optional[InferenceBase] = None
        self.training_manager: Optional[TrainingManager] = None

    def _init_ros_service(self):
        self.get_logger().info('Initializing ROS services...')
        service_definitions = [
            ('/task/command', SendCommand, self.user_interaction_callback),
            ('/get_robot_types', GetRobotTypeList, self.get_robot_types_callback),
            ('/set_robot_type', SetRobotType, self.set_robot_type_callback),
            ('/register_hf_user', SetHFUser, self.set_hf_user_callback),
            ('/get_registered_hf_user', GetHFUser, self.get_hf_user_callback),
            ('/get_policy_list', GetPolicyList, self.get_policy_list_callback),
            ('/training/command', SendTrainingCommand, self.user_training_interaction_callback),
            ('/training/get_available_policy', GetPolicyList, self.get_available_list_callback),
            ('/training/get_user_list', GetUserList, self.get_user_list_callback),
            ('/training/get_dataset_list', GetDatasetList, self.get_dataset_list_callback),
            (
                '/training/get_model_weight_list',
                GetModelWeightList,
                self.get_model_weight_list_callback
            ),
        ]

        for service_name, service_type, callback in service_definitions:
            self.create_service(service_type, service_name, callback)

        self.get_logger().info('ROS services initialized successfully')

    def _setup_timer_callbacks(self):
        self.timer_callback_dict = {
            'collection': self._data_collection_timer_callback,
            'async_inference': self._async_inference_timer_callback,
            'sync_inference': self._sync_inference_timer_callback,
        }

    def init_ros_params(self, robot_type):
        self.get_logger().info(f'Initializing ROS parameters for robot type: {robot_type}')
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

        self.inference_manager = InferenceFactory.create_inference_manager(
            'lerobot', device='cuda')

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
        self.communicator.clear_latest_data()
        self.timer_manager = TimerManager(node=self)

        if self.operation_mode == 'inference':
            if task_info.use_async_inference_mode:
                # Start Async inference process (non-blocking)
                if not self._start_inference_process(task_info.policy_path):
                    self.get_logger().error('Failed to start inference process')
                    return False

                # Initialize inference state
                self._used_action_count = 0
                self.remaining_actions = []
                self.inference_pending = False

                self.timer_manager.set_timer(
                    timer_name='async_inference',
                    timer_frequency=task_info.fps,
                    callback_function=self.timer_callback_dict['async_inference']
                )
                self.timer_manager.start(timer_name='async_inference')
            else:
                self.inference_manager.validate_policy(
                    policy_path=task_info.policy_path)
                # Start synchronous inference timer
                self.timer_manager.set_timer(
                    timer_name='sync_inference',
                    timer_frequency=task_info.fps,
                    callback_function=self.timer_callback_dict['sync_inference']
                )
                self.timer_manager.start(timer_name='sync_inference')
        else:
            # For collection mode, use single timer
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

        # Stop inference worker if running
        if self.inference_worker:
            self.inference_worker.stop()
            self.inference_worker = None

        # Clear visualization data
        if self.visualizer:
            self.visualizer.clear_data()

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

    def _data_collection_timer_callback(self):
        error_msg = ''
        current_status = TaskStatus()
        camera_msgs, follower_msgs, leader_msgs = self.communicator.get_latest_data()

        if camera_msgs is None:
            if time.perf_counter() - self.start_recording_time > self.DEFAULT_TOPIC_TIMEOUT:
                error_msg = 'Camera data not received within timeout period'
                self.get_logger().error(error_msg)
            else:
                self.get_logger().info('Waiting for camera data...')
                return

        elif follower_msgs is None:
            if time.perf_counter() - self.start_recording_time > self.DEFAULT_TOPIC_TIMEOUT:
                error_msg = 'Follower data not received within timeout period'
                self.get_logger().error(error_msg)
            else:
                self.get_logger().info('Waiting for follower data...')
                return

        elif leader_msgs is None:
            if time.perf_counter() - self.start_recording_time > self.DEFAULT_TOPIC_TIMEOUT:
                error_msg = 'Leader data not received within timeout period'
                self.get_logger().error(error_msg)
            else:
                self.get_logger().info('Waiting for leader data...')
                return

        try:
            camera_data, follower_data, leader_data = self.data_manager.convert_msgs_to_raw_datas(
                camera_msgs,
                follower_msgs,
                self.total_joint_order,
                leader_msgs,
                self.joint_order)

        except Exception as e:
            error_msg = f'Failed to convert messages: {str(e)}, please check the robot type again!'
            self.on_recording = False
            current_status.phase = TaskStatus.READY
            current_status.error = error_msg
            self.communicator.publish_status(status=current_status)
            self.timer_manager.stop(timer_name=self.operation_mode)
            return

        if not self.data_manager.check_lerobot_dataset(
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

        if self.communicator.joystick_state['updated']:
            self.handle_joystick_trigger(
                joystick_mode=self.communicator.joystick_state['mode'])
            self.communicator.joystick_state['updated'] = False

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

    def _sync_inference_timer_callback(self):
        error_msg = ''
        current_status = TaskStatus()
        if not self.on_inference:
            self.get_logger().info('Inference mode is not active')
            current_status = self.data_manager.get_current_record_status()
            current_status.phase = TaskStatus.READY
            self.communicator.publish_status(status=current_status)
            self.inference_manager.clear_policy()
            self.timer_manager.stop(timer_name='sync_inference')
            return

        camera_msgs, follower_msgs, _ = self.communicator.get_latest_data()
        if (camera_msgs is None or
                len(camera_msgs) != len(self.params['camera_topic_list'])):
            self.get_logger().info('Waiting for camera data...')
            return
        elif follower_msgs is None:
            self.get_logger().info('Waiting for follower data...')
            return

        try:
            camera_data, follower_data, _ = self.data_manager.convert_msgs_to_raw_datas(
                camera_msgs,
                follower_msgs,
                self.total_joint_order)
        except Exception as e:
            error_msg = f'Failed to convert messages: {str(e)}, please check the robot type again!'
            self.on_inference = False
            current_status.phase = TaskStatus.READY
            current_status.error = error_msg
            self.communicator.publish_status(status=current_status)
            self.inference_manager.clear_policy()
            self.timer_manager.stop(timer_name='sync_inference')
            return

        if self.inference_manager.policy is None:
            if not self.inference_manager.load_policy():
                self.get_logger().error('Failed to load policy')
                return

        try:
            action = self.inference_manager.predict(
                images=camera_data,
                state=follower_data,
                task_instruction=self.task_instruction[0]
            )

            self.get_logger().info(
                f'Action data: {action}')
            action_pub_msgs = self.data_manager.data_converter.tensor_array2joint_msgs(
                action,
                self.joint_topic_types,
                self.joint_order
            )

            self.communicator.publish_action(
                joint_msg_datas=action_pub_msgs
            )
            current_status = self.data_manager.get_current_record_status()
            current_status.phase = TaskStatus.INFERENCING
            self.communicator.publish_status(status=current_status)

        except Exception as e:
            self.get_logger().error(f'Inference failed, please check : {str(e)}')
            error_msg = f'Inference failed, please check : {str(e)}'
            self.on_recording = False
            self.on_inference = False
            current_status = self.data_manager.get_current_record_status()
            current_status.phase = TaskStatus.READY
            current_status.error = error_msg
            self.communicator.publish_status(status=current_status)
            self.inference_manager.clear_policy()
            self.timer_manager.stop(timer_name='sync_inference')
            return


    def _async_inference_timer_callback(self):
        error_msg = ''
        current_status = TaskStatus()

        try:
            if not self.on_inference:
                current_status = self.data_manager.get_current_record_status()
                current_status.phase = TaskStatus.READY
                self.communicator.publish_status(status=current_status)
                self._stop_inference_process()
                self.timer_manager.stop(timer_name='async_inference')
                return

            # Check inference worker initialization status
            if self.inference_worker and self.inference_worker.is_initializing():
                if not self._check_inference_worker_initialization():
                    return

            # Only proceed if worker is ready
            if not self.inference_worker or not self.inference_worker.is_ready():
                return

            remaining_count = len(self.remaining_actions)
            if remaining_count > 0:
                action = self.remaining_actions.pop(0)
                action_pub_msgs = self.data_manager.data_converter.tensor_array2joint_msgs(
                    action,
                    self.joint_topic_types,
                    self.joint_order
                )
                self.communicator.publish_action(
                    joint_msg_datas=action_pub_msgs
                )
                self._used_action_count += 1
                self.last_executed_action = action.copy()
                if self.visualizer.is_enabled():
                    self.visualizer.add_action_data(
                        action_number=self._used_action_count + 1,
                        timestamp=time.time(),
                        action_values=action,
                        remaining_in_buffer=remaining_count
                    )

            if self._process_inference_results():
                self.remaining_actions.clear()
                self.remaining_actions.extend(new_chunk)

            if (
                remaining_count <= self.inference_threshold and
                not self.inference_pending
            ):
                self.inference_start_action_count = self._used_action_count
                camera_msgs, follower_msgs, _ = self.communicator.get_latest_data()

                # Validate data availability
                if (camera_msgs is None or
                        len(camera_msgs) != len(self.params['camera_topic_list'])):
                    self.get_logger().info('Waiting for camera data')
                    return

                elif follower_msgs is None:
                    self.get_logger().info('Waiting for follower data')
                    return

                try:
                    camera_data, follower_data, _ = self.data_manager.convert_msgs_to_raw_datas(
                        camera_msgs,
                        follower_msgs,
                        self.total_joint_order)

                    # Prepare inference data
                    inference_data = (
                        camera_data,
                        follower_data,
                        self.task_instruction[0],
                        self.inference_start_action_count
                    )

                    try:
                        if self.inference_worker.send_request(inference_data):
                            self.inference_pending = True
                        else:
                            self.get_logger().error('Failed to send inference request')
                    except Exception as put_error:
                        self.get_logger().error(f'Exception during send - {str(put_error)}')
                        raise put_error

                except Exception as e:
                    error_msg = f'FAILED: {str(e)}'
                    self.get_logger().error(error_msg)
                    self._stop_inference_with_error(error_msg)
                    return

        except Exception as e:
            self.get_logger().error(f'Inference timer callback error: {str(e)}')
            self._stop_inference_with_error(f'Inference timer error: {str(e)}')

    def _process_inference_results(self):
        try:
            try:
                result = self.inference_worker.get_result(
                    block=False)
                if not result:
                    return []

                status, result_data = result

                if status == 'success':
                    action_chunk = result_data['actions']
                    inference_time = result_data['inference_time']
                    worker_start_count = result_data.get('inference_start_action_count', 0)

                    self.get_logger().info(
                        f'Inference completed in {inference_time*1000:.1f}ms')

                    if not action_chunk:
                        self.get_logger().warning(
                            'Received empty action chunk!')
                        return

                    # Calculate offset and apply smoothing
                    actions_executed_during_inference = max(
                        0, self._used_action_count - worker_start_count)

                    # Apply offset and smoothing
                    if self.inference_threshold > 0:
                        new_chunk = self.action_chunk_processor.apply_offset_and_smoothing(
                            action_chunk,
                            actions_executed_during_inference,
                            self.visualizer.get_action_history(),
                            self.last_executed_action)
                    else:
                        new_chunk = action_chunk

                    self.inference_pending = False

                    # Process all visualization data if enabled
                    if self.visualizer.is_enabled():
                        self.visualizer.process_complete_inference_visualization(
                            action_chunk, inference_time, worker_start_count,
                            new_chunk, self._used_action_count,
                            actions_executed_during_inference,
                            self.get_logger()
                        )
                    return new_chunk

                elif status == 'error':
                    self.inference_pending = False
                    self.get_logger().error(
                        f'Received error from inference worker: {result_data}')
                    self._stop_inference_with_error(
                        f'Inference process error: {result_data}')
                    return []

                elif status == 'pong':
                    self.get_logger().debug('Received health check pong')
                    return []

            except Exception as inner_e:
                self.get_logger().error(
                    f'Error processing single result: {str(inner_e)}')
                return []

        except Exception as e:
            self.get_logger().error(
                f'Error processing inference results: {str(e)}')
            return []

    def user_training_interaction_callback(self, request, response):
        try:
            if request.command == SendTrainingCommand.Request.START:
                self.training_manager = TrainingManager()
                self.training_timer = TimerManager(node=self)
                self.training_timer.set_timer(
                    timer_name='training_status',
                    timer_frequency=self.TRAINING_STATUS_TIMER_FREQUENCY,
                    callback_function=lambda: self.communicator.publish_training_status(
                        self.get_training_status()
                    )
                )
                self.training_timer.start(timer_name='training_status')

                if self.training_thread and self.training_thread.is_alive():
                    response.success = False
                    response.message = 'Training is already in progress'
                    return response

                output_folder_name = request.training_info.output_folder_name
                weight_save_root_path = TrainingManager.get_weight_save_root_path()
                self.get_logger().info(
                    f'Weight save root path: {weight_save_root_path}, '
                    f'Output folder name: {output_folder_name}'
                )
                output_path = weight_save_root_path / output_folder_name
                if output_path.exists():
                    response.success = False
                    response.message = f'Output folder already exists: {output_path}'
                    self.is_training = False
                    self.communicator.publish_training_status(
                        self.get_training_status()
                    )

                    self.training_manager.stop_event.set()
                    self.training_timer.stop('training_status')
                    return response

                self.training_manager.training_info = request.training_info

                def run_training():
                    try:
                        self.training_manager.train()
                    finally:
                        self.is_training = False
                        self.get_logger().info('Training completed.')
                        self.communicator.publish_training_status(
                            self.get_training_status()
                        )
                        self.training_manager.stop_event.set()
                        self.training_timer.stop('training_status')

                self.training_thread = threading.Thread(target=run_training, daemon=True)
                self.training_thread.start()
                self.is_training = True

                response.success = True
                response.message = 'Training started successfully'

            else:
                if request.command == SendTrainingCommand.Request.FINISH:
                    self.is_training = False
                    self.communicator.publish_training_status(
                        self.get_training_status()
                    )
                    self.training_timer.stop('training_status')
                    if self.training_thread and self.training_thread.is_alive():
                        self.training_manager.stop_event.set()
                        self.training_thread.join()
                        response.success = True
                        response.message = 'Training stopped successfully'
                    else:
                        response.success = False
                        response.message = 'No training in progress to stop'
                # TODO: Uncomment when resume is implemented
                # elif request.command == SendTrainingCommand.Request.RESUME:
                #     pass

        except Exception as e:
            self.get_logger().error(f'Error in user_training_interaction: {str(e)}')
            response.success = False
            response.message = f'Error in user_training_interaction: {str(e)}'
            return response
        return response

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
                self.joint_topic_types = self.communicator.get_publisher_msg_types()
                self.operation_mode = 'inference'
                self.on_inference = True
                task_info = request.task_info
                self.task_info = task_info  # Store for process restart
                self.task_instruction = task_info.task_instruction
                self.async_inference_mode = task_info.use_async_inference_mode
                self.inference_threshold = task_info.update_async_inference_step

                self.get_logger().info(
                    f'Starting inference with policy: {task_info.policy_path}, '
                    f'Async mode: {self.async_inference_mode}, '
                    f'Threshold: {self.inference_threshold}')

                if not self.init_robot_control_parameters_from_user_task(
                    task_info
                ):
                    response.success = False
                    response.message = 'Failed to initialize robot control parameters'
                    return response

                if task_info.record_inference_mode:
                    self.on_recording = True
                self.start_recording_time = time.perf_counter()

                response.success = True
                response.message = 'Inference initialization started'

            else:
                if not self.on_recording and not self.on_inference:
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
                        if len(request.task_info.task_instruction) > 1:
                            self.data_manager.record_next_episode()
                        else:
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
                        self.on_inference = False

                        response.success = True
                        response.message = 'All operations terminated'

                    elif request.command == SendCommand.Request.SKIP_TASK:
                        self.get_logger().info('Skipping task')
                        self.data_manager.record_skip_task()
                        response.success = True
                        response.message = 'Task skipped successfully'

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
        policy_list = InferenceFactory.get_all_available_policies()
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
        response.policy_list, response.device_list = TrainingManager.get_available_list()
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
                response.message = f'User ID {user_id} does not exist at path: {user_path}'
                return response

            dataset_names = [
                name for name in os.listdir(user_path)
                if (user_path / name).is_dir()
            ]

            response.dataset_list = dataset_names
            response.success = True
            response.message = f'Found {len(dataset_names)} dataset(s) for user {user_id}.'

        except Exception as e:
            response.dataset_list = []
            response.success = False
            response.message = f'Error: {str(e)}'

        return response

    def get_model_weight_list_callback(self, request, response):
        save_root_path = TrainingManager.get_weight_save_root_path()
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

    def handle_joystick_trigger(self, joystick_mode: str):
        self.get_logger().info(
            f'Joystick mode updated: {joystick_mode}')
        if self.data_manager is None:
            self.get_logger().warning(
                'Data manager is not initialized')
            return

        if not self.on_recording:
            self.get_logger().warning(
                'Not currently recording')
            return

        if joystick_mode == 'right':
            self.get_logger().info(
                'Right tact triggered - Moving to next episode')
            if len(self.data_manager.get_task_info().task_instruction) > 1:
                self.data_manager.record_next_episode()
            else:
                self.data_manager.record_early_save()
        elif joystick_mode == 'left':
            self.get_logger().info(
                'Left tact triggered - Re-record current episode')
            self.data_manager.re_record()
        elif joystick_mode == 'right_long_time':
            self.get_logger().info(
                'Right long tact triggered - Custom')
            # If you want, you can add custom functionality.
        elif joystick_mode == 'left_long_time':
            self.get_logger().info(
                'Left long tact triggered - Custom')
            # If you want, you can add custom functionality.
        else:
            self.get_logger().info(
                f'Received joystick trigger: {joystick_mode}')

    def _start_inference_process(self, policy_path, device='cuda'):
        try:
            # Create and start inference worker
            self.inference_worker = InferenceWorker(policy_path, device)
            return self.inference_worker.start()

        except Exception as e:
            self.get_logger().error(f'Failed to start inference worker: {str(e)}')
            return False

    def _stop_inference_process(self):
        try:
            if self.inference_worker:
                self.inference_worker.stop()
                self.inference_worker = None
                self.get_logger().info('Inference worker stopped')

        except Exception as e:
            self.get_logger().error(f'Error stopping inference worker: {str(e)}')

    def _check_inference_worker_initialization(self):
        if not self.inference_worker:
            return False

        status, ready = self.inference_worker.check_initialization_status()

        if status == 'ready':
            return True
        elif status == 'error' or status == 'timeout':
            error_msg = f'Inference worker initialization {status}'
            if status == 'timeout':
                error_msg += f' after {self.inference_worker.initialization_timeout}s'
            self._stop_inference_with_error(error_msg)
            return False
        elif status == 'loading':
            return False

        return False

    def _stop_inference_with_error(self, error_msg):
        self.get_logger().error(f'Stopping inference due to error: {error_msg}')
        self.on_inference = False
        self.inference_pending = False

        # Stop inference worker
        if self.inference_worker:
            self._stop_inference_process()

        # Update status to show error
        if self.data_manager and self.communicator:
            try:
                current_status = self.data_manager.get_current_record_status()
                current_status.phase = TaskStatus.READY
                current_status.error = error_msg
                self.communicator.publish_status(status=current_status)
            except Exception as e:
                self.get_logger().error(
                    f'Failed to publish error status: {str(e)}')


def main(args=None):
    multiprocessing.set_start_method('spawn', force=True)

    rclpy.init(args=args)
    node = PhysicalAIServer()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'inference_worker') and node.inference_worker:
            node._stop_inference_process()

        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
