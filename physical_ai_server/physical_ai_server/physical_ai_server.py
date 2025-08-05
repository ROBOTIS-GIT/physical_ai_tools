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
import os
from pathlib import Path
import multiprocessing
import queue
import threading
import time
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from physical_ai_interfaces.msg import TaskStatus, TrainingStatus
import numpy as np
from physical_ai_interfaces.msg import TaskStatus
from physical_ai_interfaces.srv import (
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

from physical_ai_server.communication.communicator import Communicator
from physical_ai_server.data_processing.data_manager import DataManager
from physical_ai_server.inference import InferenceFactory
from physical_ai_server.inference.inference_multi_process import InferenceWorker
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
        self.inference_lock = multiprocessing.Lock()
        self.inference_pending = False
        
        # Track inference timing for proper action offset
        self.inference_start_action_count = 0  # Actions used when inference started
        self.last_executed_action = None  # Remember last executed action for smoothing
        self.updating_action_chunk = False

    def _init_core_components(self):
        self.communicator: Optional[Communicator] = None
        self.data_manager: Optional[DataManager] = None
        self.timer_manager: Optional[TimerManager] = None
        self.heartbeat_timer: Optional[TimerManager] = None
        self.training_timer: Optional[TimerManager] = None
        self.inference_manager: Optional[InferenceManager] = None
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
        ]

        for service_name, service_type, callback in service_definitions:
            self.create_service(service_type, service_name, callback)

        self.get_logger().info('ROS services initialized successfully')

    def _setup_timer_callbacks(self):
        self.timer_callback_dict = {
            'collection': self._data_collection_timer_callback,
            'action': self._action_timer_callback,
            'inference': self._inference_timer_callback,
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

        self.inference_manager = InferenceFactory.create_inference_manager('lerobot', device='cuda')
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
            msg.training_info = training_info
            msg.current_step = current_step
            msg.is_training = self.is_training
            msg.error = ''
        except Exception as e:
            msg.current_step = 0
            msg.error = str(e)
            self.get_logger().error(f'Error publishing training status: {msg.error}')
            return
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
            # Set up two timers for inference mode
            # 1. Action timer - publishes actions at high frequency (33ms)
            self.timer_manager.set_timer(
                timer_name='action',
                timer_frequency=10,
                callback_function=self.timer_callback_dict['action']
            )
            
            # 2. Inference timer - runs at high frequency to process results quickly
            inference_frequency = 50.0  # 50Hz = 20ms interval for responsive result processing
            self.timer_manager.set_timer(
                timer_name='inference',
                timer_frequency=inference_frequency,
                callback_function=self.timer_callback_dict['inference']
            )
            
            self.timer_manager.start(timer_name='action')
            self.timer_manager.start(timer_name='inference')
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

    def _inference_timer_callback(self):
        if not self.on_inference:
            return
            
        try:
            self.get_logger().debug("Inference timer callback triggered")
            
            # Process any pending inference results first
            self._process_inference_results()
            
            # Check current state
            remaining_count = len(self.remaining_actions)
            
            if (not self.inference_pending and
                self.inference_worker and 
                self.inference_worker.is_alive()):
                
                # Record the CURRENT action count when inference starts
                # Use a lock to ensure consistency with action timer
                with self.inference_lock:
                    self.inference_start_action_count = self._used_action_count
                
                self.get_logger().info(f"Starting new inference at action count: {self.inference_start_action_count}")
                
                # Get current data for fresh inference
                camera_msgs, follower_msgs, _ = self.communicator.get_latest_data()
                
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

                    inference_data = (camera_data, follower_data, self.task_instruction[0], self.inference_start_action_count)
                    try:
                        if self.inference_worker.send_request(inference_data):
                            self.inference_pending = True
                        else:
                            self.get_logger().error('Failed to send inference request')
                    except Exception as put_error:
                        self.get_logger().error(f'Exception during send - {str(put_error)}')
                        raise put_error

                except Exception as e:
                    error_msg = f'Step 4-8 FAILED: {str(e)}'
                    self.get_logger().error(error_msg)
                    import traceback
                    self.get_logger().error(f'Traceback: {traceback.format_exc()}')
                    self._stop_inference_with_error(error_msg)
                    return
            else:
                if self.inference_pending:
                    self.get_logger().debug(f"Inference already pending, continuing with {remaining_count} actions")
                elif not self.inference_worker or not self.inference_worker.is_alive():
                    self.get_logger().warning("Inference worker is not alive")
                    
        except Exception as e:
            self.get_logger().error(f'Inference timer callback error: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            self._stop_inference_with_error(f'Inference timer error: {str(e)}')

    def _process_inference_results(self):
        """Process inference results from the worker process - one result per call"""
        try:
            # Process only one result per timer callback to avoid blocking
            try:
                result = self.inference_worker.get_result(block=False)
                if not result:
                    return  # No results available
                    
                status, result_data = result
                
                if status == 'success':
                    self.updating_action_chunk = True
                    action_chunk = result_data['actions']
                    inference_time = result_data['inference_time']
                    inference_start_count = result_data.get('inference_start_action_count', 0)
                    
                    self.get_logger().info(
                        f'Inference completed in {inference_time*1000:.1f}ms, '
                        f'generated {len(action_chunk)} actions')

                    # Calculate offset based on actions executed DURING inference
                    # Use the start count that was actually sent to the worker
                    worker_start_count = result_data.get('inference_start_action_count', inference_start_count)
                    actions_executed_during_inference = max(0, self._used_action_count - worker_start_count + 3)
                    
                    self.get_logger().info(
                        f'Worker reported start at action: {worker_start_count}, '
                        f'current action count: {self._used_action_count}, '
                        f'actions executed during inference: {actions_executed_during_inference}')
                    
                    # Apply offset first
                    if actions_executed_during_inference > 0:
                        if actions_executed_during_inference < len(action_chunk):
                            offset_action_chunk = action_chunk[actions_executed_during_inference:]
                            self.get_logger().info(
                                f'Applied offset: skipped first {actions_executed_during_inference} actions, '
                                f'using {len(offset_action_chunk)} remaining actions')
                        else:
                            self.get_logger().warning(
                                f'All {len(action_chunk)} actions were already executed during inference!')
                            offset_action_chunk = []
                    else:
                        offset_action_chunk = action_chunk
                        self.get_logger().info(
                            f'No actions executed during inference, using all {len(action_chunk)} actions')

                    # Don't reset action count here - let it continue incrementing

                    # Apply smoothing using LAST EXECUTED ACTION (not remaining actions)
                    if self.last_executed_action is not None and offset_action_chunk:
                        # Create multiple transition steps for smoother blending
                        num_transition_steps = min(5, len(offset_action_chunk))  # 5단계 전환
                        
                        for step in range(num_transition_steps):
                            # 점진적으로 가중치를 변경: 0.8, 0.6, 0.4, 0.2, 0.0
                            old_weight = 0.8 - (step * 0.2)  
                            new_weight = 1.0 - old_weight
                            
                            transition_action = []
                            for i in range(len(self.last_executed_action)):
                                blended = (old_weight * self.last_executed_action[i] + 
                                         new_weight * offset_action_chunk[step][i])
                                transition_action.append(blended)
                            
                            # Replace with smoothed version
                            offset_action_chunk[step] = transition_action
                        
                        self.get_logger().info(
                            f'Applied multi-step smoothing over {num_transition_steps} actions:')
                        self.get_logger().info(
                            f'  last_executed={self.last_executed_action[:]}')
                        self.get_logger().info(
                            f'  first_original={action_chunk[actions_executed_during_inference][:]}')
                        self.get_logger().info(
                            f'  first_smoothed={offset_action_chunk[0][:]}')
                        self.get_logger().info(
                            f'  final_smoothed={offset_action_chunk[num_transition_steps-1][:]}')
                    else:
                        if self.last_executed_action is None:
                            self.get_logger().info('No last executed action available for smoothing')
                        if not offset_action_chunk:
                            self.get_logger().info('No offset actions to smooth')

                    # Replace actions
                    with self.inference_lock:
                        old_count = len(self.remaining_actions)
                        self.remaining_actions.clear()
                        self.remaining_actions.extend(offset_action_chunk)
                        new_count = len(self.remaining_actions)
                        self.inference_pending = False
                        self.updating_action_chunk = False
                    
                    self.get_logger().info(
                        f'Action buffer REPLACED: {old_count} -> {new_count} fresh actions '
                        f'(offset applied: {actions_executed_during_inference})')

                    # Update status
                    current_status = self.data_manager.get_current_record_status()
                    current_status.phase = TaskStatus.INFERENCING
                    self.communicator.publish_status(status=current_status)
                    
                elif status == 'error':
                    self.inference_pending = False
                    self.get_logger().error(f'Received error from inference worker: {result_data}')
                    self._stop_inference_with_error(f'Inference process error: {result_data}')
                    return
                    
                elif status == 'pong':
                    self.get_logger().debug("Received health check pong")
                    return
                    
            except Exception as inner_e:
                self.get_logger().error(f'Error processing single result: {str(inner_e)}')
                    
        except Exception as e:
            self.get_logger().error(f'Error processing inference results: {str(e)}')

    def _action_timer_callback(self):
        """Action timer callback - publishes actions at high frequency"""
        if not self.on_inference:
            return

        if self.updating_action_chunk:
            self.get_logger().debug("Waiting for action chunk update to complete")
            return

        try:
            # Publish next action if available (thread-safe)
            with self.inference_lock:
                if len(self.remaining_actions) > 0:
                    action = self.remaining_actions.pop(0)
                    action_available = True
                    remaining_count = len(self.remaining_actions)
                else:
                    action_available = False
                    remaining_count = 0
            
            if action_available:
                # IMPORTANT: Remember this action for smoothing next time
                self.last_executed_action = action.copy() if isinstance(action, list) else list(action)
                
                self.get_logger().info(
                    f'Publishing action {self._used_action_count + 1} '
                    f'(remaining: {remaining_count}): {action[:3]}... '
                    f'[Current total actions executed: {self._used_action_count + 1}]')
                
                action_pub_msgs = self.data_manager.data_converter.tensor_array2joint_msgs(
                    action,
                    self.joint_topic_types,
                    self.joint_order
                )

                self.communicator.publish_action(
                    joint_msg_datas=action_pub_msgs
                )
                self._used_action_count += 1
                
                # Log when we're running very low
                if remaining_count <= 2:
                    self.get_logger().info(f"Very low on actions ({remaining_count}), inference should complete soon")
                    
            else:
                # Only warn occasionally to avoid spam
                if self._used_action_count % 10 == 0:  # Every 10th time
                    self.get_logger().warning('No actions available! Robot will wait for fresh inference...')

        except Exception as e:
            self.get_logger().error(f'Action publishing failed: {str(e)}')

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
                task_info = request.task_info
                self.task_info = task_info  # Store for process restart
                self.task_instruction = task_info.task_instruction

                # Start inference process
                if not self._start_inference_process(task_info.policy_path):
                    response.success = False
                    response.message = 'Failed to start inference process'
                    self.get_logger().error(response.message)
                    return response

                # Initialize inference state
                self._used_action_count = 0
                with self.inference_lock:
                    self.remaining_actions = []
                self.inference_pending = False

                self.init_robot_control_parameters_from_user_task(
                    task_info
                )
                if task_info.record_inference_mode:
                    self.on_recording = True
                self.on_inference = True
                self.start_recording_time = time.perf_counter()

                response.success = True
                response.message = 'Inference started'

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
                        
                        # Stop inference worker if running
                        if self.inference_worker:
                            self._stop_inference_process()
                            
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
        policy_list = InferenceFactory.get_available_frameworks()
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

    def get_saved_policies_callback(self, request, response):
        # For now, return empty list since we're using the new factory system
        # You can implement saved policy discovery later
        self.get_logger().warning('Saved policies not implemented in new system yet')
        response.saved_policy_path = []
        response.saved_policy_type = []
        response.success = False
        response.message = 'Saved policies not implemented in new system yet'
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

    def _start_inference_process(self, policy_path, device='cuda'):
        """Start the inference worker"""
        try:
            # Create inference worker
            self.inference_worker = InferenceWorker(policy_path, device)
            
            # Start the worker process
            if not self.inference_worker.start():
                self.get_logger().error('Failed to start inference worker process')
                return False
            
            # Wait for worker to be ready
            timeout = 30.0  # 30 seconds timeout
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                try:
                    result = self.inference_worker.get_result(block=False, timeout=1.0)
                    if result:
                        status, message = result
                        if status == 'ready':
                            self.get_logger().info('Inference worker started successfully')
                            return True
                        elif status == 'error':
                            self.get_logger().error(f'Inference worker failed to start: {message}')
                            return False
                except:
                    continue
                    
            self.get_logger().error('Inference worker startup timeout')
            return False
            
        except Exception as e:
            self.get_logger().error(f'Failed to start inference worker: {str(e)}')
            return False

    def _stop_inference_process(self):
        """Stop the inference worker"""
        try:
            if self.inference_worker:
                self.inference_worker.stop()
                self.inference_worker = None
                self.get_logger().info('Inference worker stopped')
            
        except Exception as e:
            self.get_logger().error(f'Error stopping inference worker: {str(e)}')

    def _check_inference_process_health(self):
        """Check if the inference worker is still alive and responsive"""
        if not self.inference_worker or not self.inference_worker.is_alive():
            self.get_logger().warning("Inference worker is not alive")
            return False
            
        # Skip ping/pong for now to avoid interference - just check if process is alive
        return True

    def _stop_inference_with_error(self, error_msg):
        """Stop inference due to error"""
        self.get_logger().error(f"Stopping inference due to error: {error_msg}")
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
                self.get_logger().error(f"Failed to publish error status: {str(e)}")


def main(args=None):
    # Set multiprocessing start method to 'spawn' for better compatibility
    multiprocessing.set_start_method('spawn', force=True)
    
    rclpy.init(args=args)
    node = PhysicalAIServer()
    
    # 멀티스레드 실행기 사용
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up inference worker
        if hasattr(node, 'inference_worker') and node.inference_worker:
            node._stop_inference_process()
            
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()