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


def inference_worker_process(input_queue, output_queue, policy_path, device='cuda'):
    """
    Separate inference process that runs independently to avoid GIL blocking
    """
    import logging
    
    # Set up logging for the worker process
    logging.basicConfig(level=logging.INFO, 
                       format='[INFERENCE_WORKER] %(levelname)s: %(message)s')
    logger = logging.getLogger('inference_worker')
    
    try:
        logger.info(f"Worker process started with PID: {os.getpid()}")
        logger.info(f"Policy path: {policy_path}")
        logger.info(f"Device: {device}")
        
        # Import here to avoid issues with multiprocessing
        from physical_ai_server.inference import InferenceFactory
        logger.info("InferenceFactory imported successfully")
        
        # Initialize inference manager in separate process
        logger.info("Creating inference manager...")
        inference_manager = InferenceFactory.create_inference_manager('lerobot', device=device)
        logger.info("Inference manager created successfully")
        
        # Validate and load policy
        logger.info(f"Validating policy: {policy_path}")
        valid_result, result_message = inference_manager.validate_policy(policy_path=policy_path)
        if not valid_result:
            error_msg = f'Policy validation failed: {result_message}'
            logger.error(error_msg)
            output_queue.put(('error', error_msg))
            return
        logger.info("Policy validation successful")
            
        logger.info("Loading policy...")
        if not inference_manager.load_policy():
            error_msg = 'Failed to load policy'
            logger.error(error_msg)
            output_queue.put(('error', error_msg))
            return
        logger.info("Policy loaded successfully")
            
        output_queue.put(('ready', 'Inference worker ready'))
        logger.info("Worker is ready and waiting for requests")
        
        request_count = 0
        last_log_time = time.time()
        
        while True:
            try:
                # Log periodic status
                current_time = time.time()
                if current_time - last_log_time > 10.0:  # Every 10 seconds
                    logger.info(f"Worker still alive, processed {request_count} requests so far")
                    logger.info(f"Input queue size: {input_queue.qsize()}")
                    last_log_time = current_time
                
                # Check for new inference requests
                try:
                    logger.debug("Checking for inference request...")
                    data = input_queue.get(timeout=1.0)  # Increased timeout
                    
                    if data is None:  # Shutdown signal
                        logger.info("Received shutdown signal")
                        break
                    elif data == 'ping':  # Health check
                        logger.debug("Received health check ping")
                        output_queue.put(('pong', 'Worker alive'))
                        continue
                        
                    request_count += 1
                    logger.info(f"*** Received inference request #{request_count} ***")
                    
                    # Unpack data with inference start action count
                    if len(data) == 4:
                        camera_data, follower_data, task_instruction, inference_start_action_count = data
                    else:
                        # Backward compatibility
                        camera_data, follower_data, task_instruction = data
                        inference_start_action_count = 0
                    
                    # Safe logging for different data types
                    camera_info = ""
                    follower_info = ""
                    
                    if isinstance(camera_data, dict):
                        camera_info = f"Dict with {len(camera_data)} keys: {list(camera_data.keys())}"
                    elif hasattr(camera_data, 'shape'):
                        camera_info = f"Array shape: {camera_data.shape}"
                    else:
                        camera_info = f"Type: {type(camera_data)}"
                    
                    if isinstance(follower_data, dict):
                        follower_info = f"Dict with {len(follower_data)} keys: {list(follower_data.keys())}"
                    elif hasattr(follower_data, 'shape'):
                        follower_info = f"Array shape: {follower_data.shape}"
                    else:
                        follower_info = f"Type: {type(follower_data)}"
                    
                    logger.info(f"Data received - Camera: {camera_info}, "
                              f"Follower: {follower_info}, Task: {task_instruction}, "
                              f"Inference start action count: {inference_start_action_count}")
                    
                    # Run inference
                    logger.info("Starting inference...")
                    start_time = time.time()
                    action_chunk = inference_manager.predict_chunk(
                        images=camera_data,
                        state=follower_data,
                        task_instruction=task_instruction
                    )
                    inference_time = time.time() - start_time
                    logger.info(f"Inference completed in {inference_time*1000:.1f}ms")
                    
                    # Convert to list if it's a numpy array
                    if isinstance(action_chunk, np.ndarray):
                        action_chunk = action_chunk.tolist()
                        logger.info(f"Converted numpy array to list, length: {len(action_chunk)}")
                    
                    # Send result back with inference start action count for offset calculation
                    result = {
                        'actions': action_chunk,
                        'inference_time': inference_time,
                        'inference_start_action_count': inference_start_action_count
                    }
                    output_queue.put(('success', result))
                    logger.info(f"*** Result sent back with {len(action_chunk)} actions "
                              f"(inference started at action {inference_start_action_count}) ***")
                    
                except queue.Empty:
                    # Timeout - continue waiting (this is normal)
                    continue
                    
            except Exception as e:
                error_msg = f'Inference error: {str(e)}'
                logger.error(error_msg)
                import traceback
                logger.error(f'Traceback: {traceback.format_exc()}')
                output_queue.put(('error', error_msg))
                
    except Exception as e:
        error_msg = f'Worker initialization error: {str(e)}'
        logger.error(error_msg)
        import traceback
        logger.error(f'Traceback: {traceback.format_exc()}')
        output_queue.put(('error', error_msg))
    
    logger.info("Worker process shutting down")


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
        self.inference_process = None
        self.inference_input_queue = None
        self.inference_output_queue = None
        self.inference_lock = multiprocessing.Lock()
        self.inference_pending = False
        
        # Track inference timing for proper action offset
        self.inference_start_action_count = 0  # Actions used when inference started
        self.last_executed_action = None  # Remember last executed action for smoothing
        
        # Action data collection for visualization
        self.action_history = []  # Store executed actions for plotting
        self.max_action_history = 300  # Number of actions to collect
        self.plot_enabled = True  # Enable/disable plotting

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
            
            # 2. Inference timer - runs inference at lower frequency (200ms)
            inference_frequency = 5.0  # 5Hz = 200ms interval
            self.timer_manager.set_timer(
                timer_name='inference',
                timer_frequency=10,
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
            
        # Stop inference process if running
        if self.inference_process:
            self._stop_inference_process()

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

    def _plot_action_graphs(self):
        """Plot action data for each joint"""
        try:
            import matplotlib.pyplot as plt
            import numpy as np
            from datetime import datetime
            
            if not self.action_history:
                self.get_logger().warning("No action history to plot")
                return
            
            # Extract data
            action_counts = [entry['action_count'] for entry in self.action_history]
            timestamps = [entry['timestamp'] for entry in self.action_history]
            actions = [entry['action'] for entry in self.action_history]
            
            # Convert to numpy array for easier manipulation
            actions_array = np.array(actions)
            num_joints = actions_array.shape[1]
            
            self.get_logger().info(f"Plotting {len(self.action_history)} actions with {num_joints} joints")
            
            # Create subplots - arrange in a grid
            if num_joints <= 4:
                rows, cols = 2, 2
            elif num_joints <= 6:
                rows, cols = 2, 3
            elif num_joints <= 9:
                rows, cols = 3, 3
            else:
                rows, cols = 4, 3
            
            fig, axes = plt.subplots(rows, cols, figsize=(15, 10))
            fig.suptitle(f'Joint Actions Over Time (Actions {action_counts[0]}-{action_counts[-1]})', 
                        fontsize=16)
            
            # Flatten axes for easier indexing
            if rows * cols > 1:
                axes_flat = axes.flatten()
            else:
                axes_flat = [axes]
            
            # Plot each joint
            for joint_idx in range(num_joints):
                ax = axes_flat[joint_idx]
                joint_values = actions_array[:, joint_idx]
                
                # Plot the joint values
                ax.plot(action_counts, joint_values, 'b-', linewidth=1.5, alpha=0.8)
                ax.set_title(f'Joint {joint_idx + 1}', fontsize=12, fontweight='bold')
                ax.set_xlabel('Action Count')
                ax.set_ylabel('Joint Value (radians)')
                ax.grid(True, alpha=0.3)
                
                # Add statistics
                mean_val = np.mean(joint_values)
                std_val = np.std(joint_values)
                min_val = np.min(joint_values)
                max_val = np.max(joint_values)
                
                # Add text box with statistics
                stats_text = f'μ={mean_val:.3f}\nσ={std_val:.3f}\nmin={min_val:.3f}\nmax={max_val:.3f}'
                ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, 
                       verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                       fontsize=8)
                
                # Highlight smoothing transitions (find large jumps)
                if len(joint_values) > 1:
                    diff = np.abs(np.diff(joint_values))
                    threshold = 3 * np.std(diff)  # 3 sigma threshold
                    large_jumps = np.where(diff > threshold)[0]
                    
                    for jump_idx in large_jumps:
                        ax.axvline(x=action_counts[jump_idx], color='red', linestyle='--', alpha=0.5)
                        ax.axvline(x=action_counts[jump_idx + 1], color='red', linestyle='--', alpha=0.5)
            
            # Hide unused subplots
            for joint_idx in range(num_joints, len(axes_flat)):
                axes_flat[joint_idx].set_visible(False)
            
            plt.tight_layout()
            
            # Save the plot
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"/tmp/action_plot_{timestamp_str}.png"
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            self.get_logger().info(f"Action plot saved to: {filename}")
            
            # Optionally show the plot (comment out if running headless)
            # plt.show()
            
            plt.close()
            
        except ImportError:
            self.get_logger().error("matplotlib not available. Install with: pip install matplotlib")
        except Exception as e:
            self.get_logger().error(f"Failed to plot action graphs: {str(e)}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")

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
        """Inference timer callback - manages communication with inference process"""
        if not self.on_inference:
            return
            
        try:
            self.get_logger().debug("Inference timer callback triggered")
            
            # Process any pending inference results first
            self._process_inference_results()
            
            # Check current state
            remaining_count = len(self.remaining_actions)
            self.get_logger().debug(f"Current state - Remaining actions: {remaining_count}, "
                                   f"Inference pending: {self.inference_pending}")
            
            # Request inference when running low on actions (but keep using current ones)
            if (remaining_count < 50 and  # Running low on actions
                not self.inference_pending and  # No pending inference request
                self.inference_process and 
                self.inference_process.is_alive()):
                
                self.get_logger().info(f"Running low on actions ({remaining_count}), requesting fresh inference...")
                
                # Record the action count when inference starts (but DON'T clear actions yet)
                self.inference_start_action_count = self._used_action_count
                
                # Get current data for fresh inference
                self.get_logger().info("Step 1: Getting latest data from communicator...")
                camera_msgs, follower_msgs, _ = self.communicator.get_latest_data()
                
                self.get_logger().info(f"Step 2: Data check - Camera msgs: {camera_msgs is not None}, "
                                       f"Follower msgs: {follower_msgs is not None}")
                
                if camera_msgs is not None:
                    self.get_logger().info(f"Camera msgs count: {len(camera_msgs)}, "
                                           f"Expected: {len(self.params['camera_topic_list'])}")
                
                if (camera_msgs is None or
                        len(camera_msgs) != len(self.params['camera_topic_list'])):
                    self.get_logger().info('Step 3: Waiting for camera data - RETURNING')
                    return
                        
                elif follower_msgs is None:
                    self.get_logger().info('Step 3: Waiting for follower data - RETURNING')
                    return

                self.get_logger().info("Step 3: Data validation passed, proceeding to conversion...")

                try:
                    self.get_logger().info("Step 4: Converting messages to raw data...")
                    start_convert_time = time.time()
                    
                    camera_data, follower_data, _ = self.data_manager.convert_msgs_to_raw_datas(
                        camera_msgs,
                        follower_msgs,
                        self.total_joint_order)
                    
                    convert_time = time.time() - start_convert_time
                    
                    # Handle different data types for logging
                    camera_info = ""
                    follower_info = ""
                    
                    if isinstance(camera_data, dict):
                        camera_info = f"Dict with keys: {list(camera_data.keys())}"
                        if camera_data:
                            first_key = list(camera_data.keys())[0]
                            first_value = camera_data[first_key]
                            if hasattr(first_value, 'shape'):
                                camera_info += f", first item shape: {first_value.shape}"
                    elif hasattr(camera_data, 'shape'):
                        camera_info = f"Array shape: {camera_data.shape}"
                    else:
                        camera_info = f"Type: {type(camera_data)}"
                    
                    if isinstance(follower_data, dict):
                        follower_info = f"Dict with keys: {list(follower_data.keys())}"
                        if follower_data:
                            first_key = list(follower_data.keys())[0]
                            first_value = follower_data[first_key]
                            if hasattr(first_value, 'shape'):
                                follower_info += f", first item shape: {first_value.shape}"
                    elif hasattr(follower_data, 'shape'):
                        follower_info = f"Array shape: {follower_data.shape}"
                    else:
                        follower_info = f"Type: {type(follower_data)}"
                    
                    self.get_logger().info(f"Step 5: Data converted in {convert_time*1000:.1f}ms - "
                                         f"Camera: {camera_info}, Follower: {follower_info}")
                    
                    # Check queue status before sending
                    queue_size = self.inference_input_queue.qsize()
                    queue_full = self.inference_input_queue.full()
                    self.get_logger().info(f"Step 6: Queue status - Size: {queue_size}/5, Full: {queue_full}")
                    
                    # Send inference request to worker process
                    if not queue_full:
                        task_instruction = (self.task_instruction[0] 
                                          if isinstance(self.task_instruction, list) 
                                          else self.task_instruction)
                        
                        self.get_logger().info(f"Step 7: Preparing inference data - Task: {task_instruction}")
                        # Include the action count when inference starts for proper offset calculation
                        inference_data = (camera_data, follower_data, task_instruction, self.inference_start_action_count)
                        
                        self.get_logger().info("Step 8: Sending inference request to worker process...")
                        send_start_time = time.time()
                        
                        try:
                            self.inference_input_queue.put(inference_data, timeout=2.0)
                            send_time = time.time() - send_start_time
                            self.inference_pending = True
                            
                            self.get_logger().info(f'Step 9: SUCCESS - Sent inference request in {send_time*1000:.1f}ms '
                                                 f'(inference started at action count: {self.inference_start_action_count})')
                            self.get_logger().info(f'Current actions will continue executing during inference')
                        except Exception as put_error:
                            self.get_logger().error(f"Step 8 FAILED: Queue put failed - {str(put_error)}")
                            raise put_error
                            
                    else:
                        self.get_logger().warning('Step 7: SKIPPED - Inference input queue is full')
                    
                except Exception as e:
                    error_msg = f'Step 4-8 FAILED: {str(e)}'
                    self.get_logger().error(error_msg)
                    import traceback
                    self.get_logger().error(f'Traceback: {traceback.format_exc()}')
                    self._stop_inference_with_error(error_msg)
                    return
            else:
                if remaining_count >= 50:
                    self.get_logger().debug(f"Sufficient actions available ({remaining_count}), no inference needed")
                elif self.inference_pending:
                    self.get_logger().debug(f"Inference already pending, continuing with {remaining_count} actions")
                elif not self.inference_process or not self.inference_process.is_alive():
                    self.get_logger().warning("Inference process is not alive")
                    
        except Exception as e:
            self.get_logger().error(f'Inference timer callback error: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            self._stop_inference_with_error(f'Inference timer error: {str(e)}')

    def _process_inference_results(self):
        """Process inference results from the worker process"""
        try:
            results_processed = 0
            while True:
                try:
                    status, result = self.inference_output_queue.get_nowait()
                    results_processed += 1
                    
                    if status == 'success':
                        action_chunk = result['actions']
                        inference_time = result['inference_time']
                        inference_start_count = result.get('inference_start_action_count', 0)
                        
                        self.get_logger().info(
                            f'Inference completed in {inference_time*1000:.1f}ms, '
                            f'generated {len(action_chunk)} actions')

                        # Calculate offset
                        actions_executed_during_inference = self._used_action_count - inference_start_count
                        
                        self.get_logger().info(
                            f'Actions executed during inference: {actions_executed_during_inference} '
                            f'(from {inference_start_count} to {self._used_action_count})')
                        
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
                                f'  last_executed={self.last_executed_action[:3]}...')
                            self.get_logger().info(
                                f'  first_original={action_chunk[actions_executed_during_inference][:3]}...')
                            self.get_logger().info(
                                f'  first_smoothed={offset_action_chunk[0][:3]}...')
                            self.get_logger().info(
                                f'  final_smoothed={offset_action_chunk[num_transition_steps-1][:3]}...')
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
                        
                        self.get_logger().info(
                            f'Action buffer REPLACED: {old_count} -> {new_count} fresh actions '
                            f'(offset applied: {actions_executed_during_inference})')

                        # Update status
                        current_status = self.data_manager.get_current_record_status()
                        current_status.phase = TaskStatus.INFERENCING
                        self.communicator.publish_status(status=current_status)
                        
                    elif status == 'error':
                        self.inference_pending = False
                        self.get_logger().error(f'Received error from inference worker: {result}')
                        self._stop_inference_with_error(f'Inference process error: {result}')
                        return
                        
                    elif status == 'pong':
                        self.get_logger().debug("Received health check pong")
                        continue
                        
                except queue.Empty:
                    break
            
            if results_processed > 0:
                self.get_logger().debug(f"Processed {results_processed} results from inference worker")
                    
        except Exception as e:
            self.get_logger().error(f'Error processing inference results: {str(e)}')

    def _action_timer_callback(self):
        """Action timer callback - publishes actions at high frequency"""
        if not self.on_inference:
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
                
                # Collect action data for visualization
                if self.plot_enabled:
                    self.action_history.append({
                        'action_count': self._used_action_count + 1,
                        'timestamp': time.time(),
                        'action': action.copy() if isinstance(action, list) else list(action)
                    })
                    
                    # Keep only the last max_action_history actions
                    if len(self.action_history) > self.max_action_history:
                        self.action_history.pop(0)
                    
                    # Plot when we have enough data
                    if len(self.action_history) == self.max_action_history:
                        self._plot_action_graphs()
                        # Reset for next batch
                        self.action_history = []
                
                self.get_logger().info(
                    f'Publishing action {self._used_action_count + 1} '
                    f'(remaining: {remaining_count}): {action[:3]}... '
                    f'[History: {len(self.action_history)}/{self.max_action_history}]')
                
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
                        
                        # Stop inference process if running
                        if self.inference_process:
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
        """Start the inference process"""
        try:
            # Create multiprocessing queues
            self.inference_input_queue = multiprocessing.Queue(maxsize=5)
            self.inference_output_queue = multiprocessing.Queue(maxsize=10)
            
            # Start inference worker process
            self.inference_process = multiprocessing.Process(
                target=inference_worker_process,
                args=(self.inference_input_queue, self.inference_output_queue, policy_path, device),
                daemon=True
            )
            self.inference_process.start()
            
            # Wait for worker to be ready
            timeout = 30.0  # 30 seconds timeout
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                try:
                    status, message = self.inference_output_queue.get(timeout=1.0)
                    if status == 'ready':
                        self.get_logger().info('Inference process started successfully')
                        return True
                    elif status == 'error':
                        self.get_logger().error(f'Inference process failed to start: {message}')
                        return False
                except queue.Empty:
                    continue
                    
            self.get_logger().error('Inference process startup timeout')
            return False
            
        except Exception as e:
            self.get_logger().error(f'Failed to start inference process: {str(e)}')
            return False

    def _stop_inference_process(self):
        """Stop the inference process"""
        try:
            if self.inference_process and self.inference_process.is_alive():
                # Send shutdown signal
                if self.inference_input_queue:
                    try:
                        self.inference_input_queue.put(None, timeout=1.0)
                    except queue.Full:
                        pass
                
                # Wait for process to terminate
                self.inference_process.join(timeout=5.0)
                
                if self.inference_process.is_alive():
                    self.get_logger().warning('Inference process did not terminate gracefully, forcing termination')
                    self.inference_process.terminate()
                    self.inference_process.join(timeout=2.0)
                    
                    if self.inference_process.is_alive():
                        self.get_logger().error('Failed to terminate inference process, killing it')
                        self.inference_process.kill()
                        
            # Clean up queues
            if self.inference_input_queue:
                try:
                    while not self.inference_input_queue.empty():
                        self.inference_input_queue.get_nowait()
                except:
                    pass
                    
            if self.inference_output_queue:
                try:
                    while not self.inference_output_queue.empty():
                        self.inference_output_queue.get_nowait()
                except:
                    pass
                    
            self.inference_process = None
            self.inference_input_queue = None
            self.inference_output_queue = None
            self.get_logger().info('Inference process stopped')
            
        except Exception as e:
            self.get_logger().error(f'Error stopping inference process: {str(e)}')

    def _check_inference_process_health(self):
        """Check if the inference process is still alive and responsive"""
        if not self.inference_process or not self.inference_process.is_alive():
            self.get_logger().warning("Inference process is not alive")
            return False
            
        # Skip ping/pong for now to avoid interference - just check if process is alive
        return True

    def _stop_inference_with_error(self, error_msg):
        """Stop inference due to error"""
        self.get_logger().error(f"Stopping inference due to error: {error_msg}")
        self.on_inference = False
        self.inference_pending = False
        
        # Stop inference process
        if self.inference_process:
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
        # Clean up inference process
        if hasattr(node, 'inference_process') and node.inference_process:
            node._stop_inference_process()
            
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()