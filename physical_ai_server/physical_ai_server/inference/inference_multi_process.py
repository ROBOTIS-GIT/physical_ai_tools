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


import logging
import multiprocessing
import os
import queue
import time
import numpy as np


class InferenceWorker:
    def __init__(self, policy_path, device='cuda'):
        self.policy_path = policy_path
        self.device = device
        self.input_queue = multiprocessing.Queue()
        self.output_queue = multiprocessing.Queue()
        self.process = None
        self.logger = logging.getLogger('InferenceWorker')
        # Basic config for the main process logger
        logging.basicConfig(
            level=logging.INFO,
            format='%(name)s - %(levelname)s - %(message)s')

    def start(self):
        if self.process and self.process.is_alive():
            self.logger.warning('Inference process is already running.')
            return False

        self.process = multiprocessing.Process(
            target=self._worker_process_loop,
            args=(
                self.input_queue,
                self.output_queue,
                self.policy_path,
                self.device
            )
        )
        self.process.start()
        self.logger.info(
            f'Inference worker process started with PID: {self.process.pid}')
        return True

    def stop(self, timeout=5.0):
        if not self.is_alive():
            self.logger.info(
                'Inference process is not running or already stopped.')
            return

        try:
            self.logger.info(
                'Sending shutdown signal to inference worker...')
            self.input_queue.put(None)
            self.process.join(timeout)
            if self.process.is_alive():
                self.logger.warning(
                    'Inference process did not terminate gracefully. Forcing termination.')
                self.process.terminate()
                self.process.join()
            self.logger.info('Inference process stopped.')
        except Exception as e:
            self.logger.error(f'Error stopping inference process: {e}')
        finally:
            self.process = None

    def is_alive(self):
        return self.process and self.process.is_alive()

    def send_request(self, data):
        if self.is_alive():
            self.input_queue.put(data)
            return True
        else:
            self.logger.error(
                'Cannot send request, inference process is not running.')
            return False

    def get_result(self, block=True, timeout=None):
        '''Retrieves a result from the worker.'''
        try:
            return self.output_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return None

    @staticmethod
    def _worker_process_loop(input_queue, output_queue, policy_path, device):
        '''
        The main loop for the worker process. This runs in a separate process.
        '''
        # Set up logging for the worker process
        logging.basicConfig(level=logging.INFO, 
                           format='[INFERENCE_WORKER] %(levelname)s: %(message)s')
        logger = logging.getLogger('inference_worker')
        
        try:
            logger.info(f'Worker process started with PID: {os.getpid()}')
            logger.info(f'Policy path: {policy_path}')
            logger.info(f'Device: {device}')
            
            # Import here to avoid issues with multiprocessing
            from physical_ai_server.inference import InferenceFactory
            logger.info('InferenceFactory imported successfully')
            
            # Initialize inference manager in separate process
            logger.info('Creating inference manager...')
            inference_manager = InferenceFactory.create_inference_manager('lerobot', device=device)
            logger.info('Inference manager created successfully')
            
            # Validate and load policy
            logger.info(f'Validating policy: {policy_path}')
            valid_result, result_message = inference_manager.validate_policy(policy_path=policy_path)
            if not valid_result:
                error_msg = f'Policy validation failed: {result_message}'
                logger.error(error_msg)
                output_queue.put(('error', error_msg))
                return
            logger.info('Policy validation successful')
                
            logger.info('Loading policy... (This may take a while for large VLM models)')
            # Send loading status to main process
            output_queue.put(('loading', 'Policy loading in progress...'))
            
            if not inference_manager.load_policy():
                error_msg = 'Failed to load policy'
                logger.error(error_msg)
                output_queue.put(('error', error_msg))
                return
            logger.info('Policy loaded successfully')
                
            output_queue.put(('ready', 'Inference worker ready'))
            logger.info('Worker is ready and waiting for requests')
            
            request_count = 0
            last_log_time = time.time()
            
            while True:
                try:
                    # Log periodic status
                    current_time = time.time()
                    if current_time - last_log_time > 10.0:  # Every 10 seconds
                        logger.info(
                            f'Worker still alive, processed {request_count} requests so far')
                        logger.info(
                            f'Input queue size: {input_queue.qsize()}')
                        last_log_time = current_time
                    
                    # Check for new inference requests
                    try:
                        data = input_queue.get(timeout=1.0)

                        if data is None:  # Shutdown signal
                            logger.info('Received shutdown signal')
                            break
                        elif data == 'ping':  # Health check
                            logger.debug('Received health check ping')
                            output_queue.put(('pong', 'Worker alive'))
                            continue

                        request_count += 1
                        logger.info(f'*** Received inference request #{request_count} ***')

                        if len(data) == 4:
                            camera_data, follower_data, task_instruction, inference_start_action_count = data
                        else:
                            # Backward compatibility
                            camera_data, follower_data, task_instruction = data
                            inference_start_action_count = 0

                        # *** SIMPLIFIED: Log key input validation info ***
                        logger.info(f'🧪 INFERENCE WORKER INPUT #{request_count}:')
                        logger.info(f'  Start Action: {inference_start_action_count}, Task: {task_instruction}')
                        
                        # Log camera data freshness
                        if isinstance(camera_data, dict):
                            logger.info(f'  Camera: dict with {len(camera_data)} cameras')
                            for cam_name, cam_msg in camera_data.items():
                                if hasattr(cam_msg, 'header') and hasattr(cam_msg.header, 'stamp'):
                                    cam_timestamp = cam_msg.header.stamp.sec + cam_msg.header.stamp.nanosec * 1e-9
                                    current_time = time.time()
                                    age_ms = (current_time - cam_timestamp) * 1000
                                    freshness = "FRESH" if age_ms <= 50 else "STALE"
                                    logger.info(f'    📸 {cam_name}: {freshness} (age: {age_ms:.1f}ms)')
                        
                        # Log follower data  
                        if isinstance(follower_data, (list, tuple)):
                            logger.info(f'  🤖 Joints: {follower_data[:7]}')  # First 7 joints

                        # Run inference
                        logger.info('Starting inference...')
                        start_time = time.time()
                        action_chunk = inference_manager.predict_chunk(
                            images=camera_data,
                            state=follower_data,
                            task_instruction=task_instruction
                        )
                        inference_time = time.time() - start_time
                        logger.info(f'Inference completed in {inference_time*1000:.1f}ms')

                        # Convert to list if it's a numpy array
                        if isinstance(action_chunk, np.ndarray):
                            original_shape = action_chunk.shape
                            action_chunk = action_chunk.tolist()
                            logger.info(f'Converted numpy array {original_shape} to list, length: {len(action_chunk)}')

                        # *** SIMPLIFIED: Log key output validation info ***
                        logger.info(f'🧪 INFERENCE WORKER OUTPUT #{request_count}:')
                        logger.info(f'  Generated {len(action_chunk)} actions in {inference_time*1000:.1f}ms')
                        
                        if action_chunk and len(action_chunk) > 0:
                            # Log first action as fingerprint
                            first_action = action_chunk[0]
                            if hasattr(first_action, '__len__') and len(first_action) >= 7:
                                logger.info(f'  🎯 First Action: {first_action[:7]}')
                            
                            # Log action statistics for first 3 joints only
                            if hasattr(action_chunk[0], '__len__') and len(action_chunk[0]) > 0:
                                actions_array = np.array(action_chunk)
                                logger.info(f'  📊 Stats: shape={actions_array.shape}')
                                for joint_idx in range(min(3, actions_array.shape[1])):
                                    joint_values = actions_array[:, joint_idx]
                                    logger.info(f'    J{joint_idx}: [{joint_values.min():.3f}, {joint_values.max():.3f}] mean={joint_values.mean():.3f}')

                        # Validate the converted result
                        if not action_chunk or len(action_chunk) == 0:
                            error_msg = f'Invalid action chunk after conversion: empty or None'
                            logger.error(error_msg)
                            output_queue.put(('error', error_msg))
                            continue

                        # Send result back with inference start action count for offset calculation
                        result = {
                            'actions': action_chunk,
                            'inference_time': inference_time,
                            'inference_start_action_count': inference_start_action_count
                        }
                        output_queue.put(('success', result))
                        logger.info(f'*** Result sent back with {len(action_chunk)} actions '
                                  f'(inference started at action {inference_start_action_count}) ***')

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
        
        logger.info('Worker process shutting down')