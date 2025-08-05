import logging
import multiprocessing
import os
import queue
import time
import numpy as np


class InferenceWorker:
    """
    Manages a separate inference process to avoid GIL blocking and encapsulates
    all related logic, including process lifecycle and communication.
    """
    def __init__(self, policy_path, device='cuda'):
        self.policy_path = policy_path
        self.device = device
        self.input_queue = multiprocessing.Queue()
        self.output_queue = multiprocessing.Queue()
        self.process = None
        self.logger = logging.getLogger('InferenceWorker')
        # Basic config for the main process logger
        logging.basicConfig(level=logging.INFO, format='%(name)s - %(levelname)s - %(message)s')

    def start(self):
        """Starts the inference worker process."""
        if self.process and self.process.is_alive():
            self.logger.warning("Inference process is already running.")
            return False

        self.process = multiprocessing.Process(
            target=self._worker_process_loop,
            args=(self.input_queue, self.output_queue, self.policy_path, self.device)
        )
        self.process.start()
        self.logger.info(f"Inference worker process started with PID: {self.process.pid}")
        return True

    def stop(self, timeout=5.0):
        """Stops the inference worker process."""
        if not self.is_alive():
            self.logger.info("Inference process is not running or already stopped.")
            return

        try:
            self.logger.info("Sending shutdown signal to inference worker...")
            self.input_queue.put(None)
            self.process.join(timeout)
            if self.process.is_alive():
                self.logger.warning("Inference process did not terminate gracefully. Forcing termination.")
                self.process.terminate()
                self.process.join()
            self.logger.info("Inference process stopped.")
        except Exception as e:
            self.logger.error(f"Error stopping inference process: {e}")
        finally:
            self.process = None

    def is_alive(self):
        """Checks if the inference process is alive."""
        return self.process and self.process.is_alive()

    def send_request(self, data):
        """Sends an inference request to the worker."""
        if self.is_alive():
            self.input_queue.put(data)
            return True
        else:
            self.logger.error("Cannot send request, inference process is not running.")
            return False

    def get_result(self, block=True, timeout=None):
        """Retrieves a result from the worker."""
        try:
            return self.output_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return None

    @staticmethod
    def _worker_process_loop(input_queue, output_queue, policy_path, device):
        """
        The main loop for the worker process. This runs in a separate process.
        """
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
                
            logger.info("Loading policy... (This may take a while for large VLM models)")
            # Send loading status to main process
            output_queue.put(('loading', 'Policy loading in progress...'))
            
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
                        
                        # Log observation characteristics for debugging stale data issues
                        if isinstance(follower_data, dict):
                            for key, value in follower_data.items():
                                if hasattr(value, 'shape'):
                                    logger.info(f"  Follower[{key}] shape: {value.shape}, sample: {value.flatten()[:3]}")
                                elif isinstance(value, (list, tuple)):
                                    logger.info(f"  Follower[{key}] length: {len(value)}, sample: {value[:3]}")
                        
                        if isinstance(camera_data, dict):
                            for key, value in camera_data.items():
                                if hasattr(value, 'shape'):
                                    logger.info(f"  Camera[{key}] shape: {value.shape}")
                                    # Log some pixel statistics to detect static images
                                    if len(value.shape) >= 2:
                                        pixel_mean = np.mean(value) if hasattr(value, 'mean') else 0
                                        pixel_std = np.std(value) if hasattr(value, 'std') else 0
                                        logger.info(f"    Pixel stats - mean: {pixel_mean:.2f}, std: {pixel_std:.2f}")
                        
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
                        
                        # Detailed validation and logging before sending result
                        if action_chunk is None:
                            error_msg = "Inference returned None action chunk"
                            logger.error(error_msg)
                            output_queue.put(('error', error_msg))
                            continue
                        
                        # Check data type and structure
                        logger.info(f"Raw inference result type: {type(action_chunk)}")
                        if hasattr(action_chunk, 'shape'):
                            logger.info(f"Raw inference result shape: {action_chunk.shape}")
                        elif hasattr(action_chunk, '__len__'):
                            logger.info(f"Raw inference result length: {len(action_chunk)}")
                        
                        # Convert to list if it's a numpy array
                        if isinstance(action_chunk, np.ndarray):
                            original_shape = action_chunk.shape
                            action_chunk = action_chunk.tolist()
                            logger.info(f"Converted numpy array {original_shape} to list, length: {len(action_chunk)}")
                        
                        # Validate the converted result
                        if not action_chunk or len(action_chunk) == 0:
                            error_msg = f"Invalid action chunk after conversion: empty or None"
                            logger.error(error_msg)
                            output_queue.put(('error', error_msg))
                            continue
                        
                        # Check first action for structure validation
                        first_action = action_chunk[0]
                        if not isinstance(first_action, (list, tuple)) and not hasattr(first_action, '__len__'):
                            error_msg = f"Invalid first action structure: {type(first_action)}, value: {first_action}"
                            logger.error(error_msg)
                            output_queue.put(('error', error_msg))
                            continue
                        
                        joint_count = len(first_action)
                        logger.info(f"Action chunk validation successful:")
                        logger.info(f"  - Chunk size: {len(action_chunk)} actions")
                        logger.info(f"  - Joint count: {joint_count}")
                        logger.info(f"  - First action: {first_action}")
                        logger.info(f"  - Last action: {action_chunk[-1]}")
                        
                        # Additional statistics for Joint 0 analysis
                        if joint_count > 0:
                            joint_0_values = [action[0] for action in action_chunk if len(action) > 0]
                            if joint_0_values:
                                joint_0_min = min(joint_0_values)
                                joint_0_max = max(joint_0_values)
                                joint_0_mean = sum(joint_0_values) / len(joint_0_values)
                                logger.info(f"  - Joint 0 statistics: min={joint_0_min:.4f}, max={joint_0_max:.4f}, mean={joint_0_mean:.4f}")
                                
                                # Check for unusual values
                                if abs(joint_0_max - joint_0_min) > 1.0:  # Large range
                                    logger.warning(f"  - LARGE Joint 0 range detected: {joint_0_max - joint_0_min:.4f}")
                                if abs(joint_0_mean) > 1.0:  # Large mean
                                    logger.warning(f"  - LARGE Joint 0 mean detected: {joint_0_mean:.4f}")
                        
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