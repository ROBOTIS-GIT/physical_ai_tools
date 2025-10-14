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

from dataclasses import dataclass
from io import BytesIO
from typing import Callable
import time
import threading
import uuid

import torch
import zmq


class ZmqInferenceServer:
    def __init__(
            self,
            server_address: str,
            port: int = 5555):

        self.running = True
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f'tcp://{server_address}:{port}')
        self._callback_group = {}
        self.policy = None
        
        # Async inference management
        self.inference_tasks = {}  # task_id -> {status, result, thread}
        self.inference_lock = threading.Lock()

        self.add_callback(name='ping', callback=self._ping_callback)
        self.add_callback(name='kill', callback=self._kill_server_callback)
        self.add_callback(name='load_policy', callback=self._load_policy_callback)
        self.add_callback(name='unload_policy', callback=self._unload_policy_callback)
        self.add_callback(name='start_inference', callback=self._start_inference_callback)
        self.add_callback(name='check_inference', callback=self._check_inference_callback)
        self.add_callback(name='get_inference_result', callback=self._get_inference_result_callback)

    def add_callback(
            self,
            name: str,
            callback: Callable):
        self._callback_group[name] = callback

    def convert_dict_to_bytes(self, data: dict) -> bytes:
        bytes_buffer = BytesIO()
        torch.save(data, bytes_buffer)
        return bytes_buffer.getvalue()

    def convert_dict_from_bytes(self, data: bytes) -> dict:
        bytes_buffer = BytesIO(data)
        dict_data = torch.load(bytes_buffer, weights_only=False)
        return dict_data

    def _ping_callback(self, data) -> dict:
        return {'status': 'ok', 'message': 'Server is running'}

    def _kill_server_callback(self, data):
        self.running = False

    def _load_policy_callback(self, data: dict) -> dict:
        if (
            'policy_type' not in data or
            'policy_path' not in data or
            'robot_type' not in data
        ):
            return {
                'status': 'error',
                'message': "Missing required fields: 'policy_type', 'policy_path', 'robot_type'"
            }

        try:
            if data['policy_type'] == 'GR00T_N1_5':
                from gr00t.experiment.data_config import load_data_config
                from gr00t.model.policy import Gr00tPolicy
                data_config = load_data_config(data['robot_type'])
                self.policy = Gr00tPolicy(
                    model_path=data['policy_path'],
                    modality_config=data_config.modality_config(),
                    modality_transform=data_config.transform(),
                    embodiment_tag='new_embodiment',
                    denoising_steps=data.get('denoising_steps', 4),
                )
                self.add_callback('get_action', self.policy.get_action)
                return {
                    'status': 'ok',
                    'message': 'Policy loaded successfully'
                }
            elif data['policy_type'] == 'GR00T_N1_5_TRT':
                from gr00t.experiment.data_config import load_data_config
                from gr00t.model.policy import Gr00tPolicy
                from trt_model_forward import setup_tensorrt_engines
                data_config = load_data_config(data['robot_type'])
                self.policy = Gr00tPolicy(
                    model_path=data['policy_path'],
                    modality_config=data_config.modality_config(),
                    modality_transform=data_config.transform(),
                    embodiment_tag='new_embodiment',
                    denoising_steps=data.get('denoising_steps', 4),
                )
                setup_tensorrt_engines(self.policy, '/workspace/checkpoints/ROBOTIS/ffw_bg2_rev4_pick_coffee_bottle_env5_1_to_31_joint_fix_20k_engine')

                self.add_callback('get_action', self.policy.get_action)
                return {
                    'status': 'ok',
                    'message': 'Policy loaded successfully'
                }
            else:
                return {
                    'status': 'error',
                    'message': 'Policy not supported yet'
                }
        except Exception as e:
            return {
                'status': 'error',
                'message': f"Failed to load policy: {e}"
            }

    def _unload_policy_callback(self, data) -> dict:
        if self.policy is None:
            return {'status': 'error', 'message': 'No policy loaded'}
        self.policy = None
        if 'get_action' in self._callback_group:
            del self._callback_group['get_action']
        
        # Clear all pending inference tasks
        with self.inference_lock:
            self.inference_tasks.clear()
            
        return {'status': 'ok', 'message': 'Policy unloaded successfully'}

    def _start_inference_callback(self, data: dict) -> dict:
        """Start async inference and return task ID immediately"""
        if self.policy is None:
            return {
                'status': 'error',
                'message': 'No policy loaded'
            }
        
        # Generate unique task ID
        task_id = str(uuid.uuid4())
        
        # Create task entry
        with self.inference_lock:
            self.inference_tasks[task_id] = {
                'status': 'processing',
                'result': None,
                'thread': None
            }
        
        # Start inference in background thread
        def run_inference():
            try:
                result = self.policy.get_action(data)
                with self.inference_lock:
                    if task_id in self.inference_tasks:
                        self.inference_tasks[task_id]['status'] = 'completed'
                        self.inference_tasks[task_id]['result'] = result
            except Exception as e:
                with self.inference_lock:
                    if task_id in self.inference_tasks:
                        self.inference_tasks[task_id]['status'] = 'error'
                        self.inference_tasks[task_id]['result'] = {'error': str(e)}
        
        thread = threading.Thread(target=run_inference)
        thread.daemon = True
        
        with self.inference_lock:
            self.inference_tasks[task_id]['thread'] = thread
        
        thread.start()
        
        return {
            'status': 'ok',
            'task_id': task_id,
            'message': 'Inference started'
        }

    def _check_inference_callback(self, data: dict) -> dict:
        """Check if inference task is completed without heavy data transfer"""
        task_id = data.get('task_id')
        if not task_id:
            return {
                'status': 'error',
                'message': 'task_id required'
            }
        
        with self.inference_lock:
            if task_id not in self.inference_tasks:
                return {
                    'status': 'error',
                    'message': 'Task not found'
                }
            
            task_status = self.inference_tasks[task_id]['status']
            
        return {
            'status': 'ok',
            'task_status': task_status,
            'is_ready': task_status in ['completed', 'error']
        }

    def _get_inference_result_callback(self, data: dict) -> dict:
        """Get inference result and clean up task"""
        task_id = data.get('task_id')
        if not task_id:
            return {
                'status': 'error',
                'message': 'task_id required'
            }
        
        with self.inference_lock:
            if task_id not in self.inference_tasks:
                return {
                    'status': 'error',
                    'message': 'Task not found'
                }
            
            task = self.inference_tasks[task_id]
            if task['status'] == 'processing':
                return {
                    'status': 'error',
                    'message': 'Inference still processing'
                }
            
            result = task['result']
            # Clean up completed task
            del self.inference_tasks[task_id]
            
        if task['status'] == 'error':
            return {
                'status': 'error',
                'message': f"Inference failed: {result.get('error', 'Unknown error')}"
            }
        
        return result

    def run(self):
        addr = self.socket.getsockopt_string(zmq.LAST_ENDPOINT)
        print(f'Server is ready and listening on {addr}')
        while self.running:
            try:
                message = self.socket.recv()
                request = self.convert_dict_from_bytes(message)
                command = request.get('command', 'get_action')

                if command not in self._callback_group:
                    error_response = {
                        'status': 'error',
                        'message': f'Unknown command: {command}', 
                    }
                    self.socket.send(self.convert_dict_to_bytes(error_response))
                    continue

                if command == 'load_policy' and self.policy is not None:
                    error_response = {
                        'status': 'error',
                        'message': 'Policy already loaded. Unload it first.'
                    }
                    self.socket.send(self.convert_dict_to_bytes(error_response))
                    continue

                callback = self._callback_group[command]
                result = (
                    callback(request.get('data', {}))
                )
                self.socket.send(self.convert_dict_to_bytes(result))
            except Exception as e:
                print(f'Error in server: {e}')
                import traceback

                print(traceback.format_exc())
                error_response = {'status': 'error', 'message': str(e)}
                self.socket.send(self.convert_dict_to_bytes(error_response))
        
        # Cleanup when server stops
        self.socket.close()
        self.context.term()

def main():
    """Main function for testing the server"""
    import argparse
    import time
    
    parser = argparse.ArgumentParser(description='ZMQ Inference Server')
    parser.add_argument('--host', default='localhost', help='Server host (default: localhost)')
    parser.add_argument('--port', type=int, default=5555, help='Server port (default: 5555)')

    args = parser.parse_args()

    print(f"Starting ZMQ Inference Server on {args.host}:{args.port}")

    server = ZmqInferenceServer(args.host, args.port)

    try:
        server.run()
    except KeyboardInterrupt:
        print("\nShutting down server...")
    except Exception as e:
        print(f"Server error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
