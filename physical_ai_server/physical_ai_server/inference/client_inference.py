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

from io import BytesIO
from typing import Any, Dict

import torch
import zmq

import numpy as np

class ZmqInferenceClient:

    def __init__(
            self,
            host: str = 'localhost',
            port: int = 5555,
            timeout_ms: int = 15000):
        self.context = zmq.Context()
        self.host = host
        self.port = port
        self.timeout_ms = timeout_ms
        self.current_task_id = None
        self._init_socket()

    def _init_socket(self):
        if hasattr(self, 'socket'):
            self.socket.close()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self.socket.connect(f'tcp://{self.host}:{self.port}')

    def convert_dict_to_bytes(self, dict_data: dict) -> bytes:
        bytes_buffer = BytesIO()
        torch.save(dict_data, bytes_buffer)
        return bytes_buffer.getvalue()

    def convert_dict_from_bytes(self, bytes_data: bytes) -> dict:
        bytes_buffer = BytesIO(bytes_data)
        dict_data = torch.load(bytes_buffer, weights_only=False)
        return dict_data

    def execute_command(
        self,
        command: str,
        data: dict | None = None
    ) -> dict:
        request: dict = {'command': command}
        if data is not None:
            request['data'] = data
        try:
            self.socket.send(self.convert_dict_to_bytes(request))
            message = self.socket.recv()
            if message == b'ERROR':
                raise RuntimeError('Server error')
            response = self.convert_dict_from_bytes(message)

            if isinstance(response, dict) and response.get('status') == 'error':
                raise RuntimeError(f"Server error: {response.get('message', 'Unknown error')}")

            return response
        except zmq.error.Again:
            raise TimeoutError(f"Request timed out after {self.timeout_ms}ms")
        except zmq.error.ZMQError as e:
            raise RuntimeError(f"ZMQ error: {e}")

    def ping(self) -> bool:
        try:
            self.execute_command('ping')
            return True
        except zmq.error.ZMQError:
            self._init_socket()  # Recreate socket for next attempt
            return False

    def kill_server(self):
        self.execute_command('kill')

    def get_action(self, observations: Dict[str, Any]) -> Dict[str, Any]:
        return self.execute_command('get_action', observations)

    def start_inference(self, observations: Dict[str, Any]) -> str:
        """Start async inference and return task ID"""
        response = self.execute_command('start_inference', observations)
        if response.get('status') == 'ok':
            self.current_task_id = response.get('task_id')
            return self.current_task_id
        else:
            raise RuntimeError(f"Failed to start inference: {response.get('message')}")

    def stop_inference(self):
        """Stop all inference tasks, clean up thread pool, and unload policy"""
        response = self.execute_command('stop_inference')
        if response.get('status') == 'ok':
            self.current_task_id = None
            return response
        else:
            raise RuntimeError(f"Failed to stop inference: {response.get('message')}")

    def check_inference_ready(self, task_id: str = None) -> bool:
        """Check if inference is ready (lightweight check)"""
        if task_id is None:
            task_id = self.current_task_id
        if task_id is None:
            return False
            
        try:
            response = self.execute_command('check_inference', {'task_id': task_id})
            return response.get('is_ready', False)
        except Exception:
            return False

    def get_inference_result(self, task_id: str = None) -> Dict[str, Any]:
        """Get inference result and clean up task"""
        if task_id is None:
            task_id = self.current_task_id
        if task_id is None:
            raise RuntimeError("No task ID provided")
            
        result = self.execute_command('get_inference_result', {'task_id': task_id})
        
        # Clear current task if this was the current one
        if task_id == self.current_task_id:
            self.current_task_id = None
            
        return result

    def has_pending_inference(self) -> bool:
        """Check if there's a pending inference task"""
        return self.current_task_id is not None

    def load_policy(
            self,
            policy_info: Dict[str, Any]) -> Dict[str, Any]:
        return self.execute_command('load_policy', policy_info)
    
    def unload_policy(self) -> Dict[str, Any]:
        return self.execute_command('unload_policy')

    def __del__(self):
        self.socket.close()
        self.context.term()


def test_basic_communication(host='localhost', port=5555):
    """Test basic client-server communication"""
    import time
    
    print(f"Connecting to server at {host}:{port}...")
    
    client = ZmqInferenceClient(
        host=host,
        port=port,
        timeout_ms=50000
    )
    
    print("\n=== Testing Basic Communication ===")
    
    # Test 1: Ping
    print("1. Testing ping...")
    try:
        is_alive = client.ping()
        print(f"   ✅ Ping successful: {is_alive}")
    except Exception as e:
        print(f"   ❌ Ping failed: {e}")
        return False

    # Test 3: Load policy
    print("\n3. Testing load policy...")
    try:
        policy_info = {
            'policy_type': 'GR00T_N1_5',
            'policy_path': '/workspace/checkpoints/ROBOTIS/gr00t_test',
            'robot_type': 'ffw_bg2'
        }
        response = client.execute_command('load_policy', policy_info)
        print(f"   ✅ Load policy result: {response}")
    except Exception as e:
        print(f"   ❌ Load policy failed: {e}")
    
    # Test 4: Warmup
    print("\n4. Testing warmup...")
    try:
        start_time = time.time()
        obs = {
            "video.cam_head": np.random.randint(0, 256, (1, 224, 224, 3), dtype=np.uint8),
            "video.cam_head_right": np.random.randint(0, 256, (1, 224, 224, 3), dtype=np.uint8),
            "state.left_arm": np.random.rand(1, 8),
            "state.right_arm": np.random.rand(1, 8),
            "annotation.human.action.task_description": ["TEST"],
        }

        for _ in range(10):
            action = client.get_action(obs)
        end_time = time.time()
        print(f"   ✅ Warmup successful in {end_time - start_time:.2f}s")
    except Exception as e:
        print(f"   ❌ Warmup failed: {e}")
        return False

    # Test 5: Get action (mock inference)
    print("\n4. Testing get_action...")
    try:
        obs = {
            "video.cam_head": np.random.randint(0, 256, (1, 224, 224, 3), dtype=np.uint8),
            "video.cam_head_right": np.random.randint(0, 256, (1, 224, 224, 3), dtype=np.uint8),
            "state.left_arm": np.random.rand(1, 8),
            "state.right_arm": np.random.rand(1, 8),
            "annotation.human.action.task_description": ["TEST"],
        }
        for _ in range(50):
            start_time = time.time()
            action = client.get_action(obs)
            end_time = time.time()
            print(f"   ✅ Get action successful in {end_time - start_time:.2f}s")
            print(f"   ✅ Action response keys: {list(action.keys())}")
            print(f"   ✅ Sample action data: { {k: v.shape for k, v in action.items()} }")
    except Exception as e:
        print(f"   ❌ Get action failed: {e}")
    
    print("\n=== All Client Tests Complete ===")
    return True


def interactive_mode(host='localhost', port=5555):
    """Interactive mode for manual testing"""
    import time

    print(f"\n=== Interactive Mode ===")
    print("Enter commands or 'quit' to exit")
    print("Available commands: ping, echo, add, get_action, get_config")
    
    client = ZmqInferenceClient(
        policy_type="test_policy",
        host=host, 
        port=port,
        timeout_ms=50000
    )
    
    while True:
        try:
            command = input("\nEnter command: ").strip().lower()
            
            if command == 'quit':
                break
            elif command == 'ping':
                result = client.ping()
                print(f"Ping result: {result}")
            elif command == 'echo':
                message = input("Enter message to echo: ")
                data = {'message': message, 'timestamp': time.time()}
                response = client.execute_command('echo', data)
                print(f"Echo response: {response}")
            elif command == 'add':
                try:
                    a = float(input("Enter first number: "))
                    b = float(input("Enter second number: "))
                    data = {'a': a, 'b': b}
                    response = client.execute_command('add', data)
                    print(f"Addition result: {response}")
                except ValueError:
                    print("Please enter valid numbers")
            elif command == 'get_action':
                response = client.get_action({'test': 'data'})
                print(f"Action: {response}")
            elif command == 'get_config':
                response = client.execute_command('get_modality_config', requires_input=False)
                print(f"Config: {response}")
            else:
                print("Unknown command. Available: ping, echo, add, get_action, get_config, quit")

        except Exception as e:
            print(f"Error: {e}")


def main():
    """Main function for testing the client"""
    import argparse
    
    parser = argparse.ArgumentParser(description='ZMQ Inference Client')
    parser.add_argument('--host', default='localhost', help='Server host (default: localhost)')
    parser.add_argument('--port', type=int, default=5555, help='Server port (default: 5555)')
    parser.add_argument('--interactive', action='store_true', help='Run in interactive mode')
    parser.add_argument('--test-only', action='store_true', help='Run automated tests only')
    
    args = parser.parse_args()
    
    print("ZMQ Inference Client")
    print("=" * 20)
    
    if args.test_only:
        # Run automated tests only
        success = test_basic_communication(args.host, args.port)
        if success:
            print("\n✅ All tests passed!")
        else:
            print("\n❌ Some tests failed!")
    elif args.interactive:
        # Run interactive mode only
        interactive_mode(args.host, args.port)
    else:
        # Run tests first, then ask for interactive mode
        success = test_basic_communication(args.host, args.port)
        
        if success:
            while True:
                choice = input("\nRun interactive mode? (y/n): ").strip().lower()
                if choice in ['y', 'yes']:
                    interactive_mode(args.host, args.port)
                    break
                elif choice in ['n', 'no']:
                    break
                else:
                    print("Please enter 'y' or 'n'")


if __name__ == "__main__":
    main()
