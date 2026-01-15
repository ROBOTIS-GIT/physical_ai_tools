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
# Author: Physical AI Team

from typing import Callable, Optional, List
import numpy as np

from physical_ai_server.communication import ZenohLeRobotClient, LeRobotResponse


class ZenohInferenceManager:
    """
    Inference manager that delegates to LeRobot Docker container via Zenoh.
    
    This manager does not import lerobot directly. Instead, it sends commands
    to the isolated LeRobot Docker container through Zenoh communication.
    
    For real-time inference (Mode 1), the LeRobot container publishes actions
    directly to the action topic which this manager subscribes to.
    """
    
    SUPPORTED_POLICIES = [
        'tdmpc', 'diffusion', 'act', 'vqbet', 'pi0', 'pi0fast', 'smolvla'
    ]

    def __init__(self):
        self.client = ZenohLeRobotClient()
        self._connected = False
        self._action_callback: Optional[Callable] = None
        self._status_callback: Optional[Callable] = None
        self._current_status = "idle"
        self.policy_path: Optional[str] = None
        self.policy_type: Optional[str] = None

    def connect(self) -> bool:
        if self._connected:
            return True
        self._connected = self.client.connect()
        if self._connected:
            self.client.subscribe_status(self._on_status_update)
            self.client.subscribe_actions(self._on_action_received)
        return self._connected

    def disconnect(self):
        if self._connected:
            self.client.disconnect()
            self._connected = False

    def _on_status_update(self, status_data: dict):
        self._current_status = status_data.get("status", "unknown")
        if self._status_callback:
            self._status_callback(status_data)

    def _on_action_received(self, action_data: dict):
        if self._action_callback:
            self._action_callback(action_data)

    def set_action_callback(self, callback: Callable[[dict], None]):
        self._action_callback = callback

    def set_status_callback(self, callback: Callable[[dict], None]):
        self._status_callback = callback

    def validate_policy(self, policy_path: str) -> tuple[bool, str]:
        self.policy_path = policy_path
        return True, f"Policy path set: {policy_path}"

    def load_policy(self) -> LeRobotResponse:
        if not self._connected:
            if not self.connect():
                return LeRobotResponse(
                    success=False,
                    message="Failed to connect to LeRobot server",
                    data={},
                    request_id=""
                )
        
        if not self.policy_path:
            return LeRobotResponse(
                success=False,
                message="Policy path not set",
                data={},
                request_id=""
            )
        
        return self.client.load_model(self.policy_path)

    def clear_policy(self) -> LeRobotResponse:
        if not self._connected:
            return LeRobotResponse(
                success=False,
                message="Not connected to LeRobot server",
                data={},
                request_id=""
            )
        return self.client.unload_model()

    def start_inference(
        self,
        model_path: str = None,
        env_name: str = None,
        num_episodes: int = None
    ) -> LeRobotResponse:
        if not self._connected:
            if not self.connect():
                return LeRobotResponse(
                    success=False,
                    message="Failed to connect to LeRobot server",
                    data={},
                    request_id=""
                )
        
        path = model_path or self.policy_path
        if not path:
            return LeRobotResponse(
                success=False,
                message="Model path not specified",
                data={},
                request_id=""
            )
        
        return self.client.start_inference(path, env_name, num_episodes)

    def stop_inference(self) -> LeRobotResponse:
        if not self._connected:
            return LeRobotResponse(
                success=False,
                message="Not connected to LeRobot server",
                data={},
                request_id=""
            )
        return self.client.stop_inference()

    def get_status(self) -> LeRobotResponse:
        if not self._connected:
            return LeRobotResponse(
                success=False,
                message="Not connected to LeRobot server",
                data={},
                request_id=""
            )
        return self.client.get_inference_status()

    def list_models(self) -> LeRobotResponse:
        if not self._connected:
            if not self.connect():
                return LeRobotResponse(
                    success=False,
                    message="Failed to connect to LeRobot server",
                    data={},
                    request_id=""
                )
        return self.client.list_models()

    @staticmethod
    def get_available_policies() -> List[str]:
        return ZenohInferenceManager.SUPPORTED_POLICIES
