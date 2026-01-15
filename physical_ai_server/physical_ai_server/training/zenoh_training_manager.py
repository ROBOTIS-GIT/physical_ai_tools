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

import threading
from typing import Callable, Optional

from physical_ai_interfaces.msg import TrainingInfo, TrainingStatus
from physical_ai_server.communication import ZenohLeRobotClient, LeRobotResponse


class ZenohTrainingManager:
    """
    Training manager that delegates to LeRobot Docker container via Zenoh.
    
    This manager does not import lerobot directly. Instead, it sends commands
    to the isolated LeRobot Docker container through Zenoh communication.
    """
    
    SUPPORTED_POLICIES = [
        'tdmpc', 'diffusion', 'act', 'vqbet', 'pi0', 'pi0fast', 'smolvla'
    ]
    
    SUPPORTED_DEVICES = ['cuda', 'cpu']

    def __init__(self):
        self.training_info = TrainingInfo()
        self.client = ZenohLeRobotClient()
        self._connected = False
        self._status_callback: Optional[Callable] = None
        self._current_status = "idle"
        self._current_step = 0
        self._current_loss = float('nan')
        
        self.resume = False
        self.resume_model_path = None

    def connect(self) -> bool:
        if self._connected:
            return True
        self._connected = self.client.connect()
        if self._connected:
            self.client.subscribe_status(self._on_status_update)
        return self._connected

    def disconnect(self):
        if self._connected:
            self.client.disconnect()
            self._connected = False

    def _on_status_update(self, status_data: dict):
        self._current_status = status_data.get("status", "unknown")
        if self._status_callback:
            self._status_callback(status_data)

    def set_status_callback(self, callback: Callable):
        self._status_callback = callback

    @staticmethod
    def get_available_list() -> tuple[list[str], list[str]]:
        return (
            ZenohTrainingManager.SUPPORTED_POLICIES,
            ZenohTrainingManager.SUPPORTED_DEVICES
        )

    def get_current_training_status(self) -> TrainingStatus:
        status = TrainingStatus()
        status.training_info = self.training_info
        status.current_step = self._current_step
        status.current_loss = self._current_loss
        return status

    def train(self) -> LeRobotResponse:
        if not self._connected:
            if not self.connect():
                return LeRobotResponse(
                    success=False,
                    message="Failed to connect to LeRobot server",
                    data={},
                    request_id=""
                )

        if self.resume and self.resume_model_path:
            return self.client.resume_training(self.resume_model_path)
        
        return self.client.start_training(
            policy_type=self.training_info.policy_type,
            dataset_path=self.training_info.dataset,
            output_dir=self.training_info.output_folder_name or None,
            num_epochs=self.training_info.steps if self.training_info.steps > 0 else None,
            batch_size=self.training_info.batch_size if self.training_info.batch_size > 0 else None
        )

    def stop(self) -> LeRobotResponse:
        if not self._connected:
            return LeRobotResponse(
                success=False,
                message="Not connected to LeRobot server",
                data={},
                request_id=""
            )
        return self.client.stop_training()

    def get_status(self) -> LeRobotResponse:
        if not self._connected:
            return LeRobotResponse(
                success=False,
                message="Not connected to LeRobot server",
                data={},
                request_id=""
            )
        return self.client.get_training_status()
