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

import logging
import time
from typing import Callable, Optional

from physical_ai_interfaces.msg import TrainingInfo, TrainingStatus
from physical_ai_server.communication import LeRobotResponse, ZenohLeRobotClient

logger = logging.getLogger(__name__)


class ZenohTrainingManager:
    """
    Training manager that delegates to LeRobot Docker container via Zenoh.

    This manager does not import lerobot directly. Instead, it sends commands
    to the isolated LeRobot Docker container through Zenoh communication.
    """

    SUPPORTED_POLICIES = [
        'tdmpc', 'diffusion', 'act', 'vqbet', 'pi0', 'pi0_fast', 'pi05',
        'smolvla', 'groot', 'xvla', 'sac'
    ]

    SUPPORTED_DEVICES = ['cuda', 'cpu']

    _cached_policies: Optional[list] = None
    _cached_policy_details: Optional[list] = None

    def __init__(self):
        self.training_info = TrainingInfo()
        self.client = ZenohLeRobotClient()
        self._connected = False
        self._status_callback: Optional[Callable] = None
        self._current_status = 'idle'
        self._current_step = 0
        self._total_steps = 0
        self._current_loss = float('nan')
        self._training_completed = False

        self.resume = False
        self.resume_model_path = None

    def connect(self) -> bool:
        if self._connected:
            return True
        self._connected = self.client.connect()
        if self._connected:
            self.client.subscribe_status(self._on_status_update)
            self.client.subscribe_training_log(self._on_training_log_update)
        return self._connected

    def disconnect(self):
        if self._connected:
            self.client.disconnect()
            self._connected = False

    def _on_status_update(self, status_data: dict):
        new_status = status_data.get('status', 'unknown')

        if 'step' in status_data:
            self._current_step = status_data.get('step', 0)
        if 'total_steps' in status_data:
            self._total_steps = status_data.get('total_steps', 0)
        if 'loss' in status_data:
            loss_value = status_data.get('loss')
            if loss_value is not None and loss_value != 0:
                self._current_loss = float(loss_value)

        if self._current_status == 'training' and new_status == 'idle':
            self._training_completed = True
            logger.info('Training completed (state changed to idle)')
        elif new_status == 'error':
            self._training_completed = True
            logger.error('Training failed with error')

        self._current_status = new_status

        if self._status_callback:
            self._status_callback(status_data)

    def _on_training_log_update(self, log_data: dict):
        """Handle detailed training log updates from lerobot/training_log topic."""
        if 'step' in log_data:
            self._current_step = log_data.get('step', 0)
        if 'loss' in log_data:
            loss_value = log_data.get('loss')
            if loss_value is not None and loss_value != 0:
                self._current_loss = float(loss_value)

    def set_status_callback(self, callback: Callable):
        self._status_callback = callback

    @staticmethod
    def get_available_list() -> tuple[list[str], list[str]]:
        """Get available policies and devices."""
        if ZenohTrainingManager._cached_policies is None:
            ZenohTrainingManager._fetch_policies_from_container()

        policy_list = (
            ZenohTrainingManager._cached_policies
            if ZenohTrainingManager._cached_policies
            else ZenohTrainingManager.SUPPORTED_POLICIES
        )

        return (policy_list, ZenohTrainingManager.SUPPORTED_DEVICES)

    @staticmethod
    def get_policy_details() -> list:
        """Get detailed policy information from LeRobot container."""
        if ZenohTrainingManager._cached_policy_details is None:
            ZenohTrainingManager._fetch_policies_from_container()

        return ZenohTrainingManager._cached_policy_details or []

    @staticmethod
    def _fetch_policies_from_container():
        """Fetch policy list from LeRobot container via ROS2."""
        pass

    def get_current_training_status(self) -> TrainingStatus:
        status = TrainingStatus()
        status.training_info = self.training_info
        status.current_step = self._current_step
        status.current_loss = self._current_loss
        return status

    def train(self) -> LeRobotResponse:
        """Start training and wait for completion."""
        if not self._connected:
            if not self.connect():
                return LeRobotResponse(
                    success=False,
                    message='Failed to connect to LeRobot server',
                    data={},
                    request_id=''
                )

        self._training_completed = False
        self._current_status = 'idle'

        if self.resume and self.resume_model_path:
            response = self.client.resume_training(self.resume_model_path)
        else:
            steps = self.training_info.steps if self.training_info.steps > 0 else 0
            batch = self.training_info.batch_size if self.training_info.batch_size > 0 else 0
            response = self.client.start_training(
                policy_type=self.training_info.policy_type,
                dataset_path=self.training_info.dataset,
                output_dir=self.training_info.output_folder_name or '',
                num_epochs=steps,
                batch_size=batch,
                eval_freq=self.training_info.eval_freq if self.training_info.eval_freq > 0 else 0,
                log_freq=self.training_info.log_freq if self.training_info.log_freq > 0 else 0,
                save_freq=self.training_info.save_freq if self.training_info.save_freq > 0 else 0
            )

        if not response.success:
            return response

        timeout = 3600
        start_time = time.time()

        while not self._training_completed:
            time.sleep(1)

            if time.time() - start_time > timeout:
                logger.warning(f'Training timeout after {timeout}s')
                break

        final_response = LeRobotResponse(
            success=self._current_status == 'idle',
            message=f'Training {self._current_status}',
            data={
                'status': self._current_status,
                'step': self._current_step,
                'loss': self._current_loss
            },
            request_id=''
        )

        return final_response

    def stop(self) -> LeRobotResponse:
        if not self._connected:
            return LeRobotResponse(
                success=False,
                message='Not connected to LeRobot server',
                data={},
                request_id=''
            )
        return self.client.stop_training()

    def get_status(self) -> LeRobotResponse:
        if not self._connected:
            return LeRobotResponse(
                success=False,
                message='Not connected to LeRobot server',
                data={},
                request_id=''
            )
        return self.client.get_training_status()
