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

"""
Training Manager Adapter

Provides a unified interface for training via Docker execution (Zenoh communication).
LeRobot runs inside Docker container, not on the host.

Usage:
    # Docker mode (default, requires LeRobot Docker container running)
    adapter = TrainingManagerAdapter()
    
    # Or explicitly specify docker backend
    adapter = TrainingManagerAdapter(backend='docker')
"""

import threading
from enum import Enum
from typing import Callable, Optional, Union

from physical_ai_interfaces.msg import TrainingInfo, TrainingStatus


class TrainingBackend(Enum):
    """Backend execution mode for training."""
    LOCAL = 'local'
    DOCKER = 'docker'


class TrainingManagerAdapter:
    
    SUPPORTED_POLICIES = [
        'tdmpc', 'diffusion', 'act', 'vqbet', 'pi0', 'pi0_fast', 'pi05',
        'smolvla', 'groot', 'xvla', 'sac'
    ]
    
    SUPPORTED_DEVICES = ['cuda', 'cpu']
    
    _cached_policies: list = None

    def __init__(self, backend: Union[str, TrainingBackend] = 'docker'):
        """
        Initialize training manager adapter.
        
        Parameters
        ----------
        backend : str or TrainingBackend
            Execution backend: 'local' or 'docker'
        """
        if isinstance(backend, str):
            backend = TrainingBackend(backend.lower())
        
        self._backend = backend
        self._manager = None
        self._initialized = False
        self.stop_event = threading.Event()
        
        # Common properties
        self._training_info = TrainingInfo()
        self._resume = False
        self._resume_model_path = None
        
        self._init_backend()

    def _init_backend(self):
        """Initialize the appropriate backend manager."""
        if self._backend == TrainingBackend.LOCAL:
            self._init_local_backend()
        else:
            self._init_docker_backend()

    def _init_local_backend(self):
        """Initialize local training manager."""
        try:
            from physical_ai_server.training.training_manager import TrainingManager
            self._manager = TrainingManager()
            self._initialized = True
        except ImportError as e:
            raise ImportError(
                f"Failed to import local TrainingManager. "
                f"Ensure lerobot is installed: {e}"
            )

    def _init_docker_backend(self):
        """Initialize Docker/Zenoh training manager."""
        try:
            from physical_ai_server.training.zenoh_training_manager import ZenohTrainingManager
            self._manager = ZenohTrainingManager()
            self._initialized = True
        except ImportError as e:
            raise ImportError(
                f"Failed to import ZenohTrainingManager: {e}"
            )

    @property
    def backend(self) -> TrainingBackend:
        """Get current backend."""
        return self._backend

    @property
    def training_info(self) -> TrainingInfo:
        """Get training info."""
        return self._training_info

    @training_info.setter
    def training_info(self, value: TrainingInfo):
        """Set training info and propagate to underlying manager."""
        self._training_info = value
        if self._manager:
            self._manager.training_info = value

    @property
    def resume(self) -> bool:
        """Get resume flag."""
        return self._resume

    @resume.setter
    def resume(self, value: bool):
        """Set resume flag and propagate to underlying manager."""
        self._resume = value
        if self._manager:
            self._manager.resume = value

    @property
    def resume_model_path(self) -> Optional[str]:
        """Get resume model path."""
        return self._resume_model_path

    @resume_model_path.setter
    def resume_model_path(self, value: Optional[str]):
        """Set resume model path and propagate to underlying manager."""
        self._resume_model_path = value
        if self._manager:
            self._manager.resume_model_path = value

    @staticmethod
    def get_available_list() -> tuple[list[str], list[str]]:
        if TrainingManagerAdapter._cached_policies is None:
            TrainingManagerAdapter._fetch_policies_from_container()
        
        policy_list = (
            TrainingManagerAdapter._cached_policies
            if TrainingManagerAdapter._cached_policies
            else TrainingManagerAdapter.SUPPORTED_POLICIES
        )
        return (policy_list, TrainingManagerAdapter.SUPPORTED_DEVICES)
    
    @staticmethod
    def _fetch_policies_from_container():
        try:
            from physical_ai_server.training.zenoh_training_manager import ZenohTrainingManager
            policies, _ = ZenohTrainingManager.get_available_list()
            if policies:
                TrainingManagerAdapter._cached_policies = policies
        except Exception:
            pass

    @staticmethod
    def get_weight_save_root_path():
        """
        Get the root path for saving training weights.
        
        For local backend, delegates to TrainingManager.
        For docker backend, returns a standard path.
        
        Returns
        -------
        Path
            Path to training outputs directory
        """
        try:
            from physical_ai_server.training.training_manager import TrainingManager
            return TrainingManager.get_weight_save_root_path()
        except ImportError:
            # Fallback for docker-only mode
            from pathlib import Path
            return Path.home() / '.cache' / 'lerobot' / 'outputs' / 'train'

    def get_current_training_status(self) -> TrainingStatus:
        """
        Get current training status.
        
        Returns
        -------
        TrainingStatus
            Current training status message
        """
        if self._manager is None:
            status = TrainingStatus()
            status.training_info = self._training_info
            status.current_step = 0
            status.current_loss = float('nan')
            return status
        
        return self._manager.get_current_training_status()

    def train(self):
        """
        Execute training.
        
        For local backend: Calls TrainingManager.train()
        For docker backend: Sends training command via Zenoh
        
        Returns
        -------
        For docker backend, returns LeRobotResponse.
        For local backend, returns None (blocking call).
        """
        if self._manager is None:
            raise RuntimeError("Training manager not initialized")
        
        # Sync properties to manager
        self._manager.training_info = self._training_info
        self._manager.resume = self._resume
        self._manager.resume_model_path = self._resume_model_path
        
        if self._backend == TrainingBackend.LOCAL:
            # Local manager uses stop_event from adapter
            self._manager.stop_event = self.stop_event
            return self._manager.train()
        else:
            # Docker backend returns response
            return self._manager.train()

    def stop(self):
        """
        Stop training.
        
        For local backend: Sets stop_event
        For docker backend: Sends stop command via Zenoh
        """
        self.stop_event.set()
        
        if self._backend == TrainingBackend.DOCKER and self._manager:
            return self._manager.stop()

    def connect(self) -> bool:
        """
        Connect to backend (only for docker mode).
        
        Returns
        -------
        bool
            True if connected successfully
        """
        if self._backend == TrainingBackend.DOCKER and self._manager:
            return self._manager.connect()
        return True  # Local mode doesn't need connection

    def disconnect(self):
        """Disconnect from backend (only for docker mode)."""
        if self._backend == TrainingBackend.DOCKER and self._manager:
            self._manager.disconnect()

    def set_status_callback(self, callback: Callable):
        """
        Set callback for status updates (only for docker mode).
        
        Parameters
        ----------
        callback : Callable
            Function to call on status updates
        """
        if self._backend == TrainingBackend.DOCKER and self._manager:
            self._manager.set_status_callback(callback)

    def is_docker_mode(self) -> bool:
        """Check if running in Docker mode."""
        return self._backend == TrainingBackend.DOCKER

    def is_local_mode(self) -> bool:
        """Check if running in local mode."""
        return self._backend == TrainingBackend.LOCAL
