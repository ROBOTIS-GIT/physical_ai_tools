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
Inference Manager Adapter

Provides a unified interface for inference that can use either:
- Local execution (direct lerobot import)
- Docker execution (via Zenoh communication)

Usage:
    # Local mode (default, requires lerobot installed)
    adapter = InferenceManagerAdapter(backend='local')
    
    # Docker mode (requires LeRobot Docker container running)
    adapter = InferenceManagerAdapter(backend='docker')
"""

from enum import Enum
from typing import Callable, Dict, List, Optional, Union

import numpy as np


class InferenceBackend(Enum):
    """Backend execution mode for inference."""
    LOCAL = 'local'
    DOCKER = 'docker'


class InferenceManagerAdapter:
    """
    Adapter that provides unified interface for both local and Docker-based inference.
    
    This adapter allows seamless switching between:
    - Local execution: Uses InferenceManager which imports lerobot directly
    - Docker execution: Uses ZenohInferenceManager which communicates via Zenoh
    
    Attributes
    ----------
    backend : InferenceBackend
        The execution backend (LOCAL or DOCKER)
    device : str
        Device for inference (cuda/cpu)
    policy_type : str
        Type of policy loaded
    policy_path : str
        Path to the policy
    """
    
    SUPPORTED_POLICIES = [
        'tdmpc', 'diffusion', 'act', 'vqbet', 'pi0', 'pi0fast', 'smolvla'
    ]

    def __init__(
            self,
            backend: Union[str, InferenceBackend] = 'local',
            device: str = 'cuda'):
        """
        Initialize inference manager adapter.
        
        Parameters
        ----------
        backend : str or InferenceBackend
            Execution backend: 'local' or 'docker'
        device : str
            Device for inference (cuda/cpu), only used for local backend
        """
        if isinstance(backend, str):
            backend = InferenceBackend(backend.lower())
        
        self._backend = backend
        self._device = device
        self._manager = None
        self._initialized = False
        
        # Common properties
        self._policy_type: Optional[str] = None
        self._policy_path: Optional[str] = None
        
        self._init_backend()

    def _init_backend(self):
        """Initialize the appropriate backend manager."""
        if self._backend == InferenceBackend.LOCAL:
            self._init_local_backend()
        else:
            self._init_docker_backend()

    def _init_local_backend(self):
        """Initialize local inference manager."""
        try:
            from physical_ai_server.inference.inference_manager import InferenceManager
            self._manager = InferenceManager(device=self._device)
            self._initialized = True
        except ImportError as e:
            raise ImportError(
                f"Failed to import local InferenceManager. "
                f"Ensure lerobot is installed: {e}"
            )

    def _init_docker_backend(self):
        """Initialize Docker/Zenoh inference manager."""
        try:
            from physical_ai_server.inference.zenoh_inference_manager import ZenohInferenceManager
            self._manager = ZenohInferenceManager()
            self._initialized = True
        except ImportError as e:
            raise ImportError(
                f"Failed to import ZenohInferenceManager: {e}"
            )

    @property
    def backend(self) -> InferenceBackend:
        """Get current backend."""
        return self._backend

    @property
    def device(self) -> str:
        """Get device."""
        return self._device

    @property
    def policy_type(self) -> Optional[str]:
        """Get policy type."""
        if self._manager:
            return self._manager.policy_type
        return self._policy_type

    @property
    def policy_path(self) -> Optional[str]:
        """Get policy path."""
        if self._manager:
            return self._manager.policy_path
        return self._policy_path

    @property
    def policy(self):
        """
        Get loaded policy (local mode only).
        
        Returns None for docker mode as policy lives in container.
        """
        if self._backend == InferenceBackend.LOCAL and self._manager:
            return self._manager.policy
        return None

    def validate_policy(self, policy_path: str) -> tuple[bool, str]:
        """
        Validate policy at given path.
        
        Parameters
        ----------
        policy_path : str
            Path to the policy directory
            
        Returns
        -------
        tuple[bool, str]
            (success, message)
        """
        if self._manager is None:
            return False, "Inference manager not initialized"
        
        return self._manager.validate_policy(policy_path)

    def load_policy(self) -> bool:
        """
        Load the policy.
        
        For local backend: Loads policy into memory
        For docker backend: Sends load command to container
        
        Returns
        -------
        bool
            True if successful
        """
        if self._manager is None:
            return False
        
        result = self._manager.load_policy()
        
        # Handle different return types
        if self._backend == InferenceBackend.DOCKER:
            # ZenohInferenceManager returns LeRobotResponse
            return result.success if hasattr(result, 'success') else bool(result)
        else:
            # Local InferenceManager returns bool
            return bool(result)

    def clear_policy(self):
        """
        Clear/unload the policy.
        
        For local backend: Deletes policy from memory
        For docker backend: Sends unload command to container
        """
        if self._manager:
            return self._manager.clear_policy()

    def predict(
            self,
            images: Dict[str, np.ndarray],
            state: List[float],
            task_instruction: str = None) -> List:
        """
        Run inference to get action.
        
        Parameters
        ----------
        images : dict
            Dictionary of camera images {name: np.ndarray}
        state : list
            Robot state (joint positions)
        task_instruction : str, optional
            Task instruction for VLA models
            
        Returns
        -------
        list
            Action (joint positions/velocities)
        """
        if self._manager is None:
            raise RuntimeError("Inference manager not initialized")
        
        if self._backend == InferenceBackend.LOCAL:
            return self._manager.predict(images, state, task_instruction)
        else:
            # Docker mode - need to handle async inference
            # For now, raise not implemented as real-time inference
            # via Docker requires the action subscription pattern
            raise NotImplementedError(
                "Direct predict() not supported in Docker mode. "
                "Use start_inference() with action callback instead."
            )

    def get_policy_config(self):
        """
        Get policy configuration.
        
        Returns
        -------
        Policy config object (local) or dict (docker)
        """
        if self._manager is None:
            return None
        
        if self._backend == InferenceBackend.LOCAL:
            return self._manager.get_policy_config()
        else:
            # Docker mode returns via Zenoh
            response = self._manager.get_status()
            return response.data if hasattr(response, 'data') else {}

    # Docker-specific methods
    def connect(self) -> bool:
        """
        Connect to backend (only for docker mode).
        
        Returns
        -------
        bool
            True if connected successfully
        """
        if self._backend == InferenceBackend.DOCKER and self._manager:
            return self._manager.connect()
        return True  # Local mode doesn't need connection

    def disconnect(self):
        """Disconnect from backend (only for docker mode)."""
        if self._backend == InferenceBackend.DOCKER and self._manager:
            self._manager.disconnect()

    def start_inference(
            self,
            model_path: str = None,
            env_name: str = None,
            num_episodes: int = None):
        """
        Start continuous inference (docker mode).
        
        Parameters
        ----------
        model_path : str, optional
            Path to the model
        env_name : str, optional
            Environment name
        num_episodes : int, optional
            Number of episodes to run
            
        Returns
        -------
        LeRobotResponse for docker mode
        """
        if self._backend == InferenceBackend.DOCKER and self._manager:
            return self._manager.start_inference(model_path, env_name, num_episodes)
        raise NotImplementedError("start_inference() only available in Docker mode")

    def stop_inference(self):
        """
        Stop continuous inference (docker mode).
        
        Returns
        -------
        LeRobotResponse for docker mode
        """
        if self._backend == InferenceBackend.DOCKER and self._manager:
            return self._manager.stop_inference()
        raise NotImplementedError("stop_inference() only available in Docker mode")

    def set_action_callback(self, callback: Callable[[Dict], None]):
        """
        Set callback for action updates (docker mode only).
        
        Parameters
        ----------
        callback : Callable
            Function to call when action is received
        """
        if self._backend == InferenceBackend.DOCKER and self._manager:
            self._manager.set_action_callback(callback)

    def set_status_callback(self, callback: Callable[[Dict], None]):
        """
        Set callback for status updates (docker mode only).
        
        Parameters
        ----------
        callback : Callable
            Function to call on status updates
        """
        if self._backend == InferenceBackend.DOCKER and self._manager:
            self._manager.set_status_callback(callback)

    def list_models(self):
        """
        List available models (docker mode).
        
        Returns
        -------
        LeRobotResponse with model list
        """
        if self._backend == InferenceBackend.DOCKER and self._manager:
            return self._manager.list_models()
        raise NotImplementedError("list_models() only available in Docker mode")

    # Static methods
    @staticmethod
    def get_available_policies() -> List[str]:
        """Get list of supported policy types."""
        return InferenceManagerAdapter.SUPPORTED_POLICIES

    @staticmethod
    def get_saved_policies() -> tuple[List[str], List[str]]:
        """
        Get saved policies from HuggingFace cache.
        
        Returns
        -------
        tuple
            (policy_paths, policy_types)
        """
        try:
            from physical_ai_server.inference.inference_manager import InferenceManager
            return InferenceManager.get_saved_policies()
        except ImportError:
            # Fallback for docker-only mode
            return [], []

    def is_docker_mode(self) -> bool:
        """Check if running in Docker mode."""
        return self._backend == InferenceBackend.DOCKER

    def is_local_mode(self) -> bool:
        """Check if running in local mode."""
        return self._backend == InferenceBackend.LOCAL
