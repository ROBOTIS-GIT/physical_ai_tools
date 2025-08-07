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

from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple, Any
import numpy as np


# Abstract base class for inference managers supporting different AI frameworks
class InferenceBase(ABC):

    def __init__(self, device: str = 'cuda'):
        # Initialize inference base with device ('cuda' or 'cpu')
        self.device = device
        self.policy_type = None
        self.policy_path = None
        self.policy = None

    @abstractmethod
    def validate_policy(self, policy_path: str) -> Tuple[bool, str]:
        # Validate if policy at given path is valid and supported
        # Returns: (is_valid, message)
        pass

    @abstractmethod
    def load_policy(self) -> bool:
        # Load policy from validated path
        # Returns: True if successful, False otherwise
        pass

    @abstractmethod
    def clear_policy(self) -> None:
        # Clear loaded policy from memory
        pass

    @abstractmethod
    def get_policy_config(self) -> Any:
        # Get configuration of loaded policy
        pass

    @abstractmethod
    def predict(
        self,
        images: Dict[str, np.ndarray],
        state: List[float],
        task_instruction: Optional[str] = None
    ) -> np.ndarray:
        # Perform single-step inference
        # Args: images (camera dict), state (robot state), task_instruction (optional)
        # Returns: predicted action as list
        pass

    @abstractmethod
    def predict_chunk(
        self,
        images: Dict[str, np.ndarray],
        state: List[float],
        task_instruction: Optional[str] = None
    ) -> np.ndarray:
        # Perform chunk-based inference (multiple steps)
        # Args: images (camera dict), state (robot state), task_instruction (optional)
        # Returns: predicted action chunk as numpy array
        pass

    @classmethod
    @abstractmethod
    def get_available_policies(cls) -> List[str]:
        # Get list of available policy types for this inference manager
        pass

    @abstractmethod
    def _preprocess(
        self,
        images: Dict[str, np.ndarray],
        state: List[float],
        task_instruction: Optional[str] = None
    ) -> Any:
        # Preprocess inputs for specific policy framework
        # Returns: framework-specific observation format
        pass

    def is_policy_loaded(self) -> bool:
        # Check if a policy is currently loaded
        return self.policy is not None

    def get_policy_type(self) -> Optional[str]:
        # Get type of currently loaded policy
        return self.policy_type

    def get_policy_path(self) -> Optional[str]:
        # Get path of currently loaded policy
        return self.policy_path

    def get_device(self) -> str:
        # Get device being used for inference
        return self.device
