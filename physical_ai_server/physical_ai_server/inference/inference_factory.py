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

from typing import Dict, Optional, Type

from .inference_base import InferenceBase
from .lerobot_inference import LeRobotInference
from .physical_intelligence_inference import PhysicalIntelligenceInference
from .groot_inference import GrootInference


# Factory class for creating inference managers with unified interface
class InferenceFactory:

    # Registry of available inference managers
    _registry: Dict[str, Type[InferenceBase]] = {
        'lerobot': LeRobotInference,
        'physical_intelligence': PhysicalIntelligenceInference,
        'pi': PhysicalIntelligenceInference,  # Alias
        'groot': GrootInference,
        'nvidia': GrootInference,  # Alias
        'isaac': GrootInference,   # Alias
    }

    @classmethod
    def create_inference_manager(
        cls,
        framework: str,
        device: str = 'cuda'
    ) -> InferenceBase:
        # Create inference manager for specified framework
        framework_lower = framework.lower()
        
        if framework_lower not in cls._registry:
            available_frameworks = list(cls._registry.keys())
            raise ValueError(
                f"Unsupported framework '{framework}'. "
                f"Available frameworks: {available_frameworks}"
            )
        
        inference_class = cls._registry[framework_lower]
        return inference_class(device=device)

    @classmethod
    def get_available_frameworks(cls) -> list[str]:
        # Get list of available framework names
        return list(cls._registry.keys())

    @classmethod
    def register_inference_manager(
        cls,
        framework: str,
        inference_class: Type[InferenceBase]
    ) -> None:
        # Register new inference manager class
        if not issubclass(inference_class, InferenceBase):
            raise TypeError(
                f"inference_class must be a subclass of InferenceBase, "
                f"got {inference_class}"
            )
        
        cls._registry[framework.lower()] = inference_class

    @classmethod
    def detect_framework_from_path(cls, policy_path: str) -> Optional[str]:
        # Attempt to detect framework type from policy path
        # Tests validation with each available framework
        for framework_name, inference_class in cls._registry.items():
            try:
                # Create temporary instance to test validation
                temp_manager = inference_class()
                is_valid, _ = temp_manager.validate_policy(policy_path)
                if is_valid:
                    return framework_name
            except Exception:
                # If validation fails due to missing dependencies, continue
                continue
        
        return None

    @classmethod
    def get_all_available_policies(cls) -> Dict[str, list[str]]:
        # Get all available policy types from all registered frameworks
        all_policies = {}
        
        for framework_name, inference_class in cls._registry.items():
            try:
                policies = inference_class.get_available_policies()
                all_policies[framework_name] = policies
            except Exception as e:
                # If getting policies fails, set empty list and continue
                all_policies[framework_name] = []
                print(f"Warning: Could not get policies for {framework_name}: {e}")
        
        return all_policies

    @classmethod
    def get_all_saved_policies(cls) -> Dict[str, tuple[list[str], list[str]]]:
        # Get all saved policies from all registered frameworks
        # Returns: Dict mapping framework names to (policy_paths, policy_types) tuples
        all_saved_policies = {}
        
        for framework_name, inference_class in cls._registry.items():
            try:
                saved_policies = inference_class.get_saved_policies()
                all_saved_policies[framework_name] = saved_policies
            except Exception as e:
                # If getting saved policies fails, set empty lists and continue
                all_saved_policies[framework_name] = ([], [])
                print(f"Warning: Could not get saved policies for {framework_name}: {e}")
        
        return all_saved_policies


# Convenience function for backward compatibility
def create_inference_manager(framework: str = 'lerobot', device: str = 'cuda') -> InferenceBase:
    # Convenience function to create inference manager
    return InferenceFactory.create_inference_manager(framework, device)
