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


class InferenceFactory:

    # Registry of available inference managers
    _registry: Dict[str, Type[InferenceBase]] = {
        'lerobot': LeRobotInference,
        'physical_intelligence': PhysicalIntelligenceInference,
        'groot': GrootInference
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
