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

from .inference_base import InferenceBase
from .lerobot_inference import LeRobotInference
# from .physical_intelligence_inference import PhysicalIntelligenceInference
# from .groot_inference import GrootInference
from .inference_factory import InferenceFactory, create_inference_manager

# For backward compatibility, import old InferenceManager as alias
from .lerobot_inference import LeRobotInference as InferenceManager

__all__ = [
    'InferenceBase',
    'LeRobotInference', 
    # 'PhysicalIntelligenceInference',
    # 'GrootInference',
    'InferenceFactory',
    'create_inference_manager',
    'InferenceManager',  # Backward compatibility
]
