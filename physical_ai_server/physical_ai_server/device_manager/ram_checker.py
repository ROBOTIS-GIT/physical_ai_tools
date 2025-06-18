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

from typing import Tuple

import psutil


class RAMChecker:

    @staticmethod
    def get_ram_gb() -> Tuple[float, float]:
        try:
            memory = psutil.virtual_memory()
            total_gb = memory.total / (1024 ** 3)
            used_gb = memory.used / (1024 ** 3)
            return total_gb, used_gb
        except Exception:
            return 0.0, 0.0

    @staticmethod
    def get_free_ram_gb() -> float:
        try:
            memory = psutil.virtual_memory()
            free_gb = memory.available / (1024 ** 3)
            return free_gb
        except Exception:
            return 0.0
