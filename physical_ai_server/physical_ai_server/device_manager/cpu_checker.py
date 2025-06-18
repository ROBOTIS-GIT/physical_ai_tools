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

from collections import deque

import psutil


class CPUChecker:

    def __init__(self, window_size: int = 30):
        self.window_size = window_size
        self.cpu_samples = deque(maxlen=window_size)
        psutil.cpu_percent(interval=None)

    def get_cpu_usage(self) -> float:
        try:
            current_cpu = psutil.cpu_percent(interval=None)
            self.cpu_samples.append(current_cpu)
            if len(self.cpu_samples) > 0:
                return sum(self.cpu_samples) / len(self.cpu_samples)
            else:
                return 0.0

        except Exception:
            return 0.0
