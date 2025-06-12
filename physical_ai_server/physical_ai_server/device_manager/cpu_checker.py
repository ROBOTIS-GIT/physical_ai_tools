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

import psutil


class CPUChecker:

    @staticmethod
    def get_cpu_usage() -> float:
        """
        Returns the current overall CPU usage percentage.

        This method retrieves the current CPU utilization across all cores
        as a percentage value. The measurement is taken over a brief interval
        to provide an accurate reading.

        Returns:
            float: CPU usage percentage (0.0 to 100.0). Returns 0.0 if an error occurs.
        """
        try:
            # Get CPU usage percentage with a 1 second interval for accuracy
            cpu_percent = psutil.cpu_percent(interval=1)
            return cpu_percent
        except Exception:
            return 0.0
