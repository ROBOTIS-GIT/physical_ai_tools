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
# Author: Seongwoo Kim

"""Timed inference action that runs for a specified duration."""

import time
from typing import TYPE_CHECKING

from physical_ai_bt.actions.base_action import NodeStatus, BaseAction

if TYPE_CHECKING:
    from rclpy.node import Node


class TimedInference(BaseAction):
    """Action that runs inference for a specified duration then completes."""

    def __init__(
        self,
        node: 'Node',
        duration: float = 20.0,
    ):
        """
        Initialize timed inference action.

        Args:
            node: ROS2 node reference
            duration: Duration in seconds to run inference (default: 20.0)
        """
        super().__init__(node, name="TimedInference")
        self.duration = duration
        self.start_time = None

    def tick(self) -> NodeStatus:
        """Execute one tick of timed inference action."""
        # First tick: record start time
        if self.start_time is None:
            self.start_time = time.time()
            self.log_info(f"Starting timed inference for {self.duration} seconds")
            return NodeStatus.RUNNING

        # Check if duration has elapsed
        elapsed = time.time() - self.start_time
        if elapsed >= self.duration:
            self.log_info(f"Timed inference completed after {elapsed:.1f} seconds")
            return NodeStatus.SUCCESS

        # Still running
        return NodeStatus.RUNNING

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.start_time = None
