#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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

"""Action node that keeps inference running for a specified duration."""

import time
from typing import TYPE_CHECKING

from physical_ai_bt.actions.base_action import BaseAction
from physical_ai_bt.bt_core import NodeStatus

if TYPE_CHECKING:
    from rclpy.node import Node


class InferenceForDuration(BaseAction):
    """Wait for a specified duration while inference is running, then succeed."""

    def __init__(
        self,
        node: 'Node',
        duration: float = 5.0,
    ):
        """Initialize the InferenceForDuration action."""
        super().__init__(node, name='InferenceForDuration')
        self.duration = duration
        self._start_time = None

    def tick(self) -> NodeStatus:
        """Return RUNNING until duration elapses, then SUCCESS."""
        if self._start_time is None:
            self._start_time = time.monotonic()
            self.log_info(
                f'Inference running for {self.duration}s'
            )

        elapsed = time.monotonic() - self._start_time
        if elapsed >= self.duration:
            self.log_info(
                f'Inference duration complete ({self.duration}s)'
            )
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING

    def reset(self):
        """Reset the action to its initial state."""
        super().reset()
        self._start_time = None
