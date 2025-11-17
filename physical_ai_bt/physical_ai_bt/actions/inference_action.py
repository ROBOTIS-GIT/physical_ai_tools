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

"""Inference action using VLA model via AI Server."""

import time
from typing import TYPE_CHECKING

from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from physical_ai_interfaces.msg import TaskStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node


class InferenceAction(BaseAction):

    def __init__(
        self,
        node: 'Node',
        timeout: float = 5.0
    ):
        """
        Initialize inference action.

        Args:
            node: ROS2 node reference
            timeout: Duration to keep inference running
        """
        super().__init__(node, name="InferenceAction")
        self.timeout = timeout

        # Subscriber to monitor AI Server status
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.status_sub = self.node.create_subscription(
            TaskStatus,
            '/task/status',
            self._status_callback,
            qos_profile
        )

        # State variables
        self.inference_detected = False
        self.inference_start_time = None
        self.waiting_for_inference = True
        self.latest_status = None
        self.execution_active = False  # True only during active tick() execution

    def _status_callback(self, msg: TaskStatus):
        """Callback for AI Server status messages."""
        self.latest_status = msg

        # Only process INFERENCING callbacks when execution is active AND waiting
        # This prevents stale callbacks after SUCCESS from setting start_time
        if not self.execution_active or not self.waiting_for_inference:
            return

        # Check if inference is running and set start time once
        if msg.phase == TaskStatus.INFERENCING and not self.inference_detected:
            if self.inference_start_time is None:
                self.inference_start_time = time.time()
            self.inference_detected = True

    def tick(self) -> NodeStatus:
        """Execute one tick of inference action."""
        self.execution_active = True
        current_time = time.time()

        # Step 1: Wait for inference to start
        if self.waiting_for_inference:
            if not self.inference_detected:
                return NodeStatus.RUNNING

            # Inference detected, start monitoring
            self.waiting_for_inference = False
            return NodeStatus.RUNNING

        # Step 2: Monitor inference duration
        if self.inference_start_time is None:
            self.log_error("Inference start time not set")
            return NodeStatus.FAILURE

        elapsed = current_time - self.inference_start_time

        if elapsed >= self.timeout:
            # Timeout reached, complete without stopping inference
            # Reset state for next execution
            self.inference_detected = False
            self.inference_start_time = None
            self.waiting_for_inference = True
            self.execution_active = False

            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.inference_detected = False
        self.inference_start_time = None
        self.waiting_for_inference = True
        self.execution_active = False
