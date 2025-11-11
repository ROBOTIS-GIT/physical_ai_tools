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
from physical_ai_interfaces.srv import SendCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node


class InferenceAction(BaseAction):
    """Action that monitors and manages VLA inference from AI Server."""

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

        # Service client for AI Server
        self.command_client = self.node.create_client(
            SendCommand,
            '/task/command'
        )

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

    def _status_callback(self, msg: TaskStatus):
        """Callback for AI Server status messages."""
        # Check if inference is running (phase == INFERENCING)
        if msg.phase == TaskStatus.INFERENCING and not self.inference_detected:
            self.inference_detected = True
            self.inference_start_time = time.time()
            self.log_info("Detected inference started from AI Server")

    def tick(self) -> NodeStatus:
        """Execute one tick of inference action."""
        current_time = time.time()

        # Step 1: Wait for inference to start
        if self.waiting_for_inference:
            if not self.inference_detected:
                # Still waiting for AI Server to start inference
                return NodeStatus.RUNNING

            # Inference detected, start timer
            self.waiting_for_inference = False
            self.log_info(f"Inference monitoring started, will run for {self.timeout}s")
            return NodeStatus.RUNNING

        # Step 2: Monitor inference duration
        if self.inference_start_time is None:
            self.log_error("Inference start time not set")
            return NodeStatus.FAILURE

        elapsed = current_time - self.inference_start_time

        if elapsed >= self.timeout:
            # Timeout reached, stop inference
            self._stop_inference()
            
            # Wait for AI Server to fully stop publishing
            if not hasattr(self, '_stop_time'):
                self._stop_time = current_time
                return NodeStatus.RUNNING

            # Wait 2 seconds after sending FINISH for AI Server cleanup
            if current_time - self._stop_time < 2.0:
                return NodeStatus.RUNNING

            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING

    def _stop_inference(self):
        """Stop inference by calling AI Server service."""
        if not self.command_client.wait_for_service(timeout_sec=1.0):
            self.log_warn("AI Server service not available")
            return

        request = SendCommand.Request()
        request.command = SendCommand.Request.FINISH

        try:
            self.command_client.call_async(request)
        except Exception as e:
            self.log_warn(f"Failed to send FINISH command: {str(e)}")

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.inference_detected = False
        self.inference_start_time = None
        self.waiting_for_inference = True
        if hasattr(self, '_stop_time'):
            delattr(self, '_stop_time')
