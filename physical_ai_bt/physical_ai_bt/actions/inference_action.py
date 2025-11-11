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
        policy_path: str,
        task_instruction: str,
        timeout: float = 5.0
    ):
        """
        Initialize inference action.

        Args:
            node: ROS2 node reference
            policy_path: Path to the policy model (not used, for future)
            task_instruction: Instruction for the task (not used, for future)
            timeout: Duration to keep inference running
        """
        super().__init__(node, name="InferenceAction")
        self.policy_path = policy_path
        self.task_instruction = task_instruction
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
        self.log_info("Subscribed to /task/status topic")

        # State variables
        self.inference_detected = False
        self.inference_start_time = None
        self.waiting_for_inference = True
        self.log_info(f"InferenceAction initialized (timeout={timeout}s)")

    def _status_callback(self, msg: TaskStatus):
        """Callback for AI Server status messages."""
        # Debug: Log all status messages
        self.log_info(f"Status callback: phase={msg.phase} (INFERENCING={TaskStatus.INFERENCING})")

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
                # Log periodically
                if not hasattr(self, '_last_wait_log') or (current_time - self._last_wait_log) > 2.0:
                    self.log_info("Waiting for AI Server to start inference...")
                    self._last_wait_log = current_time
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
            self.log_info(f"Inference duration ({self.timeout}s) reached, stopping inference")
            self._stop_inference()
            # Wait a bit for AI Server to fully stop publishing
            if not hasattr(self, '_stop_time'):
                self._stop_time = current_time
                return NodeStatus.RUNNING

            # Wait 0.5 seconds after sending FINISH
            if current_time - self._stop_time < 0.5:
                return NodeStatus.RUNNING

            return NodeStatus.SUCCESS

        # Still running
        if int(elapsed) != int(elapsed - 0.1):  # Log approximately once per second
            self.log_info(f"Inference running... ({elapsed:.1f}s / {self.timeout}s)")

        return NodeStatus.RUNNING

    def _stop_inference(self):
        """Stop inference by calling AI Server service."""
        if not self.command_client.wait_for_service(timeout_sec=1.0):
            self.log_warn("AI Server service not available for FINISH command")
            return

        request = SendCommand.Request()
        request.command = SendCommand.Request.FINISH

        try:
            future = self.command_client.call_async(request)
            self.log_info("Sent FINISH command to AI Server")
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
