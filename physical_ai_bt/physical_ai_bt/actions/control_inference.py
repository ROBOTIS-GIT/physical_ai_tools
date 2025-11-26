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

"""Actions to control action publishing from AI Server."""

from typing import TYPE_CHECKING

from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from physical_ai_interfaces.srv import SendCommand
from physical_ai_interfaces.msg import TaskInfo

if TYPE_CHECKING:
    from rclpy.node import Node

class PauseInference(BaseAction):
    """Action to pause inference and action publishing from AI Server."""

    def __init__(self, node: 'Node'):
        """
        Initialize pause inference action.

        Args:
            node: ROS2 node reference
        """
        super().__init__(node, name="PauseInference")

        # Service client for controlling action publish (direct to AI Server)
        self.control_client = self.node.create_client(
            SendCommand,
            '/ai_server/task/command'
        )

        # State tracking
        self.request_sent = False
        self.future = None

    def tick(self) -> NodeStatus:
        """Execute pause inference action."""
        # First tick: send request
        if not self.request_sent:
            if not self.control_client.wait_for_service(timeout_sec=1.0):
                self.log_error("Control action publish service not available")
                return NodeStatus.FAILURE

            request = SendCommand.Request()
            request.command = SendCommand.Request.STOP
            request.task_info = TaskInfo()

            try:
                self.future = self.control_client.call_async(request)
                self.request_sent = True
                return NodeStatus.RUNNING
            except Exception as e:
                self.log_error(f"Exception while sending pause request: {str(e)}")
                return NodeStatus.FAILURE

        # Subsequent ticks: check if response received
        if self.future is not None and self.future.done():
            try:
                response = self.future.result()
                if response.success:
                    self.log_info("Inference stopped (stop_inference=True)")
                    return NodeStatus.SUCCESS
                else:
                    self.log_error(f"Failed to stop inference: {response.message}")
                    return NodeStatus.FAILURE
            except Exception as e:
                self.log_error(f"Exception while getting pause response: {str(e)}")
                return NodeStatus.FAILURE

        # Still waiting for response
        return NodeStatus.RUNNING

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.request_sent = False
        self.future = None


class ResumeInference(BaseAction):
    """Action to resume inference and action publishing from AI Server."""

    def __init__(self, node: 'Node'):
        """
        Initialize resume inference action.

        Args:
            node: ROS2 node reference
        """
        super().__init__(node, name="ResumeInference")

        # Service client for controlling action publish (direct to AI Server)
        self.control_client = self.node.create_client(
            SendCommand,
            '/ai_server/task/command'
        )

        # State tracking
        self.request_sent = False
        self.future = None

    def tick(self) -> NodeStatus:
        """Execute resume inference action."""
        # First tick: send request
        if not self.request_sent:
            if not self.control_client.wait_for_service(timeout_sec=1.0):
                self.log_error("Control action publish service not available")
                return NodeStatus.FAILURE

            request = SendCommand.Request()
            request.command = SendCommand.Request.START_INFERENCE
            request.task_info = TaskInfo()

            try:
                self.future = self.control_client.call_async(request)
                self.request_sent = True
                return NodeStatus.RUNNING
            except Exception as e:
                self.log_error(f"Exception while sending resume request: {str(e)}")
                return NodeStatus.FAILURE

        # Subsequent ticks: check if response received
        if self.future is not None and self.future.done():
            try:
                response = self.future.result()
                if response.success:
                    self.log_info("Inference resumed (stop_inference=False)")
                    return NodeStatus.SUCCESS
                else:
                    self.log_error(f"Failed to resume inference: {response.message}")
                    return NodeStatus.FAILURE
            except Exception as e:
                self.log_error(f"Exception while getting resume response: {str(e)}")
                return NodeStatus.FAILURE

        # Still waiting for response
        return NodeStatus.RUNNING

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.request_sent = False
        self.future = None
