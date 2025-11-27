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

"""Action to update task instruction during inference."""

from typing import TYPE_CHECKING

from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from physical_ai_bt.blackboard import Blackboard
from physical_ai_interfaces.srv import SendCommand
from physical_ai_interfaces.msg import TaskInfo

if TYPE_CHECKING:
    from rclpy.node import Node


class UpdateTaskInstruction(BaseAction):
    """Action to update language instruction during active inference session."""

    def __init__(self, node: 'Node', instruction: str = "", prefix: str = "Pick up the "):
        """
        Initialize update task instruction action.

        Args:
            node: ROS2 node reference
            instruction: Fixed instruction (ignored if blackboard has task_instruction)
            prefix: Prefix to prepend to blackboard value (default: "Pick up the ")
        """
        super().__init__(node, name="UpdateTaskInstruction")

        self.instruction = instruction  # Fallback if blackboard empty
        self.prefix = prefix
        self.blackboard = Blackboard()

        # Service client for sending command to AI Server
        self.command_client = self.node.create_client(
            SendCommand,
            '/ai_server/task/command'
        )

        # State tracking
        self.request_sent = False
        self.future = None

    def tick(self) -> NodeStatus:
        """Execute update task instruction action."""
        # First tick: send request
        if not self.request_sent:
            if not self.command_client.wait_for_service(timeout_sec=1.0):
                self.log_error("AI Server command service not available")
                return NodeStatus.FAILURE

            # Build instruction from blackboard or use fallback
            task_obj = self.blackboard.get('task_instruction', '')

            if task_obj:
                # Build instruction: prefix + blackboard value
                final_instruction = self.prefix + task_obj
                self.log_info(f"Built instruction from blackboard: '{final_instruction}'")
            else:
                # Use fallback instruction
                final_instruction = self.instruction
                self.log_info(f"Using fallback instruction: '{final_instruction}'")

            # Validate instruction
            if not final_instruction or final_instruction.strip() == "":
                self.log_error("Instruction cannot be empty")
                return NodeStatus.FAILURE

            # Create request
            request = SendCommand.Request()
            request.command = SendCommand.Request.UPDATE_TASK_INSTRUCTION
            request.task_info = TaskInfo()
            request.task_info.task_instruction = [final_instruction]

            try:
                self.log_info(f"Updating task instruction: '{final_instruction}'")
                self.future = self.command_client.call_async(request)
                self.request_sent = True
                return NodeStatus.RUNNING
            except Exception as e:
                self.log_error(f"Exception while sending update request: {str(e)}")
                return NodeStatus.FAILURE

        # Subsequent ticks: check if response received
        if self.future is not None and self.future.done():
            try:
                response = self.future.result()
                if response.success:
                    self.log_info(f"Task instruction updated successfully: '{self.instruction}'")
                    return NodeStatus.SUCCESS
                else:
                    self.log_error(f"Failed to update task instruction: {response.message}")
                    return NodeStatus.FAILURE
            except Exception as e:
                self.log_error(f"Exception while getting update response: {str(e)}")
                return NodeStatus.FAILURE

        # Still waiting for response
        return NodeStatus.RUNNING

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.request_sent = False
        self.future = None
