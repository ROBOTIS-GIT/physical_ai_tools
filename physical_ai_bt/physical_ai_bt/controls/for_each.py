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

"""ForEach control node - iterates children for each element in a list."""

from typing import TYPE_CHECKING
from physical_ai_bt.actions.base_action import BaseControl, NodeStatus
from physical_ai_bt.blackboard import Blackboard

if TYPE_CHECKING:
    from rclpy.node import Node


class ForEach(BaseControl):
    """
    ForEach control node - iterates children once per list element.

    Reads a list from blackboard and executes children for each element.
    Sets 'current_task_index' in blackboard for children to access.

    Behavior:
    - On first tick: Read list from blackboard, start at index 0
    - Execute all children as Sequence
    - On children SUCCESS: increment index, reset children, repeat
    - On children FAILURE: return FAILURE (stop iteration)
    - When index reaches list length: return SUCCESS
    - On children RUNNING: return RUNNING
    """

    def __init__(self, node: 'Node', name: str = "ForEach",
                 list_key: str = "task_instruction_list"):
        """
        Initialize ForEach control node.

        Args:
            node: ROS2 node reference
            name: Name of the control node
            list_key: Blackboard key for the list to iterate (default: "task_instruction_list")
        """
        super().__init__(node, name)
        self.list_key = list_key
        self.blackboard = Blackboard()
        self.current_index = 0
        self.task_list = []
        self.current_child_index = 0
        self.initialized = False

    def tick(self) -> NodeStatus:
        """
        Execute ForEach logic.

        Returns:
            NodeStatus: SUCCESS when all iterations complete,
                       FAILURE if any child fails,
                       RUNNING if iteration in progress
        """
        # Initialize on first tick
        if not self.initialized:
            self.task_list = self.blackboard.get(self.list_key, [])
            if not self.task_list:
                self.log_warn(f"ForEach: No list found at blackboard key '{self.list_key}'")
                return NodeStatus.FAILURE

            self.log_info(f"ForEach: Starting iteration over {len(self.task_list)} items")
            self.current_index = 0
            self.current_child_index = 0
            self.initialized = True

        # Check if all iterations complete
        if self.current_index >= len(self.task_list):
            self.log_info("ForEach: All iterations completed")
            return NodeStatus.SUCCESS

        # Set current index in blackboard for children to access
        self.blackboard.set('current_task_index', self.current_index)

        # Execute children as sequence
        while self.current_child_index < len(self.children):
            current_child = self.children[self.current_child_index]
            status = current_child.tick()

            if status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            elif status == NodeStatus.FAILURE:
                self.log_warn(f"Child {current_child.name} failed on iteration {self.current_index}")
                return NodeStatus.FAILURE
            else:  # SUCCESS
                self.log_info(f"Child {current_child.name} succeeded")
                self.current_child_index += 1

        # All children succeeded for this iteration
        self.log_info(f"Iteration {self.current_index} completed ({self.task_list[self.current_index]})")

        # Move to next iteration
        self.current_index += 1
        self.current_child_index = 0

        # Reset all children for next iteration
        for child in self.children:
            child.reset()

        # Continue to next iteration (will be processed on next tick)
        return NodeStatus.RUNNING

    def reset(self):
        """Reset ForEach and all children."""
        super().reset()
        self.current_index = 0
        self.current_child_index = 0
        self.initialized = False
        self.task_list = []
