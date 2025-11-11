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

"""Sequence control node."""

from typing import TYPE_CHECKING

from physical_ai_bt.actions.base_action import BaseControl, NodeStatus

if TYPE_CHECKING:
    from rclpy.node import Node


class Sequence(BaseControl):
    """
    Sequence control node.
    
    Executes children in order until one fails or all succeed.
    Returns SUCCESS if all children succeed.
    Returns FAILURE if any child fails.
    Returns RUNNING if current child is still running.
    """

    def __init__(self, node: 'Node', name: str = "Sequence"):
        """
        Initialize sequence node.

        Args:
            node: ROS2 node reference
            name: Name of the sequence node
        """
        super().__init__(node, name)
        self.current_child_index = 0

    def tick(self) -> NodeStatus:
        """Execute sequence logic."""
        while self.current_child_index < len(self.children):
            current_child = self.children[self.current_child_index]
            status = current_child.tick()

            if status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            elif status == NodeStatus.FAILURE:
                self.log_warn(f"Child {current_child.name} failed")
                return NodeStatus.FAILURE
            else:  # SUCCESS
                self.log_info(f"Child {current_child.name} succeeded")
                self.current_child_index += 1

        # All children succeeded
        self.log_info("All children succeeded")
        return NodeStatus.SUCCESS

    def reset(self):
        """Reset sequence and all children."""
        super().reset()
        self.current_child_index = 0
