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
# Author: GitHub Copilot

"""Recovery control node for Behavior Tree."""

from typing import TYPE_CHECKING
from physical_ai_bt.actions.base_action import BaseControl, NodeStatus

if TYPE_CHECKING:
    from rclpy.node import Node

class Recovery(BaseControl):
    """
    Recovery control node.

    Executes primary child. If it fails, executes recovery child.
    Returns SUCCESS if either child succeeds.
    Returns FAILURE if both fail.
    Returns RUNNING if current child is still running.
    """

    def __init__(self, node: 'Node', name: str = "Recovery"):
        """
        Initialize recovery node.
        Args:
            node: ROS2 node reference
            name: Node name
        """
        super().__init__(node, name=name)
        self.children = []

    def add_child(self, child):
        self.children.append(child)

    def tick(self) -> NodeStatus:
        if len(self.children) < 2:
            self.log_error("Recovery node requires two children: [primary, recovery]")
            return NodeStatus.FAILURE

        primary = self.children[0]
        recovery = self.children[1]

        status = primary.tick()
        if status == NodeStatus.SUCCESS:
            return NodeStatus.SUCCESS
        elif status == NodeStatus.FAILURE:
            status = recovery.tick()
            return status
        else:
            return NodeStatus.RUNNING

    def reset(self):
        for child in self.children:
            child.reset()
