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

"""Base classes for all Behavior Tree nodes."""

from enum import Enum
from typing import TYPE_CHECKING, List

if TYPE_CHECKING:
    from rclpy.node import Node


class NodeStatus(Enum):
    """Status returned by node tick() method."""
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3


class BTNode:
    """Base class for all BT nodes (actions, controls, decorators)."""

    def __init__(self, node: 'Node', name: str):
        """
        Initialize BT node.

        Args:
            node: ROS2 node reference for logging and communication
            name: Name of the node for logging
        """
        self.node = node
        self.name = name
        self.status = NodeStatus.RUNNING

    def tick(self) -> NodeStatus:
        """
        Execute one tick of the node.

        Returns:
            NodeStatus: Current status of the node
        """
        raise NotImplementedError("Subclasses must implement tick() method")

    def reset(self):
        """Reset node state for re-execution."""
        self.status = NodeStatus.RUNNING

    def log_info(self, message: str):
        """Log info message."""
        self.node.get_logger().info(f"[{self.name}] {message}")

    def log_warn(self, message: str):
        """Log warning message."""
        self.node.get_logger().warn(f"[{self.name}] {message}")

    def log_error(self, message: str):
        """Log error message."""
        self.node.get_logger().error(f"[{self.name}] {message}")


class BaseAction(BTNode):
    """Base class for BT action nodes (leaf nodes)."""
    pass


class BaseControl(BTNode):
    """Base class for BT control nodes (composite nodes)."""

    def __init__(self, node: 'Node', name: str):
        """
        Initialize control node.

        Args:
            node: ROS2 node reference
            name: Name of the control node
        """
        super().__init__(node, name)
        self.children: List[BTNode] = []

    def add_child(self, child: BTNode):
        """Add a child node."""
        self.children.append(child)

    def reset(self):
        """Reset this node and all children."""
        super().reset()
        for child in self.children:
            child.reset()


class BaseDecorator(BTNode):
    """Base class for BT decorator nodes."""

    def __init__(self, node: 'Node', name: str, child: BTNode = None):
        """
        Initialize decorator node.

        Args:
            node: ROS2 node reference
            name: Name of the decorator
            child: Child node to decorate
        """
        super().__init__(node, name)
        self.child = child

    def set_child(self, child: BTNode):
        """Set the child node."""
        self.child = child

    def reset(self):
        """Reset this node and child."""
        super().reset()
        if self.child:
            self.child.reset()
