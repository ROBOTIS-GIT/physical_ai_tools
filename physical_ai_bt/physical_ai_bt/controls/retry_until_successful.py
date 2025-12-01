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

"""Retry control node that retries child until success or max retries exceeded."""

from typing import TYPE_CHECKING

from physical_ai_bt.actions.base_action import BaseControl, NodeStatus

if TYPE_CHECKING:
    from rclpy.node import Node


class RetryUntilSuccessful(BaseControl):
    """
    Retry control node - executes child until success or max retries exceeded.

    Behavior:
    - Ticks child node repeatedly
    - On child FAILURE: increment retry count and reset child
    - On child SUCCESS: return SUCCESS
    - On child RUNNING: return RUNNING
    - If retry_count >= max_retries: return FAILURE

    The child sequence is reset on each retry, ensuring clean state.
    """

    def __init__(self, node: 'Node', name: str = "RetryUntilSuccessful",
                 max_retries: int = 3):
        """
        Initialize RetryUntilSuccessful control node.

        Args:
            node: ROS2 node reference
            name: Name of the control node
            max_retries: Maximum number of retry attempts (default: 3)
        """
        super().__init__(node, name)
        self.max_retries = max_retries
        self.retry_count = 0

    def tick(self) -> NodeStatus:
        """
        Execute retry logic.

        Returns:
            NodeStatus: SUCCESS if child succeeds, FAILURE if max retries exceeded,
                       RUNNING if child is running or retry in progress
        """
        # Validate single child constraint
        if len(self.children) != 1:
            self.log_error("RetryUntilSuccessful requires exactly 1 child")
            return NodeStatus.FAILURE

        child = self.children[0]
        status = child.tick()

        if status == NodeStatus.SUCCESS:
            self.log_info(f"Child succeeded on attempt {self.retry_count + 1}")
            return NodeStatus.SUCCESS

        elif status == NodeStatus.FAILURE:
            self.retry_count += 1
            self.log_warn(f"Child failed - retry {self.retry_count}/{self.max_retries}")

            # Check if max retries exceeded
            if self.retry_count >= self.max_retries:
                self.log_error(f"Max retries ({self.max_retries}) exceeded")
                return NodeStatus.FAILURE

            # Reset child for next attempt
            try:
                child.reset()
            except Exception as e:
                self.log_error(f"Failed to reset child: {e}")
                return NodeStatus.FAILURE

            return NodeStatus.RUNNING

        else:  # RUNNING
            return NodeStatus.RUNNING

    def reset(self):
        """Reset retry count and all children."""
        super().reset()
        self.retry_count = 0
