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

"""Tree builder for constructing behavior trees."""

from typing import TYPE_CHECKING

from physical_ai_bt.actions import InferenceAction, RuleAction
from physical_ai_bt.actions.base_action import BTNode
from physical_ai_bt.controls import Sequence

if TYPE_CHECKING:
    from rclpy.node import Node


class TreeBuilder:
    """Builder class for constructing behavior trees."""

    @staticmethod
    def build_robot_control_tree(
        node: 'Node',
        inference_timeout: float,
        rule_timeout: float,
        joint_names: list,
        current_positions: list,
        target_positions: list
    ) -> BTNode:
        """
        Build the robot control behavior tree.

        Structure:
            Sequence (Execute in order)
            ├─ InferenceAction (VLA for specified duration)
            └─ RuleAction (Move to target positions)

        Args:
            node: ROS2 node reference
            inference_timeout: Inference timeout in seconds
            rule_timeout: Rule-based timeout in seconds
            joint_names: Joint names from config
            current_positions: Current joint positions
            target_positions: Target joint positions

        Returns:
            BTNode: Root node of the tree
        """
        # Create root sequence
        root = Sequence(node, name="RobotControlSequence")

        # Add inference action
        inference_action = InferenceAction(
            node=node,
            timeout=inference_timeout
        )
        root.add_child(inference_action)

        # Add rule-based action
        rule_action = RuleAction(
            node=node,
            current_positions=current_positions,
            target_positions=target_positions,
            joint_names=joint_names,
            timeout=rule_timeout
        )
        root.add_child(rule_action)

        return root
