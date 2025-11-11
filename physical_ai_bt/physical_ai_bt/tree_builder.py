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
        policy_path: str,
        task_instruction: str,
        inference_timeout: float,
        rule_timeout: float,
        joint_names: list
    ) -> BTNode:
        """
        Build the robot control behavior tree.

        Structure:
            Sequence (Execute in order)
            ├─ InferenceAction (VLA for 5 seconds)
            └─ RuleAction (Move joint1 to -1.5)

        Args:
            node: ROS2 node reference
            policy_path: Path to VLA policy
            task_instruction: Task instruction
            inference_timeout: Inference timeout in seconds
            rule_timeout: Rule-based timeout in seconds
            joint_names: Joint names from config

        Returns:
            BTNode: Root node of the tree
        """
        # Create root sequence
        root = Sequence(node, name="RobotControlSequence")

        # Add inference action (runs for 5 seconds)
        inference_action = InferenceAction(
            node=node,
            policy_path=policy_path,
            task_instruction=task_instruction,
            timeout=inference_timeout
        )
        root.add_child(inference_action)

        # Add rule-based action (moves joint1 to -1.5 after inference)
        rule_action = RuleAction(
            node=node,
            current_positions=[0, -0.378752, -0.151795, 1.541355, 0.006874, 0.73828],
            target_positions=[-1.5, -0.378752, -0.151795, 1.541355, 0.006874, 0.73828],
            joint_names=joint_names,
            timeout=rule_timeout
        )
        root.add_child(rule_action)

        return root
