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

"""Inference action that runs until arms return to target position."""

import time
from typing import TYPE_CHECKING, List

from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from rclpy.qos import QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node


# Threshold constants
POSITION_TOLERANCE = 0.1
HOME_HOLD_DURATION = 2.0

# Default home position values for arms and grippers
DEFAULT_LEFT_POSITIONS = [0.313, 0.095, -0.023, -1.613, 0.185, -0.291, -0.134, 0.128]
DEFAULT_RIGHT_POSITIONS = [0.413, -0.003, -0.108, -1.798, -0.121, -0.236, 0.238, 0.132]

# Joint names for left and right arms (including grippers)
LEFT_JOINT_NAMES = [
    'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3', 'arm_l_joint4',
    'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7', 'gripper_l_joint1'
]
RIGHT_JOINT_NAMES = [
    'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3', 'arm_r_joint4',
    'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7', 'gripper_r_joint1'
]



class InferenceUntilGesture(BaseAction):
    """
    Action that runs inference until arms return to target position.

    Returns SUCCESS when all arm joints are at target position for 2 seconds.
    """

    def __init__(
        self,
        node: 'Node',
        left_positions: List[float] = None,
        right_positions: List[float] = None,
    ):
        """
        Initialize InferenceUntilGesture action.

        Args:
            node: ROS2 node reference
            left_positions: Target positions for left arm joints (8 values: 7 arm joints + 1 gripper)
            right_positions: Target positions for right arm joints (8 values: 7 arm joints + 1 gripper)
        """
        super().__init__(node, name="InferenceUntilGesture")

        # Use provided positions or defaults
        self.left_positions = left_positions if left_positions is not None else DEFAULT_LEFT_POSITIONS
        self.right_positions = right_positions if right_positions is not None else DEFAULT_RIGHT_POSITIONS

        # Validate position counts
        if len(self.left_positions) != 8:
            self.log_error(f"Left positions must have 8 values, got {len(self.left_positions)}")
        if len(self.right_positions) != 8:
            self.log_error(f"Right positions must have 8 values, got {len(self.right_positions)}")

        # Build target position dictionary
        self.target_positions = {}
        for joint_name, position in zip(LEFT_JOINT_NAMES, self.left_positions):
            self.target_positions[joint_name] = position
        for joint_name, position in zip(RIGHT_JOINT_NAMES, self.right_positions):
            self.target_positions[joint_name] = position

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.joint_state_sub = self.node.create_subscription(
            __import__('sensor_msgs.msg').msg.JointState,
            '/joint_states',
            self._joint_state_callback,
            qos_profile
        )

        self.home_hold_start = None
        self.joint_positions = {}

    def tick(self) -> NodeStatus:
        """Execute one tick of inference action with target position check."""
        if self._is_at_target_position():
            now = time.time()
            if self.home_hold_start is None:
                self.home_hold_start = now
                self.log_info("At target position, holding...")
            elif now - self.home_hold_start >= HOME_HOLD_DURATION:
                self.log_info("Target position held for 2s, ending inference.")
                return NodeStatus.SUCCESS
        else:
            self.home_hold_start = None

        return NodeStatus.RUNNING

    def _is_at_target_position(self) -> bool:
        """Check if all arm joints are within tolerance of target position."""
        for joint, target in self.target_positions.items():
            current = self.joint_positions.get(joint, float('inf'))
            if abs(current - target) > POSITION_TOLERANCE:
                return False
        return True

    def _joint_state_callback(self, msg):
        """Callback for /joint_states to store joint positions."""
        try:
            self.joint_positions = {name: pos for name, pos in zip(msg.name, msg.position)}
        except Exception as e:
            self.log_warn(f"Error in joint state callback: {e}")

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.home_hold_start = None
        self.joint_positions = {}
