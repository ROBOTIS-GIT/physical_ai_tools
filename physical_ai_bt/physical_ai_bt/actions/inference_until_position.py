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

"""Inference action that runs until arms reach target positions."""

import math
from typing import TYPE_CHECKING, List

from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node


# Joint names for left and right arms (including grippers)
LEFT_JOINT_NAMES = [
    'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3', 'arm_l_joint4',
    'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7', 'gripper_l_joint1'
]
RIGHT_JOINT_NAMES = [
    'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3', 'arm_r_joint4',
    'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7', 'gripper_r_joint1'
]


class InferenceUntilPosition(BaseAction):
    """
    Action that runs inference until both arms reach target positions.

    Returns SUCCESS when Euclidean distance between current and target positions
    is within tolerance. Does NOT pause inference - inference continues running.
    """

    def __init__(
        self,
        node: 'Node',
        left_positions: List[float],
        right_positions: List[float],
        tolerance: float = 0.1
    ):
        """
        Initialize InferenceUntilPosition action.

        Args:
            node: ROS2 node reference
            left_positions: Target positions for left arm (8 values: 7 arm joints + gripper)
            right_positions: Target positions for right arm (8 values: 7 arm joints + gripper)
            tolerance: Euclidean distance tolerance for position matching (default: 0.1)
        """
        super().__init__(node, name="InferenceUntilPosition")

        # Validate input sizes
        if len(left_positions) != 8:
            raise ValueError(f"left_positions must have 8 values, got {len(left_positions)}")
        if len(right_positions) != 8:
            raise ValueError(f"right_positions must have 8 values, got {len(right_positions)}")

        self.left_positions = left_positions
        self.right_positions = right_positions
        self.tolerance = tolerance

        # Joint state tracking
        self.joint_state = None

        # Subscribe to joint states
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            qos_profile
        )

        self.log_info(
            f"Initialized with tolerance={tolerance:.3f}, "
            f"monitoring {len(LEFT_JOINT_NAMES) + len(RIGHT_JOINT_NAMES)} joints"
        )

    def _joint_state_callback(self, msg):
        """Callback for /joint_states to store joint positions."""
        self.joint_state = msg

    def _calculate_euclidean_distance(self) -> float:
        """
        Calculate Euclidean distance between current and target positions.

        Euclidean distance formula across all 16 joints:
        distance = sqrt(sum((current[i] - target[i])^2 for all joints))

        Returns:
            float: Euclidean distance (radians), or float('inf') if data unavailable
        """
        if self.joint_state is None:
            return float('inf')

        # Build name-to-index mapping
        name_to_idx = {name: i for i, name in enumerate(self.joint_state.name)}

        # Collect squared differences
        squared_sum = 0.0

        # Left arm joints
        for joint_name, target_pos in zip(LEFT_JOINT_NAMES, self.left_positions):
            idx = name_to_idx.get(joint_name)
            if idx is None:
                self.log_warn(f"Joint {joint_name} not found in joint_states")
                return float('inf')

            current_pos = self.joint_state.position[idx]
            diff = current_pos - target_pos
            squared_sum += diff * diff

        # Right arm joints
        for joint_name, target_pos in zip(RIGHT_JOINT_NAMES, self.right_positions):
            idx = name_to_idx.get(joint_name)
            if idx is None:
                self.log_warn(f"Joint {joint_name} not found in joint_states")
                return float('inf')

            current_pos = self.joint_state.position[idx]
            diff = current_pos - target_pos
            squared_sum += diff * diff

        # Calculate Euclidean distance
        distance = math.sqrt(squared_sum)
        return distance

    def tick(self) -> NodeStatus:
        """
        Execute one tick of inference action with position monitoring.

        Returns:
            NodeStatus.SUCCESS if distance <= tolerance
            NodeStatus.RUNNING otherwise
        """
        # Calculate distance
        distance = self._calculate_euclidean_distance()

        # Check if within tolerance
        if distance <= self.tolerance:
            self.log_info(
                f"Target positions reached! Distance: {distance:.4f} <= {self.tolerance:.4f}"
            )
            return NodeStatus.SUCCESS

        # Still moving toward target
        # Log periodically for debugging (every 30 ticks)
        if not hasattr(self, '_tick_count'):
            self._tick_count = 0

        self._tick_count += 1
        if self._tick_count % 30 == 0:
            self.log_info(f"Distance to target: {distance:.4f} (tolerance: {self.tolerance:.4f})")

        return NodeStatus.RUNNING

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.joint_state = None
        if hasattr(self, '_tick_count'):
            self._tick_count = 0
