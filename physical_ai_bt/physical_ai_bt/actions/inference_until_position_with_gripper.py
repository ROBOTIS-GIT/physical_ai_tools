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

"""Inference action that runs until arms reach target positions and gripper state changes."""

import math
import time
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


class InferenceUntilPositionWithGripper(BaseAction):
    """
    Action that runs inference until both arms reach target positions AND gripper state changes.

    Detects when either gripper closes then opens (or vice versa), and returns SUCCESS when:
    1. Gripper state change is detected (closed -> open or open -> closed)
    2. AND Euclidean distance between current and target positions is within tolerance

    Does NOT pause inference - inference continues running.
    """

    def __init__(
        self,
        node: 'Node',
        left_positions: List[float],
        right_positions: List[float],
        tolerance: float = 0.1,
        gripper_closed_threshold: float = 0.2,
        gripper_open_threshold: float = 0.7
    ):
        """
        Initialize InferenceUntilPositionWithGripper action.

        Args:
            node: ROS2 node reference
            left_positions: Target positions for left arm (8 values: 7 arm joints + gripper)
            right_positions: Target positions for right arm (8 values: 7 arm joints + gripper)
            tolerance: Euclidean distance tolerance for position matching (default: 0.1)
            gripper_closed_threshold: Position value below which gripper is considered closed (default: 0.01)
            gripper_open_threshold: Position value above which gripper is considered open (default: 0.7)
        """
        super().__init__(node, name="InferenceUntilPositionWithGripper")

        # Validate input sizes
        if len(left_positions) != 8:
            raise ValueError(f"left_positions must have 8 values, got {len(left_positions)}")
        if len(right_positions) != 8:
            raise ValueError(f"right_positions must have 8 values, got {len(right_positions)}")

        self.left_positions = left_positions
        self.right_positions = right_positions
        self.tolerance = tolerance
        self.gripper_closed_threshold = gripper_closed_threshold
        self.gripper_open_threshold = gripper_open_threshold

        # Joint state tracking
        self.joint_state = None

        # Gripper state tracking
        self.gripper_state_changed = False
        self.initial_gripper_state = None  # 'open' or 'closed' or None
        self.previous_gripper_state = None

        # Time tracking - start checking position after 5 seconds
        self.start_time = None
        self.check_delay = 5.0  # seconds

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
            f"gripper_closed_threshold={gripper_closed_threshold:.3f}, "
            f"gripper_open_threshold={gripper_open_threshold:.3f}, "
            f"monitoring {len(LEFT_JOINT_NAMES) + len(RIGHT_JOINT_NAMES)} joints"
        )

    def _joint_state_callback(self, msg):
        """Callback for /joint_states to store joint positions."""
        self.joint_state = msg

    def _get_gripper_state(self) -> str:
        """
        Determine current gripper state based on both grippers.

        Returns:
            'open' if either gripper is open
            'closed' if both grippers are closed
            'unknown' if state cannot be determined
        """
        if self.joint_state is None:
            return 'unknown'

        # Build name-to-index mapping
        name_to_idx = {name: i for i, name in enumerate(self.joint_state.name)}

        # Get left gripper position
        left_gripper_idx = name_to_idx.get('gripper_l_joint1')
        if left_gripper_idx is None:
            self.log_warn("Left gripper joint not found")
            return 'unknown'
        left_gripper_pos = self.joint_state.position[left_gripper_idx]

        # Get right gripper position
        right_gripper_idx = name_to_idx.get('gripper_r_joint1')
        if right_gripper_idx is None:
            self.log_warn("Right gripper joint not found")
            return 'unknown'
        right_gripper_pos = self.joint_state.position[right_gripper_idx]

        # Determine state for each gripper
        left_is_open = left_gripper_pos > self.gripper_open_threshold
        left_is_closed = left_gripper_pos < self.gripper_closed_threshold
        right_is_open = right_gripper_pos > self.gripper_open_threshold
        right_is_closed = right_gripper_pos < self.gripper_closed_threshold

        # If either gripper is open, overall state is 'open'
        if left_is_open or right_is_open:
            return 'open'
        # If both grippers are closed
        elif left_is_closed and right_is_closed:
            return 'closed'
        else:
            # In between - consider as previous state if available
            return self.previous_gripper_state if self.previous_gripper_state else 'unknown'

    def _check_gripper_state_change(self):
        """Check if gripper state has changed from initial state."""
        current_state = self._get_gripper_state()

        # Skip if state is unknown
        if current_state == 'unknown':
            return

        # Record initial state
        if self.initial_gripper_state is None:
            self.initial_gripper_state = current_state
            self.previous_gripper_state = current_state
            self.log_info(f"Initial gripper state: {current_state}")
            return

        # Check for state change
        if self.previous_gripper_state != current_state:
            self.log_info(
                f"Gripper state changed: {self.previous_gripper_state} -> {current_state}"
            )
            self.previous_gripper_state = current_state

            # Detect state change (open -> closed or closed -> open)
            if (self.initial_gripper_state == 'open' and current_state == 'closed') or \
               (self.initial_gripper_state == 'closed' and current_state == 'open'):
                self.gripper_state_changed = True
                self.log_info("Gripper state change detected!")

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
        Execute one tick of inference action with position and gripper state monitoring.

        Position checking starts after 5 seconds delay to allow initial movement.

        Returns:
            NodeStatus.SUCCESS if gripper state changed AND distance <= tolerance (after 5 second delay)
            NodeStatus.RUNNING otherwise
        """
        # Initialize start time on first tick
        if self.start_time is None:
            self.start_time = time.time()
            self.log_info(f"Started position and gripper monitoring with {self.check_delay}s delay")

        # Always check gripper state
        self._check_gripper_state_change()

        # Calculate elapsed time
        elapsed_time = time.time() - self.start_time

        # Wait for initial delay before checking position
        if elapsed_time < self.check_delay:
            # Log periodically during delay period (every 30 ticks)
            if not hasattr(self, '_tick_count'):
                self._tick_count = 0

            self._tick_count += 1
            if self._tick_count % 30 == 0:
                remaining = self.check_delay - elapsed_time
                self.log_info(
                    f"Waiting {remaining:.1f}s before position check "
                    f"(elapsed: {elapsed_time:.1f}s, gripper_changed: {self.gripper_state_changed})"
                )

            return NodeStatus.RUNNING

        # After delay, check both conditions
        distance = self._calculate_euclidean_distance()

        # Check if both conditions are met
        position_reached = distance <= self.tolerance

        if self.gripper_state_changed and position_reached:
            self.log_info(
                f"Both conditions met! Gripper state changed and position reached. "
                f"Distance: {distance:.4f} <= {self.tolerance:.4f} "
                f"(after {elapsed_time:.1f}s)"
            )
            return NodeStatus.SUCCESS

        # Log status periodically (every 30 ticks)
        if not hasattr(self, '_tick_count'):
            self._tick_count = 0

        self._tick_count += 1
        if self._tick_count % 30 == 0:
            self.log_info(
                f"Status - Distance: {distance:.4f} (tolerance: {self.tolerance:.4f}), "
                f"Gripper changed: {self.gripper_state_changed}, "
                f"elapsed: {elapsed_time:.1f}s"
            )

        return NodeStatus.RUNNING

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.joint_state = None
        self.start_time = None
        self.gripper_state_changed = False
        self.initial_gripper_state = None
        self.previous_gripper_state = None
        if hasattr(self, '_tick_count'):
            self._tick_count = 0
