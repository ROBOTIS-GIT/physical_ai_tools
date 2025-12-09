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
import time
from enum import Enum
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


class GestureStage(Enum):
    """Stages for gripper gesture detection."""
    WAITING_GRIPPER_CLOSE = 1
    WAITING_GRIPPER_OPEN = 2
    CHECKING_POSITION = 3


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
        tolerance: float = 0.1,
        gripper_close_threshold: float = 1.0,
        gripper_open_threshold: float = 0.2,
        enable_gesture: bool = True,
        check_delay: float = 0.0
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

        # Gripper gesture detection
        self.gripper_close_threshold = gripper_close_threshold
        self.gripper_open_threshold = gripper_open_threshold
        self.enable_gesture = enable_gesture
        self.current_stage = (GestureStage.WAITING_GRIPPER_CLOSE
                             if enable_gesture
                             else GestureStage.CHECKING_POSITION)

        # Time tracking - configurable delay before position checking
        self.start_time = None
        self.check_delay = check_delay  # seconds

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
            f"gesture_enabled={enable_gesture}, "
            f"check_delay={check_delay}s, "
            f"monitoring {len(LEFT_JOINT_NAMES) + len(RIGHT_JOINT_NAMES)} joints"
        )

    def _joint_state_callback(self, msg):
        """Callback for /joint_states to store joint positions."""
        self.joint_state = msg

    def _get_gripper_position(self, gripper_name: str) -> float:
        """
        Get current position of a gripper joint.

        Args:
            gripper_name: Name of gripper joint ('gripper_l_joint1' or 'gripper_r_joint1')

        Returns:
            float: Current gripper position, or -1.0 if unavailable
        """
        if self.joint_state is None:
            return -1.0

        name_to_idx = {name: i for i, name in enumerate(self.joint_state.name)}
        idx = name_to_idx.get(gripper_name)

        if idx is None:
            return -1.0

        return self.joint_state.position[idx]

    def _check_both_grippers_closed(self) -> bool:
        """
        Check if both grippers are closed (>= close_threshold).

        Returns:
            bool: True if both grippers >= close_threshold
        """
        left_pos = self._get_gripper_position('gripper_l_joint1')
        right_pos = self._get_gripper_position('gripper_r_joint1')

        if left_pos < 0 or right_pos < 0:
            return False

        return (left_pos >= self.gripper_close_threshold and
                right_pos >= self.gripper_close_threshold)

    def _check_both_grippers_open(self) -> bool:
        """
        Check if both grippers are open (< open_threshold).

        Returns:
            bool: True if both grippers < open_threshold
        """
        left_pos = self._get_gripper_position('gripper_l_joint1')
        right_pos = self._get_gripper_position('gripper_r_joint1')

        if left_pos < 0 or right_pos < 0:
            return False

        return (left_pos < self.gripper_open_threshold and
                right_pos < self.gripper_open_threshold)

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
        Execute one tick with gesture detection and position monitoring.

        State machine:
        1. WAITING_GRIPPER_CLOSE: Wait for both grippers >= close_threshold
        2. WAITING_GRIPPER_OPEN: Wait for both grippers < open_threshold
        3. CHECKING_POSITION: Check Euclidean distance <= tolerance

        Returns:
            NodeStatus.SUCCESS if gesture detected (if enabled) AND position reached
            NodeStatus.RUNNING otherwise
        """
        # Initialize start time on first tick
        if self.start_time is None:
            self.start_time = time.time()
            if self.enable_gesture:
                self.log_info("Started with gesture detection enabled")
            else:
                self.log_info(f"Started position monitoring with {self.check_delay}s delay")

        # STAGE 1: WAITING_GRIPPER_CLOSE
        if self.current_stage == GestureStage.WAITING_GRIPPER_CLOSE:
            if self._check_both_grippers_closed():
                self.log_info(
                    f"Stage 1 complete: Both grippers closed "
                    f"(L:{self._get_gripper_position('gripper_l_joint1'):.3f}, "
                    f"R:{self._get_gripper_position('gripper_r_joint1'):.3f})"
                )
                self.current_stage = GestureStage.WAITING_GRIPPER_OPEN
            return NodeStatus.RUNNING

        # STAGE 2: WAITING_GRIPPER_OPEN
        elif self.current_stage == GestureStage.WAITING_GRIPPER_OPEN:
            if self._check_both_grippers_open():
                self.log_info(
                    f"Stage 2 complete: Both grippers opened "
                    f"(L:{self._get_gripper_position('gripper_l_joint1'):.3f}, "
                    f"R:{self._get_gripper_position('gripper_r_joint1'):.3f}) "
                    "- Gesture detected!"
                )
                self.current_stage = GestureStage.CHECKING_POSITION

                # Reset start time for delay period (if delay > 0)
                if self.check_delay > 0:
                    self.start_time = time.time()
                    self.log_info(f"Starting {self.check_delay}s delay before position check")
            return NodeStatus.RUNNING

        # STAGE 3: CHECKING_POSITION
        elif self.current_stage == GestureStage.CHECKING_POSITION:
            elapsed_time = time.time() - self.start_time

            # Wait for delay before checking position (if check_delay > 0)
            if self.check_delay > 0 and elapsed_time < self.check_delay:
                if not hasattr(self, '_tick_count'):
                    self._tick_count = 0

                self._tick_count += 1
                if self._tick_count % 30 == 0:
                    remaining = self.check_delay - elapsed_time
                    self.log_info(
                        f"Waiting {remaining:.1f}s before position check "
                        f"(elapsed: {elapsed_time:.1f}s)"
                    )
                return NodeStatus.RUNNING

            # Check position
            distance = self._calculate_euclidean_distance()

            if distance <= self.tolerance:
                gesture_msg = "with gesture" if self.enable_gesture else "without gesture"
                self.log_info(
                    f"Target positions reached {gesture_msg}! "
                    f"Distance: {distance:.4f} <= {self.tolerance:.4f} "
                    f"(after {elapsed_time:.1f}s)"
                )
                return NodeStatus.SUCCESS

            # Still moving toward target
            if not hasattr(self, '_tick_count'):
                self._tick_count = 0

            self._tick_count += 1
            if self._tick_count % 30 == 0:
                self.log_info(
                    f"Distance to target: {distance:.4f} "
                    f"(tolerance: {self.tolerance:.4f}, elapsed: {elapsed_time:.1f}s)"
                )

            return NodeStatus.RUNNING

        # Should never reach here
        self.log_error(f"Invalid stage: {self.current_stage}")
        return NodeStatus.FAILURE

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.joint_state = None
        self.start_time = None
        self.current_stage = (GestureStage.WAITING_GRIPPER_CLOSE
                             if self.enable_gesture
                             else GestureStage.CHECKING_POSITION)
        if hasattr(self, '_tick_count'):
            self._tick_count = 0
