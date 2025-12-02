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

"""Inference action that runs until arms enter static state (minimal movement)."""

import time
from typing import TYPE_CHECKING

from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from rclpy.qos import QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node


# Position change detection thresholds
DEFAULT_POSITION_CHANGE_THRESHOLD = 0.05  # radians
DEFAULT_STATIC_DURATION = 3.0              # seconds
DEFAULT_HISTORY_WINDOW = 1.0               # seconds

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
    Action that runs inference until both arms stabilize (positions stop changing).

    Returns SUCCESS when all arm joint positions change less than threshold over 1.0s window
    for specified duration. No specific target positions required - detects when robot settles.
    """

    def __init__(
        self,
        node: 'Node',
        position_change_threshold: float = DEFAULT_POSITION_CHANGE_THRESHOLD,
        static_duration: float = DEFAULT_STATIC_DURATION,
        history_window: float = DEFAULT_HISTORY_WINDOW,
    ):
        """
        Initialize InferenceUntilGesture action.

        Args:
            node: ROS2 node reference
            position_change_threshold: Maximum position change (radians) to be considered stable (default: 0.05)
            static_duration: How long (seconds) to hold stable state before ending inference (default: 3.0)
            history_window: Time window (seconds) for position change calculation (default: 1.0)
        """
        super().__init__(node, name="InferenceUntilGesture")

        self.position_change_threshold = position_change_threshold
        self.static_duration = static_duration
        self.history_window = history_window

        # Position history tracking
        self.position_history = []  # List of (timestamp, positions_dict) tuples
        self.static_start_time = None

        # Joint names to monitor (both arms, all 8 joints each)
        self.monitored_joints = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES

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

        self.joint_positions = {}

    def _update_position_history(self):
        """
        Update position history with current joint states.

        Maintains a rolling window of recent positions for velocity calculation.
        """
        if not self.joint_positions:
            return

        current_time = time.time()

        # Extract positions for monitored joints
        current_positions = {}
        for joint_name in self.monitored_joints:
            if joint_name in self.joint_positions:
                current_positions[joint_name] = self.joint_positions[joint_name]

        # Add to history
        if len(current_positions) == len(self.monitored_joints):
            self.position_history.append((current_time, current_positions))

            # Remove old entries outside history window
            cutoff_time = current_time - self.history_window
            self.position_history = [
                (t, pos) for t, pos in self.position_history
                if t >= cutoff_time
            ]

    def _calculate_max_position_change(self) -> float:
        """
        Calculate maximum position change across all monitored joints.

        Measures how much each joint moved over the history window,
        regardless of speed.

        Returns:
            Maximum absolute position change (radians) among all joints,
            or float('inf') if insufficient data
        """
        if len(self.position_history) < 2:
            return float('inf')  # Not enough data yet

        # Get oldest and newest positions in window
        oldest_time, oldest_pos = self.position_history[0]
        newest_time, newest_pos = self.position_history[-1]

        # Calculate position change for each joint (NO time_delta division!)
        max_change = 0.0
        for joint_name in self.monitored_joints:
            if joint_name in oldest_pos and joint_name in newest_pos:
                position_change = abs(newest_pos[joint_name] - oldest_pos[joint_name])
                max_change = max(max_change, position_change)

        return max_change

    def _is_static(self) -> bool:
        """
        Check if all monitored joints are in stable state (positions not changing).

        Returns:
            True if maximum position change is below threshold
        """
        max_change = self._calculate_max_position_change()

        if max_change == float('inf'):
            return False  # Not enough data

        return max_change < self.position_change_threshold

    def tick(self) -> NodeStatus:
        """Execute one tick of inference action with static motion detection."""
        # Update position history
        self._update_position_history()

        # Check if in static state
        if self._is_static():
            now = time.time()

            if self.static_start_time is None:
                # First detection of static state
                self.static_start_time = now
                self.log_info(f"Static motion detected, holding for {self.static_duration}s...")

            elif now - self.static_start_time >= self.static_duration:
                # Static state held for required duration
                self.log_info(f"Static state held for {self.static_duration}s, ending inference.")
                return NodeStatus.SUCCESS
        else:
            # Movement detected, reset timer
            self.static_start_time = None

        return NodeStatus.RUNNING

    def _joint_state_callback(self, msg):
        """Callback for /joint_states to store joint positions."""
        try:
            self.joint_positions = {name: pos for name, pos in zip(msg.name, msg.position)}
        except Exception as e:
            self.log_warn(f"Error in joint state callback: {e}")

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.static_start_time = None
        self.position_history = []
        self.joint_positions = {}
