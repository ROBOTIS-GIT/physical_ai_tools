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

"""Inference action that runs until gesture sequence is detected."""

import time
from enum import Enum
from typing import TYPE_CHECKING

from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from physical_ai_interfaces.msg import TaskStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node


class InferenceState(Enum):
    """State machine states for inference action."""

    WAITING_GRIPPER_CLOSE = 0  # Waiting for both grippers to close
    WAITING_GRIPPER_OPEN = 1   # Waiting for both grippers to open
    WAITING_HOME_POSITION = 2  # Waiting for home position hold


# Threshold constants
GRIPPER_CLOSE_THRESHOLD = 0.8
GRIPPER_OPEN_THRESHOLD = 0.2
POSITION_TOLERANCE = 0.1
HOME_HOLD_DURATION = 2.0

# Home position reference values for arms and grippers
HOME_POSITIONS = {
    # Left arm
    'arm_l_joint1': 0.313,
    'arm_l_joint2': 0.095,
    'arm_l_joint3': -0.023,
    'arm_l_joint4': -1.613,
    'arm_l_joint5': 0.185,
    'arm_l_joint6': -0.291,
    'arm_l_joint7': -0.134,
    'gripper_l_joint1': 0.128,
    # Right arm
    'arm_r_joint1': 0.413,
    'arm_r_joint2': -0.003,
    'arm_r_joint3': -0.108,
    'arm_r_joint4': -1.798,
    'arm_r_joint5': -0.121,
    'arm_r_joint6': -0.236,
    'arm_r_joint7': 0.238,
    'gripper_r_joint1': 0.132,
}



class InferenceUntilGesture(BaseAction):
    """
    Action that runs inference until 3-stage gesture sequence is completed.

    The gesture sequence consists of:
    1. Both grippers close (≥ 0.8)
    2. Both grippers open (≤ 0.2)
    3. Arms return to home position and hold for 2 seconds
    """

    def __init__(
        self,
        node: 'Node',
    ):
        """
        Initialize InferenceUntilGesture action.

        Args:
            node: ROS2 node reference
        """
        super().__init__(node, name="InferenceUntilGesture")

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

        self.state = InferenceState.WAITING_GRIPPER_CLOSE
        self.home_hold_start = None
        self.joint_positions = {}

    def tick(self) -> NodeStatus:
        """Execute one tick of inference action with 3-stage success condition."""
        if self.state == InferenceState.WAITING_GRIPPER_CLOSE:
            if self._both_grippers_closed():
                self.state = InferenceState.WAITING_GRIPPER_OPEN
                self.log_info("Both grippers closed, waiting for open...")

        elif self.state == InferenceState.WAITING_GRIPPER_OPEN:
            if self._both_grippers_open():
                self.state = InferenceState.WAITING_HOME_POSITION
                self.log_info("Both grippers open, waiting for home position...")

        elif self.state == InferenceState.WAITING_HOME_POSITION:
            if self._is_at_home_position():
                now = time.time()
                if self.home_hold_start is None:
                    self.home_hold_start = now
                elif now - self.home_hold_start >= HOME_HOLD_DURATION:
                    self.log_info("Home position held for 2s, ending inference.")
                    return NodeStatus.SUCCESS
            else:
                self.home_hold_start = None

        return NodeStatus.RUNNING

    def _both_grippers_closed(self) -> bool:
        """Check if both grippers are closed (>= threshold)."""
        l_val = self.joint_positions.get('gripper_l_joint1', 0.0)
        r_val = self.joint_positions.get('gripper_r_joint1', 0.0)
        return l_val >= GRIPPER_CLOSE_THRESHOLD and r_val >= GRIPPER_CLOSE_THRESHOLD

    def _both_grippers_open(self) -> bool:
        """Check if both grippers are open (<= threshold)."""
        l_val = self.joint_positions.get('gripper_l_joint1', 1.0)
        r_val = self.joint_positions.get('gripper_r_joint1', 1.0)
        return l_val <= GRIPPER_OPEN_THRESHOLD and r_val <= GRIPPER_OPEN_THRESHOLD

    def _is_at_home_position(self) -> bool:
        """Check if all arm joints are within tolerance of home position."""
        for joint, target in HOME_POSITIONS.items():
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
        self.state = InferenceState.WAITING_GRIPPER_CLOSE
        self.home_hold_start = None
        self.joint_positions = {}
