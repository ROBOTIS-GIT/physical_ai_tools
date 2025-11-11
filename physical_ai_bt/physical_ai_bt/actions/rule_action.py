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

"""Rule-based action for fixed position movement."""

import time
from typing import TYPE_CHECKING, List

from builtin_interfaces.msg import Duration
from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if TYPE_CHECKING:
    from rclpy.node import Node


class RuleAction(BaseAction):
    """Action that moves robot to fixed target positions."""

    def __init__(
        self,
        node: 'Node',
        current_positions: List[float],
        target_positions: List[float],
        joint_names: List[str],
        position_threshold: float = 0.1,
        timeout: float = 15.0
    ):
        """
        Initialize rule-based action.

        Args:
            node: ROS2 node reference
            current_positions: Starting joint positions
            target_positions: Target joint positions
            joint_names: Joint names in correct order
            position_threshold: Threshold for position reach check
            timeout: Maximum time to reach target
        """
        super().__init__(node, name="RuleAction")
        self.current_positions = current_positions
        self.target_positions = target_positions
        self.joint_names = joint_names
        self.position_threshold = position_threshold
        self.timeout = timeout

        # Publisher for joint trajectory
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.trajectory_pub = self.node.create_publisher(
            JointTrajectory,
            '/leader/joint_trajectory',
            qos_profile
        )

        # Subscriber for follower joint states (for monitoring only)
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            qos_profile
        )

        # State variables
        self.command_sent = False
        self.start_time = None
        self.latest_joint_positions = None

    def _joint_state_callback(self, msg: JointState):
        """Callback for joint state messages."""
        # Reorder positions to match self.joint_names order
        if hasattr(msg, 'name') and msg.name:
            try:
                # Create mapping from received names to positions
                name_to_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
                # Reorder according to self.joint_names
                self.latest_joint_positions = [name_to_pos[name] for name in self.joint_names]
            except KeyError as e:
                if not hasattr(self, '_joint_error_logged'):
                    self._joint_error_logged = True
                    self.log_error(f"Joint name mismatch: {e}")
                self.latest_joint_positions = list(msg.position)
        else:
            self.latest_joint_positions = list(msg.position)

    def tick(self) -> NodeStatus:
        """Execute one tick of rule-based action."""
        current_time = time.time()

        # Step 1: Send trajectory command if not sent
        if not self.command_sent:
            self._send_trajectory_command()
            self.command_sent = True
            self.start_time = current_time
            return NodeStatus.RUNNING

        # Step 2: Check if target reached or timeout
        elapsed = current_time - self.start_time

        if elapsed > self.timeout:
            self.log_warn(f"Timeout ({self.timeout}s) reached")
            return NodeStatus.FAILURE

        if self._is_target_reached():
            self.log_info(f"Target reached in {elapsed:.1f}s")
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING

    def _send_trajectory_command(self):
        """Publish trajectory command with target positions."""
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.node.get_clock().now().to_msg()
        trajectory_msg.joint_names = self.joint_names

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = self.target_positions
        point.time_from_start = Duration(sec=2, nanosec=0)
        trajectory_msg.points = [point]

        self.trajectory_pub.publish(trajectory_msg)

    def _is_target_reached(self) -> bool:
        """Check if robot reached target position."""
        if self.latest_joint_positions is None:
            return False

        if len(self.latest_joint_positions) != len(self.target_positions):
            if not hasattr(self, '_mismatch_logged'):
                self._mismatch_logged = True
                self.log_warn(
                    f"Joint count mismatch: got {len(self.latest_joint_positions)}, "
                    f"expected {len(self.target_positions)}"
                )
            return False

        # Check if all joints are within threshold
        for i, (current, target) in enumerate(zip(self.latest_joint_positions, self.target_positions)):
            diff = abs(current - target)
            if diff > self.position_threshold:
                return False

        return True

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.command_sent = False
        self.start_time = None
        self.latest_joint_positions = None
