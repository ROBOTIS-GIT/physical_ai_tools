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

"""Action to move only lift joint to target position."""

import threading
import time
from typing import TYPE_CHECKING
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from rclpy.qos import QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node


class RuleLift(BaseAction):
    """Rule-based action to control lift joint independently."""

    def __init__(
        self,
        node: 'Node',
        lift_position: float = 0.0,
        position_threshold: float = 0.01,
    ):
        """
        Initialize RuleLift action.

        Args:
            node: ROS2 node reference
            lift_position: Target position for lift joint
            position_threshold: Position tolerance for completion
        """
        super().__init__(node, name="RuleLift")
        self.lift_joint_name = "lift_joint"
        self.target_position = lift_position
        self.position_threshold = position_threshold

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publisher for lift joint
        self.lift_pub = self.node.create_publisher(
            JointTrajectory,
            "/leader/joystick_controller_right/joint_trajectory",
            qos_profile
        )

        # Joint state subscription
        self.joint_state = None
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            qos_profile
        )

        # Thread control
        self._thread = None
        self._thread_done = False
        self._thread_success = False
        self._control_rate = 100  # Hz

    def _joint_state_callback(self, msg):
        """Callback for joint state updates."""
        self.joint_state = msg

    def _get_joint_position(self, joint_name: str):
        """Get current position of a joint."""
        if self.joint_state is None:
            return None

        try:
            idx = self.joint_state.name.index(joint_name)
            return self.joint_state.position[idx]
        except (ValueError, IndexError):
            return None

    def _control_loop(self):
        """Independent control loop running in separate thread."""
        rate_sleep = 1.0 / self._control_rate

        # Publish lift trajectory command
        lift_traj = JointTrajectory()
        lift_traj.joint_names = [self.lift_joint_name]
        lift_point = JointTrajectoryPoint()
        lift_point.positions = [self.target_position]
        lift_point.time_from_start.sec = 5
        lift_traj.points.append(lift_point)
        self.lift_pub.publish(lift_traj)

        self.log_info(f"Lift trajectory published: target={self.target_position}")

        # Wait for position reached
        timeout_count = 0
        while not self._thread_done and timeout_count < 2000:  # 20s timeout
            if self.joint_state is not None:
                current_pos = self._get_joint_position(self.lift_joint_name)
                if current_pos is not None:
                    if abs(current_pos - self.target_position) <= self.position_threshold:
                        self.log_info(f"Lift reached target position: {current_pos:.3f}")
                        self._thread_success = True
                        self._thread_done = True
                        break

            time.sleep(rate_sleep)
            timeout_count += 1

        if not self._thread_success:
            self.log_error("Lift timeout waiting for target position")
            self._thread_done = True

    def tick(self) -> NodeStatus:
        """Execute one tick of RuleLift action."""
        if self._thread is None:
            self.joint_state = None
            self._thread_done = False
            self._thread_success = False

            self._thread = threading.Thread(target=self._control_loop, daemon=True)
            self._thread.start()
            self.log_info(f"RuleLift thread started: target={self.target_position}")
            return NodeStatus.RUNNING

        if self._thread_done:
            return NodeStatus.SUCCESS if self._thread_success else NodeStatus.FAILURE

        return NodeStatus.RUNNING

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        if self._thread is not None and self._thread.is_alive():
            self._thread_done = True
            self._thread.join(timeout=1.0)
        self._thread = None
        self._thread_done = False
        self._thread_success = False
        self.joint_state = None
