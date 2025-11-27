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

"""Action to move only head and lift joints to target positions."""

import threading
import time
from typing import TYPE_CHECKING, List
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from physical_ai_bt.blackboard import Blackboard
from rclpy.qos import QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node

class RuleHeadLift(BaseAction):
    def __init__(
            self,
            node: 'Node',
            head_positions: List[float] = None,
            lift_position: float = None,
            position_threshold: float = 0.01,
        ):
        super().__init__(node, name="RuleHeadLift")
        self.head_joint_names = ["head_joint1", "head_joint2"]

        # Default positions (used as fallback)
        self.default_head_positions = head_positions if head_positions else [0.695, 0.0]
        self.default_lift_position = lift_position if lift_position is not None else -0.1

        # Current positions (will be set dynamically)
        self.head_positions = self.default_head_positions
        self.lift_position = self.default_lift_position

        self.lift_joint_name = "lift_joint"
        self.position_threshold = position_threshold

        # Blackboard reference
        self.blackboard = Blackboard()

        # Special object positions
        self.special_objects = ["yellow paintbrush", "orange toothbrush", "red and yellow screwdriver"]
        self.special_head_positions = [0.3, 0.0]
        self.special_lift_position = 0.0
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.head_pub = self.node.create_publisher(
            JointTrajectory,
            "/leader/joystick_controller_left/joint_trajectory",
            qos_profile
        )
        self.lift_pub = self.node.create_publisher(
            JointTrajectory,
            "/leader/joystick_controller_right/joint_trajectory",
            qos_profile
        )
        self.command_sent = False
        self.start_time = None
        self.joint_state = None
        from sensor_msgs.msg import JointState
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
        self.joint_state = msg

    def _control_loop(self):
        """Independent control loop running in separate thread."""
        rate_sleep = 1.0 / self._control_rate

        # Publish trajectory commands once
        head_traj = JointTrajectory()
        head_traj.joint_names = self.head_joint_names
        head_point = JointTrajectoryPoint()
        head_point.positions = self.head_positions
        head_point.time_from_start.sec = 5
        head_traj.points.append(head_point)
        self.head_pub.publish(head_traj)

        lift_traj = JointTrajectory()
        lift_traj.joint_names = [self.lift_joint_name]
        lift_point = JointTrajectoryPoint()
        lift_point.positions = [self.lift_position]
        lift_point.time_from_start.sec = 5
        lift_traj.points.append(lift_point)
        self.lift_pub.publish(lift_traj)

        self.log_info("Head/Lift trajectory published")

        # Wait for joint_state and check positions
        timeout_count = 0
        while not self._thread_done and timeout_count < 1000:  # 20s timeout
            if self.joint_state is None:
                time.sleep(rate_sleep)
                timeout_count += 1
                continue

            name_to_idx = {n: i for i, n in enumerate(self.joint_state.name)}
            all_reached = True

            # Check head joints
            for jname, target in zip(self.head_joint_names, self.head_positions):
                idx = name_to_idx.get(jname)
                if idx is not None:
                    pos = self.joint_state.position[idx]
                    if abs(pos - target) > self.position_threshold:
                        all_reached = False
                        break

            # Check lift joint
            if all_reached:
                idx = name_to_idx.get(self.lift_joint_name)
                if idx is not None:
                    pos = self.joint_state.position[idx]
                    if abs(pos - self.lift_position) > self.position_threshold:
                        all_reached = False

            if all_reached:
                self.log_info("Head/Lift reached target positions")
                self._thread_success = True
                self._thread_done = True
                break

            time.sleep(rate_sleep)
            timeout_count += 1

        if not self._thread_success and not self._thread_done:
            self.log_error("Head/Lift timeout waiting for target positions")
            self._thread_done = True

    def _update_positions_from_blackboard(self):
        """Update head and lift positions based on blackboard task_instruction."""
        task_obj = self.blackboard.get('task_instruction', '')

        if task_obj in self.special_objects:
            self.head_positions = self.special_head_positions
            self.lift_position = self.special_lift_position
            self.log_info(f"Using special positions for '{task_obj}': head={self.head_positions}, lift={self.lift_position}")
        else:
            self.head_positions = self.default_head_positions
            self.lift_position = self.default_lift_position
            self.log_info(f"Using default positions for '{task_obj}': head={self.head_positions}, lift={self.lift_position}")

    def tick(self) -> NodeStatus:
        """Check thread status - actual control runs in separate thread."""
        if self._thread is None:
            # Update positions from blackboard BEFORE starting thread
            self._update_positions_from_blackboard()

            self.joint_state = None
            self._thread_done = False
            self._thread_success = False

            self._thread = threading.Thread(target=self._control_loop, daemon=True)
            self._thread.start()
            self.log_info(f"HeadLift thread started with head={self.head_positions}, lift={self.lift_position}")
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
