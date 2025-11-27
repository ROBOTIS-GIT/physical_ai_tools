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

"""Action to automatically detect and open closed grippers."""

import threading
import time
from typing import TYPE_CHECKING
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from rclpy.qos import QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node


class RuleGripper(BaseAction):
    """Rule-based action to detect closed grippers and open them automatically."""

    def __init__(
        self,
        node: 'Node',
        closed_threshold: float = 1.0,
        open_position: float = 0.1,
        position_threshold: float = 0.01,
    ):
        """
        Initialize RuleGripper action.

        Args:
            node: ROS2 node reference
            closed_threshold: Threshold to detect closed gripper (>= this value)
            open_position: Target open position for grippers
            position_threshold: Position tolerance for completion
        """
        super().__init__(node, name="RuleGripper")
        self.gripper_joint_names = ["gripper_l_joint1", "gripper_r_joint1"]
        self.closed_threshold = closed_threshold
        self.open_position = open_position
        self.position_threshold = position_threshold

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers for left and right grippers
        self.left_gripper_pub = self.node.create_publisher(
            JointTrajectory,
            "/leader/joystick_controller_left/joint_trajectory",
            qos_profile
        )

        self.right_gripper_pub = self.node.create_publisher(
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

        # Track which grippers need to be opened
        self.grippers_to_open = {'left': False, 'right': False}

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

    def _detect_closed_grippers(self):
        """Detect which grippers are closed and need to be opened."""
        if not self.joint_state:
            return

        # Check left gripper
        left_pos = self._get_joint_position("gripper_l_joint1")
        if left_pos is not None and left_pos >= self.closed_threshold:
            self.grippers_to_open['left'] = True
            self.log_info(f"Left gripper closed detected: {left_pos:.3f}")

        # Check right gripper
        right_pos = self._get_joint_position("gripper_r_joint1")
        if right_pos is not None and right_pos >= self.closed_threshold:
            self.grippers_to_open['right'] = True
            self.log_info(f"Right gripper closed detected: {right_pos:.3f}")

    def _control_loop(self):
        """Independent control loop running in separate thread."""
        rate_sleep = 1.0 / self._control_rate

        # First, detect which grippers are closed
        self._detect_closed_grippers()

        if not self.grippers_to_open['left'] and not self.grippers_to_open['right']:
            self.log_info("No closed grippers detected")
            self._thread_success = True
            self._thread_done = True
            return

        # Open left gripper if closed
        if self.grippers_to_open['left']:
            left_traj = JointTrajectory()
            left_traj.joint_names = ["gripper_l_joint1"]
            left_point = JointTrajectoryPoint()
            left_point.positions = [self.open_position]
            left_point.time_from_start.sec = 5
            left_traj.points.append(left_point)
            self.left_gripper_pub.publish(left_traj)
            self.log_info(f"Opening left gripper to {self.open_position}")

        # Open right gripper if closed
        if self.grippers_to_open['right']:
            right_traj = JointTrajectory()
            right_traj.joint_names = ["gripper_r_joint1"]
            right_point = JointTrajectoryPoint()
            right_point.positions = [self.open_position]
            right_point.time_from_start.sec = 5
            right_traj.points.append(right_point)
            self.right_gripper_pub.publish(right_traj)
            self.log_info(f"Opening right gripper to {self.open_position}")

        # Wait for grippers to reach open position
        timeout_count = 0
        while not self._thread_done and timeout_count < 2000:  # 20s timeout
            if self.joint_state:
                all_opened = True

                # Check left gripper if it was being opened
                if self.grippers_to_open['left']:
                    left_pos = self._get_joint_position("gripper_l_joint1")
                    if left_pos is None or abs(left_pos - self.open_position) > self.position_threshold:
                        all_opened = False

                # Check right gripper if it was being opened
                if self.grippers_to_open['right']:
                    right_pos = self._get_joint_position("gripper_r_joint1")
                    if right_pos is None or abs(right_pos - self.open_position) > self.position_threshold:
                        all_opened = False

                if all_opened:
                    self.log_info("All closed grippers opened successfully")
                    self._thread_success = True
                    self._thread_done = True
                    break

            time.sleep(rate_sleep)
            timeout_count += 1

        if not self._thread_success:
            self.log_error("Timeout waiting for grippers to open")
            self._thread_done = True

    def tick(self) -> NodeStatus:
        """Execute one tick of RuleGripper action."""
        if self._thread is None:
            self.joint_state = None
            self._thread_done = False
            self._thread_success = False
            self.grippers_to_open = {'left': False, 'right': False}

            self._thread = threading.Thread(target=self._control_loop, daemon=True)
            self._thread.start()
            self.log_info("RuleGripper thread started")
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
        self.grippers_to_open = {'left': False, 'right': False}
