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

"""Action to detect bidirectional gripper state changes."""

import threading
import time
from typing import TYPE_CHECKING
from sensor_msgs.msg import JointState
from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from rclpy.qos import QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node


class OpenGrippers(BaseAction):
    """Action to detect gripper state changes (closed→open or open→closed)."""

    def __init__(
        self,
        node: 'Node',
        closed_threshold: float = 1.0,
        open_threshold: float = 0.2,
    ):
        """
        Initialize OpenGrippers action.

        Args:
            node: ROS2 node reference
            closed_threshold: Threshold to detect closed gripper (>= this value)
            open_threshold: Threshold to detect open gripper (< this value)
        """
        super().__init__(node, name="OpenGrippers")
        self.gripper_joint_names = ["gripper_l_joint1", "gripper_r_joint1"]
        self.closed_threshold = closed_threshold
        self.open_threshold = open_threshold

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Joint state subscription
        self.joint_state = None
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            qos_profile
        )

        # Track initial state and changes
        self.initial_state = {'left': None, 'right': None}  # 'open', 'closed', or None
        self.state_changed = {'left': False, 'right': False}

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

    def _get_gripper_state(self, position: float) -> str:
        """
        Determine gripper state from position.

        Args:
            position: Current gripper position

        Returns:
            'closed' if position >= closed_threshold
            'open' if position < open_threshold
            'intermediate' otherwise
        """
        if position >= self.closed_threshold:
            return 'closed'
        elif position < self.open_threshold:
            return 'open'
        else:
            return 'intermediate'

    def _record_initial_states(self):
        """Record initial state of both grippers."""
        if not self.joint_state:
            return False

        left_pos = self._get_joint_position("gripper_l_joint1")
        if left_pos is not None:
            self.initial_state['left'] = self._get_gripper_state(left_pos)
            self.log_info(f"Left gripper initial state: {self.initial_state['left']} ({left_pos:.3f})")

        right_pos = self._get_joint_position("gripper_r_joint1")
        if right_pos is not None:
            self.initial_state['right'] = self._get_gripper_state(right_pos)
            self.log_info(f"Right gripper initial state: {self.initial_state['right']} ({right_pos:.3f})")

        return self.initial_state['left'] is not None or self.initial_state['right'] is not None

    def _detect_state_changes(self) -> bool:
        """
        Detect if either gripper has changed state.

        Returns:
            True if any gripper changed from initial state
        """
        if not self.joint_state:
            return False

        any_change = False

        # Check left gripper
        if self.initial_state['left'] is not None and not self.state_changed['left']:
            left_pos = self._get_joint_position("gripper_l_joint1")
            if left_pos is not None:
                current_state = self._get_gripper_state(left_pos)

                # Check for state transition
                if current_state != self.initial_state['left'] and current_state != 'intermediate':
                    self.state_changed['left'] = True
                    self.log_info(
                        f"Left gripper state changed: {self.initial_state['left']} → {current_state} "
                        f"({left_pos:.3f})"
                    )
                    any_change = True

        # Check right gripper
        if self.initial_state['right'] is not None and not self.state_changed['right']:
            right_pos = self._get_joint_position("gripper_r_joint1")
            if right_pos is not None:
                current_state = self._get_gripper_state(right_pos)

                # Check for state transition
                if current_state != self.initial_state['right'] and current_state != 'intermediate':
                    self.state_changed['right'] = True
                    self.log_info(
                        f"Right gripper state changed: {self.initial_state['right']} → {current_state} "
                        f"({right_pos:.3f})"
                    )
                    any_change = True

        return any_change

    def _control_loop(self):
        """Independent monitoring loop running in separate thread."""
        rate_sleep = 1.0 / self._control_rate

        # Step 1: Record initial states
        timeout_count = 0
        while not self._thread_done and timeout_count < 50:  # 0.5s timeout for initial state
            if self._record_initial_states():
                break
            time.sleep(rate_sleep)
            timeout_count += 1

        if self.initial_state['left'] is None and self.initial_state['right'] is None:
            self.log_error("Failed to read initial gripper states")
            self._thread_done = True
            return

        # Step 2: Monitor for state changes
        timeout_count = 0
        while not self._thread_done and timeout_count < 2000:  # 20s timeout
            if self._detect_state_changes():
                self.log_info("Gripper state change detected - SUCCESS")
                self._thread_success = True
                self._thread_done = True
                return

            time.sleep(rate_sleep)
            timeout_count += 1

        # Timeout reached without state change
        self.log_warn("Timeout waiting for gripper state change")
        self._thread_done = True

    def tick(self) -> NodeStatus:
        """Execute one tick of OpenGrippers action."""
        if self._thread is None:
            self.joint_state = None
            self._thread_done = False
            self._thread_success = False
            self.initial_state = {'left': None, 'right': None}
            self.state_changed = {'left': False, 'right': False}

            self._thread = threading.Thread(target=self._control_loop, daemon=True)
            self._thread.start()
            self.log_info("OpenGrippers state change detection started")
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
        self.initial_state = {'left': None, 'right': None}
        self.state_changed = {'left': False, 'right': False}
