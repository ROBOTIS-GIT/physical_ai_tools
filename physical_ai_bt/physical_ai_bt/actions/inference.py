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

"""Inference action using VLA model via AI Server."""

import time
from typing import TYPE_CHECKING

from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from physical_ai_interfaces.msg import TaskStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node



class Inference(BaseAction):

    def __init__(
        self,
        node: 'Node',
    ):
        """
        Initialize inference action.

        Args:
            node: ROS2 node reference
            timeout: (legacy, unused) Duration to keep inference running
        """
        super().__init__(node, name="Inference")

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

        self.gripper_triggered = False
        self.gripper_hold_start = None

    def tick(self) -> NodeStatus:
        """Execute one tick of inference action. Now only monitors gripper joint states."""
        if self.gripper_triggered:
            now = time.time()
            if self.gripper_hold_start is None:
                self.gripper_hold_start = now
            elif now - self.gripper_hold_start >= 2.0:
                self.gripper_triggered = False
                self.gripper_hold_start = None
                self.log_info("Gripper joint held >= 1.0 for 2.0s, ending inference.")
                return NodeStatus.SUCCESS
        else:
            self.gripper_hold_start = None

        return NodeStatus.RUNNING

    def _joint_state_callback(self, msg):
        """Callback for /joint_states to monitor gripper joints."""
        try:
            name_to_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
            l_val = name_to_pos.get('gripper_l_joint1', 0.0)
            if l_val >= 1.0:
                self.gripper_triggered = True
            else:
                self.gripper_triggered = False
        except Exception as e:
            self.log_warn(f"Error in joint state callback: {e}")

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.gripper_triggered = False
        self.gripper_hold_start = None
