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

import time
from typing import TYPE_CHECKING, List
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from rclpy.qos import QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node

class RuleHeadLift(BaseAction):
    def __init__(
            self,
            node: 'Node',
            head_positions: List[float],
            lift_position: float,
            position_threshold: float = 0.001,
            timeout: float = 10.0
        ):
        super().__init__(node, name="RuleHeadLift")
        self.head_joint_names = ["head_joint1", "head_joint2"]
        self.head_positions = head_positions
        self.lift_joint_name = "lift_joint"
        self.lift_position = lift_position
        self.position_threshold = position_threshold
        self.timeout = timeout
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

    def _joint_state_callback(self, msg):
        self.joint_state = msg

    def tick(self) -> NodeStatus:
        current_time = time.time()
        if not self.command_sent:
            # Head trajectory
            head_traj = JointTrajectory()
            head_traj.joint_names = self.head_joint_names
            head_point = JointTrajectoryPoint()
            head_point.positions = self.head_positions
            head_point.time_from_start.sec = 2
            head_traj.points.append(head_point)
            self.head_pub.publish(head_traj)

            # Lift trajectory
            lift_traj = JointTrajectory()
            lift_traj.joint_names = [self.lift_joint_name]
            lift_point = JointTrajectoryPoint()
            lift_point.positions = [self.lift_position]
            lift_point.time_from_start.sec = 2
            lift_traj.points.append(lift_point)
            self.lift_pub.publish(lift_traj)

            self.command_sent = True
            self.start_time = current_time
            return NodeStatus.RUNNING

        # Feedback: check if joints reached target positions
        if self.joint_state:
            # Find indices for head and lift joints
            name_to_idx = {n: i for i, n in enumerate(self.joint_state.name)}
            all_reached = True
            # Head joints
            for jname, target in zip(self.head_joint_names, self.head_positions):
                idx = name_to_idx.get(jname)
                if idx is not None:
                    pos = self.joint_state.position[idx]
                    if abs(pos - target) > self.position_threshold:
                        all_reached = False
            # Lift joint
            idx = name_to_idx.get(self.lift_joint_name)
            if idx is not None:
                pos = self.joint_state.position[idx]
                if abs(pos - self.lift_position) > self.position_threshold:
                    all_reached = False
            if all_reached:
                self.status = NodeStatus.SUCCESS
                return NodeStatus.SUCCESS

        return NodeStatus.RUNNING

    def reset(self):
        super().reset()
        self.command_sent = False
        self.start_time = None
