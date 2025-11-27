#!/usr/bin/env python3
# Copyright 2025 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Author: Seongwoo Kim

"""Action to move both left and right arms to target positions."""

import time
from typing import TYPE_CHECKING, List
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from rclpy.qos import QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node

class RuleArms(BaseAction):
    def __init__(
            self,
            node: 'Node',
            left_positions: List[float],
            right_positions: List[float],
            position_threshold: float = 0.01
        ):
        super().__init__(node, name="RuleArms")
        self.left_joint_names = [
            "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
            "arm_l_joint5", "arm_l_joint6", "arm_l_joint7", "gripper_l_joint1"
        ]
        self.right_joint_names = [
            "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
            "arm_r_joint5", "arm_r_joint6", "arm_r_joint7", "gripper_r_joint1"
        ]
        self.left_positions = left_positions
        self.right_positions = right_positions
        self.position_threshold = position_threshold
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.left_pub = self.node.create_publisher(
            JointTrajectory,
            "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
            qos_profile
        )
        self.right_pub = self.node.create_publisher(
            JointTrajectory,
            "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
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
            # Left arm trajectory
            left_traj = JointTrajectory()
            left_traj.joint_names = self.left_joint_names
            left_point = JointTrajectoryPoint()
            left_point.positions = self.left_positions
            left_point.time_from_start.sec = 10
            left_traj.points.append(left_point)
            self.left_pub.publish(left_traj)

            # Right arm trajectory
            right_traj = JointTrajectory()
            right_traj.joint_names = self.right_joint_names
            right_point = JointTrajectoryPoint()
            right_point.positions = self.right_positions
            right_point.time_from_start.sec = 5
            right_traj.points.append(right_point)
            self.right_pub.publish(right_traj)

            self.command_sent = True
            self.start_time = current_time
            # Essential log only
            return NodeStatus.RUNNING

        # Feedback: check if joints reached target positions
        if self.joint_state:
            name_to_idx = {n: i for i, n in enumerate(self.joint_state.name)}
            all_reached = True
            # Left arm joints
            for jname, target in zip(self.left_joint_names, self.left_positions):
                idx = name_to_idx.get(jname)
                if idx is not None:
                    pos = self.joint_state.position[idx]
                    diff = abs(pos - target)
                    if diff > self.position_threshold:
                        all_reached = False
                        break
                else:
                    all_reached = False
                    break
            # Right arm joints
            if all_reached:
                for jname, target in zip(self.right_joint_names, self.right_positions):
                    idx = name_to_idx.get(jname)
                    if idx is not None:
                        pos = self.joint_state.position[idx]
                        diff = abs(pos - target)
                        if diff > self.position_threshold:
                            all_reached = False
                            break
                    else:
                        all_reached = False
                        break
            if all_reached:
                self.status = NodeStatus.SUCCESS
                return NodeStatus.SUCCESS
        # No joint_state received yet: silent

        return NodeStatus.RUNNING

    def reset(self):
        super().reset()
        self.command_sent = False
        self.start_time = None
