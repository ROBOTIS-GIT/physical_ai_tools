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

"""Whole-body rule-based action for fixed position movement."""

import time
from typing import TYPE_CHECKING, List
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if TYPE_CHECKING:
    from rclpy.node import Node

class RuleSwerve(BaseAction):
    """Whole-body action that moves robot to fixed target positions."""
    def __init__(
            self,
            node: 'Node',
            timeout: float = 15.0,
            topic_config: dict = None
        ):
        super().__init__(node, name="RuleSwerve")
        self.timeout = timeout
        self.topic_config = topic_config or {}
        if not isinstance(self.topic_config, dict):
            self.topic_config = {}
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.publishers = {}
        if self.topic_config and 'topic_map' in self.topic_config:
            for joint_group, topic in self.topic_config['topic_map'].items():
                if joint_group == 'leader_mobile':
                    self.publishers[joint_group] = self.node.create_publisher(
                        Twist,
                        topic,
                        qos_profile
                    )
        else:
            self.trajectory_pub = self.node.create_publisher(
                JointTrajectory,
                '/leader/joint_trajectory',
                qos_profile
            )
        self.command_sent = False
        self.start_time = None

    def tick(self) -> NodeStatus:
        """Rotate robot 90 degrees in place for the specified timeout."""
        current_time = time.time()
        if not self.command_sent:
            self._send_trajectory_command()
            self.command_sent = True
            self.start_time = current_time
            return NodeStatus.RUNNING
        elapsed = current_time - self.start_time
        if elapsed < self.timeout:
            self._publish_mobile_command()
            return NodeStatus.RUNNING
        self._stop_mobile()
        self.log_info(f"Swerve completed in {elapsed:.1f}s (90 degree rotation)")
        return NodeStatus.SUCCESS

    def _send_trajectory_command(self):
        """Publish only the swerve (90 degree rotation) command."""
        angular_z = (3.141592653589793 / 2) / self.timeout
        swerve_goal = [0.0, 0.0, angular_z]
        if self.publishers and 'leader_mobile' in self.publishers:
            twist_msg = Twist()
            twist_msg.linear.x = swerve_goal[0]
            twist_msg.linear.y = swerve_goal[1]
            twist_msg.angular.z = swerve_goal[2]
            self.publishers['leader_mobile'].publish(twist_msg)
            self.log_info(f"Published Twist for swerve: angular_z={twist_msg.angular.z}")
        else:
            # Fallback for legacy mode
            trajectory_msg = JointTrajectory()
            trajectory_msg.header.stamp.sec = 0
            trajectory_msg.header.stamp.nanosec = 0
            trajectory_msg.joint_names = ['linear_x', 'linear_y', 'angular_z']
            target_point = JointTrajectoryPoint()
            target_point.positions = swerve_goal
            target_point.time_from_start = Duration(sec=0, nanosec=0)
            trajectory_msg.points = [target_point]
            self.trajectory_pub.publish(trajectory_msg)
            self.log_info("Published legacy trajectory for swerve")

    def _publish_mobile_command(self):
        """Publish swerve velocity command (called repeatedly during execution)."""
        if 'leader_mobile' in self.publishers:
            angular_z = (3.141592653589793 / 2) / self.timeout
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.angular.z = angular_z
            self.publishers['leader_mobile'].publish(twist_msg)

    def _stop_mobile(self):
        """Stop mobile base by publishing zero velocity."""
        if 'leader_mobile' in self.publishers:
            twist_msg = Twist()
            self.publishers['leader_mobile'].publish(twist_msg)
            self.log_info("Mobile base stopped")

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.command_sent = False
        self.start_time = None
