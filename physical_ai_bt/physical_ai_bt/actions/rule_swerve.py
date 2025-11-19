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

"""Whole-body rule-based action for fixed position movement."""

import time
from typing import TYPE_CHECKING
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from rclpy.qos import QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if TYPE_CHECKING:
    from rclpy.node import Node

class RuleSwerve(BaseAction):
    @staticmethod
    def angle_diff_deg(a, b):
        """Return minimal difference between two angles in degrees (-180~180)."""
        d = a - b
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        return d
    """Whole-body action that moves robot to fixed target positions."""
    def __init__(
            self,
            node: 'Node',
            angle_deg: float = 90.0,
            topic_config: dict = None
        ):
        super().__init__(node, name="RuleSwerve")
        self.angle_deg = angle_deg
        self.topic_config = topic_config or {}
        if not isinstance(self.topic_config, dict):
            self.topic_config = {}
        self.angular_velocity = 0.2
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

        # For odom monitoring
        from nav_msgs.msg import Odometry
        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            qos_profile
        )
        self.odom_start_yaw = None
        self.odom_last_yaw = None

    def _odom_callback(self, msg):
        # Extract yaw from odometry quaternion
        import math
        q = msg.pose.pose.orientation
        # Quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        if self.odom_start_yaw is None:
            self.odom_start_yaw = yaw
        self.odom_last_yaw = yaw

    def tick(self) -> NodeStatus:
        """Rotate robot by the desired angle at fixed angular velocity."""
        current_time = time.time()
        if not self.command_sent:
            self._send_trajectory_command()
            self.command_sent = True
            self.start_time = current_time
            # Reset odom start yaw for measurement
            self.odom_start_yaw = None
            self.odom_last_yaw = None
            return NodeStatus.RUNNING

        # Odometry-based feedback control
        if self.odom_start_yaw is not None and self.odom_last_yaw is not None:
            import math
            start_deg = math.degrees(self.odom_start_yaw)
            last_deg = math.degrees(self.odom_last_yaw)
            delta_deg = self.angle_diff_deg(last_deg, start_deg)
            delta_deg_norm = ((delta_deg + 180) % 360) - 180
            self.log_info(f"[ODOM] Actual rotation: {delta_deg_norm:.2f} deg (target: {self.angle_deg} deg)")
            tolerance = 0.15
            if self.angle_deg > 0:
                if abs(delta_deg_norm - self.angle_deg) <= tolerance or abs((delta_deg_norm + 360) - self.angle_deg) <= tolerance:
                    self._stop_mobile()
                    self.log_info(f"[ODOM] Final rotation: {delta_deg_norm:.2f} deg (target: {self.angle_deg} deg)")
                    elapsed = current_time - self.start_time
                    self.log_info(f"Swerve completed in {elapsed:.1f}s ({self.angle_deg} degree rotation)")
                    return NodeStatus.SUCCESS
            elif self.angle_deg < 0:
                if abs(delta_deg_norm - self.angle_deg) <= tolerance or abs((delta_deg_norm - 360) - self.angle_deg) <= tolerance:
                    self._stop_mobile()
                    self.log_info(f"[ODOM] Final rotation: {delta_deg_norm:.2f} deg (target: {self.angle_deg} deg)")
                    elapsed = current_time - self.start_time
                    self.log_info(f"Swerve completed in {elapsed:.1f}s ({self.angle_deg} degree rotation)")
                    return NodeStatus.SUCCESS

        # If not reached, keep publishing command
        self._publish_mobile_command()
        return NodeStatus.RUNNING

    def _send_trajectory_command(self):
        """Publish swerve command for desired angle at fixed angular velocity."""
        angular_z = self.angular_velocity if self.angle_deg >= 0 else -self.angular_velocity
        swerve_goal = [0.0, 0.0, angular_z]
        if self.publishers and 'leader_mobile' in self.publishers:
            twist_msg = Twist()
            twist_msg.linear.x = swerve_goal[0]
            twist_msg.linear.y = swerve_goal[1]
            twist_msg.angular.z = swerve_goal[2]
            self.publishers['leader_mobile'].publish(twist_msg)
            self.log_info(f"Published Twist for swerve: angular_z={twist_msg.angular.z}")

    def _publish_mobile_command(self):
        """Publish swerve velocity command (called repeatedly during execution)."""
        if 'leader_mobile' in self.publishers:
            angular_z = self.angular_velocity if self.angle_deg >= 0 else -self.angular_velocity
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
        self.odom_start_yaw = None
        self.odom_last_yaw = None
