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

import math
import threading
import time
from typing import TYPE_CHECKING

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from rclpy.qos import QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory

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

        # Odom subscription
        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            qos_profile
        )
        self.odom_start_yaw = None
        self.odom_last_yaw = None

        # Thread control
        self._thread = None
        self._thread_done = False
        self._thread_success = False
        self._control_rate = 100  # Hz

    def _odom_callback(self, msg):
        """Extract yaw from odometry quaternion."""
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        if self.odom_start_yaw is None:
            self.odom_start_yaw = yaw
        self.odom_last_yaw = yaw

    def _control_loop(self):
        """Independent control loop running in separate thread."""
        rate_sleep = 1.0 / self._control_rate

        # Wait for first odom
        timeout_count = 0
        while self.odom_start_yaw is None and timeout_count < 500:
            time.sleep(0.01)
            timeout_count += 1

        if self.odom_start_yaw is None:
            self.log_error("Timeout waiting for odom data")
            self._thread_done = True
            self._thread_success = False
            return

        while not self._thread_done:
            if self.odom_last_yaw is None:
                time.sleep(rate_sleep)
                continue

            # Calculate error
            start_deg = math.degrees(self.odom_start_yaw)
            last_deg = math.degrees(self.odom_last_yaw)
            delta_deg = self.angle_diff_deg(last_deg, start_deg)
            delta_deg_norm = ((delta_deg + 180) % 360) - 180

            tolerance = 0.1
            error = self.angle_deg - delta_deg_norm

            if abs(error) <= tolerance:
                # Target reached
                self._stop_mobile()
                self.log_info(f"[Thread] Rotation complete: {delta_deg_norm:.2f} deg (target: {self.angle_deg} deg)")
                self._thread_success = True
                self._thread_done = True
                break

            # Publish command
            angular_z = self.angular_velocity if error > 0 else -self.angular_velocity
            if 'leader_mobile' in self.publishers:
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.angular.z = angular_z
                self.publishers['leader_mobile'].publish(twist_msg)

            time.sleep(rate_sleep)

    def tick(self) -> NodeStatus:
        """Check thread status - actual control runs in separate thread."""
        if self._thread is None:
            # First tick - start control thread
            self.odom_start_yaw = None
            self.odom_last_yaw = None
            self._thread_done = False
            self._thread_success = False

            self._thread = threading.Thread(target=self._control_loop, daemon=True)
            self._thread.start()
            self.log_info(f"Swerve thread started (target: {self.angle_deg} deg)")
            return NodeStatus.RUNNING

        # Subsequent ticks - just check thread status
        if self._thread_done:
            if self._thread_success:
                return NodeStatus.SUCCESS
            else:
                return NodeStatus.FAILURE

        return NodeStatus.RUNNING

    def _stop_mobile(self):
        """Stop mobile base by publishing zero velocity."""
        if 'leader_mobile' in self.publishers:
            twist_msg = Twist()
            self.publishers['leader_mobile'].publish(twist_msg)
            self.log_info("Mobile base stopped")

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        # Stop thread if running
        if self._thread is not None and self._thread.is_alive():
            self._thread_done = True
            self._thread.join(timeout=1.0)
        self._thread = None
        self._thread_done = False
        self._thread_success = False
        self.odom_start_yaw = None
        self.odom_last_yaw = None
