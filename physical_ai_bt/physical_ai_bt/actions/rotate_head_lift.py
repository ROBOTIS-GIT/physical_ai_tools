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
# Author: Claude Code

"""Action to rotate mobile base and move head/lift joints simultaneously."""

import math
import threading
import time
from typing import TYPE_CHECKING, List

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from rclpy.qos import QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node


class RotateHeadLift(BaseAction):
    """Simultaneously rotate mobile base and move head/lift joints."""

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
            head_positions: List[float] = None,
            lift_position: float = 0.0,
            position_threshold: float = 0.01,
            topic_config: dict = None
        ):
        super().__init__(node, name="RotateHeadLift")
        self.angle_deg = angle_deg
        self.head_positions = head_positions if head_positions else [0.0, 0.0]
        self.lift_position = lift_position
        self.position_threshold = position_threshold
        self.topic_config = topic_config or {}
        if not isinstance(self.topic_config, dict):
            self.topic_config = {}

        # Rotate parameters
        self.angular_velocity = 0.6
        self.kp = 0.02
        self.min_angular_velocity = 0.05

        # Head/Lift joint names
        self.head_joint_names = ["head_joint1", "head_joint2"]
        self.lift_joint_name = "lift_joint"

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Rotate publisher (Twist)
        self.publishers = {}
        if self.topic_config and 'topic_map' in self.topic_config:
            for joint_group, topic in self.topic_config['topic_map'].items():
                if joint_group == 'leader_mobile':
                    self.publishers[joint_group] = self.node.create_publisher(
                        Twist, topic, qos_profile
                    )

        # Head/Lift publishers (JointTrajectory)
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

        # Odom subscription (for rotation)
        self.odom_sub = self.node.create_subscription(
            Odometry, '/odom', self._odom_callback, qos_profile
        )
        self.odom_start_yaw = None
        self.odom_last_yaw = None

        # Joint state subscription (for head/lift)
        self.joint_state = None
        self.joint_state_sub = self.node.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, qos_profile
        )

        # Thread control - rotate
        self._rotate_thread = None
        self._rotate_done = False
        self._rotate_success = False

        # Thread control - head/lift
        self._headlift_thread = None
        self._headlift_done = False
        self._headlift_success = False

        self._control_rate = 100  # Hz

    def _odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        if self.odom_start_yaw is None:
            self.odom_start_yaw = yaw
        self.odom_last_yaw = yaw

    def _joint_state_callback(self, msg):
        self.joint_state = msg

    def _rotate_control_loop(self):
        """Rotation control loop (Twist-based proportional control)."""
        rate_sleep = 1.0 / self._control_rate

        # Wait for first odom
        timeout_count = 0
        while self.odom_start_yaw is None and timeout_count < 500:
            time.sleep(0.01)
            timeout_count += 1

        if self.odom_start_yaw is None:
            self.log_error("Timeout waiting for odom data")
            self._rotate_done = True
            self._rotate_success = False
            return

        while not self._rotate_done:
            if self.odom_last_yaw is None:
                time.sleep(rate_sleep)
                continue

            start_deg = math.degrees(self.odom_start_yaw)
            last_deg = math.degrees(self.odom_last_yaw)
            delta_deg = self.angle_diff_deg(last_deg, start_deg)
            delta_deg_norm = ((delta_deg + 180) % 360) - 180

            tolerance = 0.1
            error = self.angle_deg - delta_deg_norm

            if abs(error) <= tolerance:
                self._stop_mobile()
                self.log_info(
                    f"[Rotate] Complete: {delta_deg_norm:.2f} deg "
                    f"(target: {self.angle_deg} deg)"
                )
                self._rotate_success = True
                self._rotate_done = True
                break

            # Proportional control
            angular_z = self.kp * error
            angular_z = max(-self.angular_velocity, min(self.angular_velocity, angular_z))
            if 0 < abs(angular_z) < self.min_angular_velocity:
                angular_z = self.min_angular_velocity if angular_z > 0 else -self.min_angular_velocity

            if 'leader_mobile' in self.publishers:
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.angular.z = angular_z
                self.publishers['leader_mobile'].publish(twist_msg)

            time.sleep(rate_sleep)

    def _headlift_control_loop(self):
        """Head/Lift control loop (JointTrajectory-based)."""
        rate_sleep = 1.0 / self._control_rate

        # Publish trajectory commands
        head_traj = JointTrajectory()
        head_traj.joint_names = self.head_joint_names
        head_point = JointTrajectoryPoint()
        head_point.positions = self.head_positions
        head_point.time_from_start.sec = 1
        head_traj.points.append(head_point)
        self.head_pub.publish(head_traj)

        lift_traj = JointTrajectory()
        lift_traj.joint_names = [self.lift_joint_name]
        lift_point = JointTrajectoryPoint()
        lift_point.positions = [self.lift_position]
        lift_point.time_from_start.sec = 1
        lift_traj.points.append(lift_point)
        self.lift_pub.publish(lift_traj)

        self.log_info("Head/Lift trajectory published")

        # Wait for convergence
        timeout_count = 0
        while not self._headlift_done and timeout_count < 1000:  # 20s timeout
            if self.joint_state is None:
                time.sleep(rate_sleep)
                timeout_count += 1
                continue

            name_to_idx = {n: i for i, n in enumerate(self.joint_state.name)}
            all_reached = True

            for jname, target in zip(self.head_joint_names, self.head_positions):
                idx = name_to_idx.get(jname)
                if idx is not None:
                    pos = self.joint_state.position[idx]
                    if abs(pos - target) > self.position_threshold:
                        all_reached = False
                        break

            if all_reached:
                idx = name_to_idx.get(self.lift_joint_name)
                if idx is not None:
                    pos = self.joint_state.position[idx]
                    if abs(pos - self.lift_position) > self.position_threshold:
                        all_reached = False

            if all_reached:
                self.log_info("Head/Lift reached target positions")
                self._headlift_success = True
                self._headlift_done = True
                break

            time.sleep(rate_sleep)
            timeout_count += 1

        if not self._headlift_success and not self._headlift_done:
            self.log_error("Head/Lift timeout waiting for target positions")
            self._headlift_done = True

    def _stop_mobile(self):
        if 'leader_mobile' in self.publishers:
            twist_msg = Twist()
            self.publishers['leader_mobile'].publish(twist_msg)

    def tick(self) -> NodeStatus:
        if self._rotate_thread is None:
            # First tick - start both threads simultaneously
            self.odom_start_yaw = None
            self.odom_last_yaw = None
            self.joint_state = None
            self._rotate_done = False
            self._rotate_success = False
            self._headlift_done = False
            self._headlift_success = False

            self._rotate_thread = threading.Thread(
                target=self._rotate_control_loop, daemon=True
            )
            self._headlift_thread = threading.Thread(
                target=self._headlift_control_loop, daemon=True
            )
            self._rotate_thread.start()
            self._headlift_thread.start()
            self.log_info(
                f"RotateHeadLift started (angle={self.angle_deg}, "
                f"head={self.head_positions}, lift={self.lift_position})"
            )
            return NodeStatus.RUNNING

        # Check if both threads are done
        if self._rotate_done and self._headlift_done:
            if self._rotate_success and self._headlift_success:
                return NodeStatus.SUCCESS
            else:
                return NodeStatus.FAILURE

        return NodeStatus.RUNNING

    def reset(self):
        super().reset()
        # Stop rotate thread
        if self._rotate_thread is not None and self._rotate_thread.is_alive():
            self._rotate_done = True
            self._rotate_thread.join(timeout=1.0)
        # Stop headlift thread
        if self._headlift_thread is not None and self._headlift_thread.is_alive():
            self._headlift_done = True
            self._headlift_thread.join(timeout=1.0)
        self._rotate_thread = None
        self._headlift_thread = None
        self._rotate_done = False
        self._rotate_success = False
        self._headlift_done = False
        self._headlift_success = False
        self.odom_start_yaw = None
        self.odom_last_yaw = None
        self.joint_state = None
