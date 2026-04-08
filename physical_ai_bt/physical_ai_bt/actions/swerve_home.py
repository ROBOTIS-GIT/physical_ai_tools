#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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

"""Action node for moving swerve, arms, and head to home position simultaneously."""

import subprocess
import threading
import time
from typing import List
from typing import TYPE_CHECKING

from physical_ai_bt.actions.base_action import BaseAction
from physical_ai_bt.bt_core import NodeStatus
from physical_ai_bt.constants import *  # noqa: F403
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

if TYPE_CHECKING:
    from rclpy.node import Node


class SwerveHome(BaseAction):
    """Move swerve steering, arms, and head to home position simultaneously."""

    DEFAULT_SWERVE_JOINTS = [
        'left_wheel_steer',
        'right_wheel_steer',
        'rear_wheel_steer',
    ]
    DEFAULT_LEFT_ARM_JOINTS = [
        'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3', 'arm_l_joint4',
        'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7', 'gripper_l_joint1',
    ]
    DEFAULT_RIGHT_ARM_JOINTS = [
        'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3', 'arm_r_joint4',
        'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7', 'gripper_r_joint1',
    ]
    DEFAULT_HEAD_JOINTS = ['head_joint1', 'head_joint2']

    def __init__(
        self,
        node: 'Node',
        swerve_positions: List[float] = None,
        left_positions: List[float] = None,
        right_positions: List[float] = None,
        head_positions: List[float] = None,
        duration: float = 10.0,
        arms_duration: float = 2.0,
        head_duration: float = 2.0,
        swerve_action_topic: str = '/swerve_steering_initial_position_controller/follow_joint_trajectory',
    ):
        super().__init__(node, name='SwerveHome')
        self.swerve_positions = swerve_positions or [0.0, 0.0, 0.0]
        self.left_positions = left_positions or [0.75, 0.0, 0.0, -2.3, 0.0, 0.0, 0.0, 0.0]
        self.right_positions = right_positions or [0.75, 0.0, 0.0, -2.3, 0.0, 0.0, 0.0, 0.0]
        self.head_positions = head_positions or [0.0, 0.0]
        self.duration = duration
        self.arms_duration = arms_duration
        self.head_duration = head_duration
        self.swerve_action_topic = swerve_action_topic

        # Arms & head publishers
        qos_profile = QoSProfile(
            depth=QOS_QUEUE_DEPTH,  # noqa: F405
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.left_pub = self.node.create_publisher(
            JointTrajectory,
            '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
            qos_profile,
        )
        self.right_pub = self.node.create_publisher(
            JointTrajectory,
            '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
            qos_profile,
        )
        self.head_pub = self.node.create_publisher(
            JointTrajectory,
            '/leader/joystick_controller_left/joint_trajectory',
            qos_profile,
        )

        # Joint state monitoring
        self.joint_state = None
        self.joint_state_sub = self.node.create_subscription(
            JointState, '/joint_states',
            self._joint_state_callback, qos_profile,
        )

        self._lock = threading.Lock()
        self._swerve_done = False
        self._arms_head_done = False
        self._started = False
        self._stop_event = threading.Event()
        self._thread = None
        self._swerve_proc = None

    def _joint_state_callback(self, msg):
        self.joint_state = msg

    def _send_swerve_goal(self):
        """Send swerve home goal via ros2 action send_goal subprocess."""
        positions = ', '.join(str(p) for p in self.swerve_positions)
        joint_names = ', '.join(self.DEFAULT_SWERVE_JOINTS)
        goal_str = (
            '{trajectory: {'
            f'joint_names: [{joint_names}], '
            'points: [{'
            f'positions: [{positions}], '
            'velocities: [0.0, 0.0, 0.0], '
            'accelerations: [0.0, 0.0, 0.0], '
            f'time_from_start: {{sec: {int(self.duration)}, nanosec: 0}}'
            '}]}}'
        )
        cmd = [
            'ros2', 'action', 'send_goal',
            self.swerve_action_topic,
            'control_msgs/action/FollowJointTrajectory',
            goal_str,
        ]
        try:
            self.log_info('Sending swerve home goal via CLI')
            self._swerve_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            stdout, stderr = self._swerve_proc.communicate(
                timeout=self.duration + 10
            )
            rc = self._swerve_proc.returncode
            if rc == 0:
                self.log_info('Swerve steering reached home')
            else:
                self.log_warn(
                    f'Swerve action returned code {rc}: '
                    f'{stderr.decode().strip()}'
                )
        except subprocess.TimeoutExpired:
            self.log_warn('Swerve action timed out')
            if self._swerve_proc:
                self._swerve_proc.kill()
        except Exception as e:
            self.log_warn(f'Swerve action failed: {e}')
        finally:
            with self._lock:
                self._swerve_done = True

    def _arms_head_control_loop(self):
        """Publish arms & head trajectories and wait for completion."""
        rate_sleep = RATE_SLEEP_SEC  # noqa: F405

        # Publish left arm
        left_traj = JointTrajectory()
        left_traj.joint_names = self.DEFAULT_LEFT_ARM_JOINTS
        left_point = JointTrajectoryPoint()
        left_point.positions = self.left_positions
        left_point.time_from_start.sec = int(self.arms_duration)
        left_traj.points.append(left_point)
        self.left_pub.publish(left_traj)

        # Publish right arm
        right_traj = JointTrajectory()
        right_traj.joint_names = self.DEFAULT_RIGHT_ARM_JOINTS
        right_point = JointTrajectoryPoint()
        right_point.positions = self.right_positions
        right_point.time_from_start.sec = int(self.arms_duration)
        right_traj.points.append(right_point)
        self.right_pub.publish(right_traj)

        # Publish head
        head_traj = JointTrajectory()
        head_traj.joint_names = self.DEFAULT_HEAD_JOINTS
        head_point = JointTrajectoryPoint()
        head_point.positions = self.head_positions
        head_point.time_from_start.sec = int(self.head_duration)
        head_traj.points.append(head_point)
        self.head_pub.publish(head_traj)

        self.log_info('Arms & head trajectories published')

        # Wait for arms & head to reach target
        all_joints = (
            self.DEFAULT_LEFT_ARM_JOINTS
            + self.DEFAULT_RIGHT_ARM_JOINTS
            + self.DEFAULT_HEAD_JOINTS
        )
        all_targets = self.left_positions + self.right_positions + self.head_positions
        threshold = POSITION_THRESHOLD_RAD  # noqa: F405

        timeout_count = 0
        max_timeout = MOVE_ARMS_TIMEOUT_TICKS  # noqa: F405
        while not self._stop_event.is_set() and timeout_count < max_timeout:
            if self.joint_state is None:
                time.sleep(rate_sleep)
                timeout_count += 1
                continue

            name_to_idx = {
                n: i for i, n in enumerate(self.joint_state.name)
            }
            all_reached = True
            for jname, target in zip(all_joints, all_targets):
                idx = name_to_idx.get(jname)
                if idx is not None:
                    if abs(self.joint_state.position[idx] - target) > threshold:
                        all_reached = False
                        break
                else:
                    all_reached = False
                    break

            if all_reached:
                self.log_info('Arms & head reached home')
                break

            time.sleep(rate_sleep)
            timeout_count += 1

        if timeout_count >= max_timeout:
            self.log_warn('Arms & head timeout, continuing')

        with self._lock:
            self._arms_head_done = True

    def tick(self) -> NodeStatus:
        if not self._started:
            self._started = True
            self._stop_event.clear()
            with self._lock:
                self._swerve_done = False
                self._arms_head_done = False

            # Start swerve in a thread (subprocess call)
            swerve_thread = threading.Thread(
                target=self._send_swerve_goal, daemon=True
            )
            swerve_thread.start()

            # Start arms & head in a thread
            self._thread = threading.Thread(
                target=self._arms_head_control_loop, daemon=True
            )
            self._thread.start()

            return NodeStatus.RUNNING

        with self._lock:
            done = self._swerve_done and self._arms_head_done

        if done:
            self.log_info('All home movements complete')
            return NodeStatus.SUCCESS
        return NodeStatus.RUNNING

    def reset(self):
        super().reset()
        self._stop_event.set()
        if self._swerve_proc and self._swerve_proc.poll() is None:
            self._swerve_proc.kill()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=THREAD_JOIN_TIMEOUT_SEC)  # noqa: F405
        self._thread = None
        self._swerve_proc = None
        self._started = False
        with self._lock:
            self._swerve_done = False
            self._arms_head_done = False
        self.joint_state = None
