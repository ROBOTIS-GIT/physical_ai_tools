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

"""LIDAR-based rotation action coordinated via services with lift control."""

import threading
import time
from typing import TYPE_CHECKING
from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node


class RotateLidar(BaseAction):
    """
    Action that coordinates LIDAR-based rotation and lift movement simultaneously.

    Rotation: Sends trigger to AI Worker node via /rotation_trigger service
              and waits for /rotation_finish completion signal.
    Lift: Moves lift joint to target position via trajectory control.

    Returns SUCCESS only when BOTH rotation and lift are complete.
    Does not perform rotation directly - relies on external LIDAR control.
    """

    def __init__(
        self,
        node: 'Node',
        face_tape: bool = True,
        lift_position: float = 0.0,
        position_threshold: float = 0.01
    ):
        """
        Initialize RotateLidar action.

        Args:
            node: ROS2 node reference
            face_tape: Rotation mode
                - True: Rotate to face reflective tape head-on
                - False: Rotate 90° left from reflective tape
            lift_position: Target position for lift joint
            position_threshold: Position tolerance for lift completion
        """
        super().__init__(node, name="RotateLidar")

        self.face_tape = face_tape
        self.lift_joint_name = "lift_joint"
        self.target_lift_position = lift_position
        self.position_threshold = position_threshold

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Service client to trigger rotation
        self.trigger_client = self.node.create_client(
            SetBool,
            '/rotation_trigger'
        )

        # Service server to receive completion signal
        self.finish_service = self.node.create_service(
            Trigger,
            '/rotation_finish',
            self._finish_callback
        )

        # Publisher for lift joint
        self.lift_pub = self.node.create_publisher(
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

        # Rotation state tracking
        self.trigger_sent = False
        self.trigger_future = None
        self.rotation_finished = False

        # Lift control thread
        self._lift_thread = None
        self._lift_thread_done = False
        self._lift_thread_success = False
        self._control_rate = 100  # Hz

    def _finish_callback(self, request, response):
        """
        Service callback when AI Worker completes rotation.

        Called by external node when LIDAR-based rotation finishes.
        """
        self.log_info("Rotation completion signal received from AI Worker")
        self.rotation_finished = True

        response.success = True
        response.message = 'Rotation completion acknowledged by BT'
        return response

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

    def _lift_control_loop(self):
        """Independent lift control loop running in separate thread."""
        rate_sleep = 1.0 / self._control_rate

        # Publish lift trajectory command
        lift_traj = JointTrajectory()
        lift_traj.joint_names = [self.lift_joint_name]
        lift_point = JointTrajectoryPoint()
        lift_point.positions = [self.target_lift_position]
        lift_point.time_from_start.sec = 5
        lift_traj.points.append(lift_point)
        self.lift_pub.publish(lift_traj)

        self.log_info(f"Lift trajectory published: target={self.target_lift_position}")

        # Wait for position reached
        timeout_count = 0
        while not self._lift_thread_done and timeout_count < 2000:  # 20s timeout
            if self.joint_state is not None:
                current_pos = self._get_joint_position(self.lift_joint_name)
                if current_pos is not None:
                    if abs(current_pos - self.target_lift_position) <= self.position_threshold:
                        self.log_info(f"Lift reached target position: {current_pos:.3f}")
                        self._lift_thread_success = True
                        self._lift_thread_done = True
                        break

            time.sleep(rate_sleep)
            timeout_count += 1

        if not self._lift_thread_success:
            self.log_error("Lift timeout waiting for target position")
            self._lift_thread_done = True

    def tick(self) -> NodeStatus:
        """Execute rotation and lift coordination logic (simultaneous)."""

        # First tick: send rotation trigger AND start lift movement
        if not self.trigger_sent:
            # Check service availability
            if not self.trigger_client.wait_for_service(timeout_sec=1.0):
                self.log_error("Rotation trigger service /rotation_trigger not available")
                return NodeStatus.FAILURE

            # Create rotation request
            request = SetBool.Request()
            request.data = self.face_tape

            try:
                mode = "face reflective tape" if self.face_tape else "rotate 90° left"
                self.log_info(f"Sending rotation trigger: {mode}")

                self.trigger_future = self.trigger_client.call_async(request)
                self.trigger_sent = True

                # Start lift movement thread simultaneously
                self.joint_state = None
                self._lift_thread_done = False
                self._lift_thread_success = False

                self._lift_thread = threading.Thread(target=self._lift_control_loop, daemon=True)
                self._lift_thread.start()
                self.log_info(f"Lift thread started: target={self.target_lift_position}")

                return NodeStatus.RUNNING

            except Exception as e:
                self.log_error(f"Failed to send rotation trigger: {str(e)}")
                return NodeStatus.FAILURE

        # Wait for trigger response
        if self.trigger_future is not None and not self.trigger_future.done():
            return NodeStatus.RUNNING

        # Check trigger response
        if self.trigger_future is not None and self.trigger_future.done():
            try:
                response = self.trigger_future.result()
                if not response.success:
                    self.log_error(f"Rotation trigger failed: {response.message}")
                    return NodeStatus.FAILURE

                self.log_info(f"Rotation trigger accepted: {response.message}")
                self.log_info("Waiting for rotation and lift completion...")

            except Exception as e:
                self.log_error(f"Rotation trigger exception: {str(e)}")
                return NodeStatus.FAILURE

            # Clear future to avoid re-checking
            self.trigger_future = None

        # Check if BOTH rotation and lift are finished
        rotation_done = self.rotation_finished
        lift_done = self._lift_thread_done

        if rotation_done and lift_done:
            # Check if both succeeded
            if self._lift_thread_success:
                self.log_info("LIDAR rotation and lift movement completed successfully")
                return NodeStatus.SUCCESS
            else:
                self.log_error("Lift movement failed")
                return NodeStatus.FAILURE

        # Still waiting for completion (either rotation or lift or both)
        return NodeStatus.RUNNING

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()

        # Reset rotation state
        self.trigger_sent = False
        self.trigger_future = None
        self.rotation_finished = False

        # Clean up lift thread
        if self._lift_thread is not None and self._lift_thread.is_alive():
            self._lift_thread_done = True
            self._lift_thread.join(timeout=1.0)
        self._lift_thread = None
        self._lift_thread_done = False
        self._lift_thread_success = False
        self.joint_state = None
