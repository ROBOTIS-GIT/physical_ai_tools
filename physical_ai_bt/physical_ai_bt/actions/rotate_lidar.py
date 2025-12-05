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

"""LIDAR-based rotation action coordinated via services."""

from typing import TYPE_CHECKING
from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from std_srvs.srv import SetBool, Trigger

if TYPE_CHECKING:
    from rclpy.node import Node


class RotateLidar(BaseAction):
    """
    Action that coordinates LIDAR-based rotation via service communication.

    Sends rotation trigger to AI Worker node and waits for completion signal.
    Does not perform rotation directly - relies on external LIDAR control.
    """

    def __init__(self, node: 'Node', face_tape: bool = True):
        """
        Initialize RotateLidar action.

        Args:
            node: ROS2 node reference
            face_tape: Rotation mode
                - True: Rotate to face reflective tape head-on
                - False: Rotate 90° left from reflective tape
        """
        super().__init__(node, name="RotateLidar")

        self.face_tape = face_tape

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

        # State tracking
        self.trigger_sent = False
        self.trigger_future = None
        self.rotation_finished = False

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

    def tick(self) -> NodeStatus:
        """Execute rotation coordination logic."""

        # First tick: send rotation trigger
        if not self.trigger_sent:
            # Check service availability
            if not self.trigger_client.wait_for_service(timeout_sec=1.0):
                self.log_error("Rotation trigger service /rotation_trigger not available")
                return NodeStatus.FAILURE

            # Create request
            request = SetBool.Request()
            request.data = self.face_tape

            try:
                mode = "face reflective tape" if self.face_tape else "rotate 90° left"
                self.log_info(f"Sending rotation trigger: {mode}")

                self.trigger_future = self.trigger_client.call_async(request)
                self.trigger_sent = True
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
                self.log_info("Waiting for rotation completion signal...")

            except Exception as e:
                self.log_error(f"Rotation trigger exception: {str(e)}")
                return NodeStatus.FAILURE

            # Clear future to avoid re-checking
            self.trigger_future = None

        # Wait for finish callback
        if self.rotation_finished:
            self.log_info("LIDAR rotation completed successfully")
            return NodeStatus.SUCCESS

        # Still waiting for completion signal
        return NodeStatus.RUNNING

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.trigger_sent = False
        self.trigger_future = None
        self.rotation_finished = False
