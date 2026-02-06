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

"""Inference action that runs until VLA model classifies the task phase."""

import time
from typing import TYPE_CHECKING

from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from physical_ai_interfaces.msg import TaskStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    from rclpy.node import Node


class InferenceUntilPhase(BaseAction):
    """
    Action that runs inference until the VLA model classifies a target task phase.

    Subscribes to /task/status and monitors the task_phase_classification field.
    Returns SUCCESS when the classification matches target_phase for a sustained
    duration with sufficient confidence (consecutive readings).

    Task phase values:
        0 = Before task
        1 = During task
        2 = Task complete
    """

    def __init__(
        self,
        node: 'Node',
        target_phase: int = 2,
        sustain_duration: float = 1.0,
        confidence_window: int = 5,
    ):
        """
        Initialize InferenceUntilPhase action.

        Args:
            node: ROS2 node reference
            target_phase: Target phase to wait for (default: 2 = task complete)
            sustain_duration: How long (seconds) the phase must be sustained (default: 1.0)
            confidence_window: Number of consecutive matching readings required (default: 5)
        """
        super().__init__(node, name="InferenceUntilPhase")

        self.target_phase = target_phase
        self.sustain_duration = sustain_duration
        self.confidence_window = confidence_window

        self.recent_phases = []
        self.phase_start_time = None
        self.current_phase = 0

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.status_sub = self.node.create_subscription(
            TaskStatus,
            '/task/status',
            self._status_callback,
            qos_profile
        )

    def _status_callback(self, msg):
        """Callback for /task/status to track task_phase_classification."""
        try:
            self.current_phase = msg.task_phase_classification
        except Exception as e:
            self.log_warn(f"Error in status callback: {e}")

    def tick(self) -> NodeStatus:
        """Execute one tick of inference action with task phase detection."""
        self.recent_phases.append(self.current_phase)
        # Keep only recent readings within confidence window
        if len(self.recent_phases) > self.confidence_window:
            self.recent_phases = self.recent_phases[-self.confidence_window:]

        # Check if all recent phases match target
        if (len(self.recent_phases) >= self.confidence_window
                and all(p == self.target_phase for p in self.recent_phases)):
            now = time.time()

            if self.phase_start_time is None:
                self.phase_start_time = now
                self.log_info(
                    f"Target phase {self.target_phase} detected, "
                    f"holding for {self.sustain_duration}s...")

            elif now - self.phase_start_time >= self.sustain_duration:
                self.log_info(
                    f"Task phase {self.target_phase} sustained for "
                    f"{self.sustain_duration}s, ending inference.")
                return NodeStatus.SUCCESS
        else:
            # Phase changed or not enough consistent readings, reset timer
            if self.phase_start_time is not None:
                self.log_info("Phase changed, resetting sustain timer.")
            self.phase_start_time = None

        return NodeStatus.RUNNING

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.recent_phases = []
        self.phase_start_time = None
        self.current_phase = 0
