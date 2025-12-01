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

"""Action node to check camera depth and proceed if average depth <= 10cm."""

import numpy as np
from sensor_msgs.msg import Image
from physical_ai_bt.actions.base_action import NodeStatus, BTNode
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Threshold constants
DEFAULT_SUCCESS_THRESHOLD = 0.10  # meters - object close enough
DEFAULT_FAILURE_THRESHOLD = 1.50  # meters - object too far, validation failed

class CameraDepth(BTNode):
    def __init__(self, node, depth_topic="/camera/cam_wrist_right/depth/image_rect_raw",
                 success_threshold=None, failure_threshold=None):
        super().__init__(node, name="CameraDepth")
        self.depth_topic = depth_topic
        self.success_threshold = success_threshold if success_threshold is not None else DEFAULT_SUCCESS_THRESHOLD
        self.failure_threshold = failure_threshold if failure_threshold is not None else DEFAULT_FAILURE_THRESHOLD
        self.depth_sub = None
        self.latest_depth = None
        self.status = NodeStatus.RUNNING
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.depth_sub = node.create_subscription(
            Image,
            self.depth_topic,
            self._depth_callback,
            qos_profile
        )

    def _depth_callback(self, msg):
        try:
            if msg.encoding == "32FC1":
                arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            elif msg.encoding == "16UC1":
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                arr = arr.astype(np.float32) / 1000.0  # convert mm to meters
            else:
                self.node.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}")
                return
            self.latest_depth = arr
        except Exception as e:
            self.node.get_logger().error(f"CameraDepth callback error: {e}")
            self.latest_depth = None

    def tick(self):
        # Wait for depth data
        if self.latest_depth is None:
            return NodeStatus.RUNNING

        arr = self.latest_depth
        avg_depth = np.nanmean(arr)

        # Handle NaN/invalid data
        if np.isnan(avg_depth):
            self.node.get_logger().warn("[CameraDepth] Depth contains NaN values")
            return NodeStatus.RUNNING

        # SUCCESS: object close enough
        if avg_depth <= self.success_threshold:
            self.node.get_logger().info(
                f"[CameraDepth] SUCCESS: depth={avg_depth:.3f}m <= {self.success_threshold}m"
            )
            self.status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS

        # FAILURE: object too far (validation failed)
        elif avg_depth >= self.failure_threshold:
            self.node.get_logger().error(
                f"[CameraDepth] FAILURE: depth={avg_depth:.3f}m >= {self.failure_threshold}m"
            )
            self.status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        # RUNNING: intermediate range, keep waiting
        else:
            self.node.get_logger().info(
                f"[CameraDepth] Waiting: depth={avg_depth:.3f}m"
            )
            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING

    def reset(self):
        self.status = NodeStatus.RUNNING
        self.latest_depth = None
