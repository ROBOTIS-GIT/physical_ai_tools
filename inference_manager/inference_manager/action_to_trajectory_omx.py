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

from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PolicyTrajectory(Node):

    def __init__(self):
        super().__init__('policy_trajectory_publisher')
        self.publisher = self.create_publisher(JointTrajectory, '/leader/joint_trajectory', 10)

        self.output_joint_order = [
            'joint2',
            'joint3',
            'joint1',
            'joint4',
            'gripper_left_joint'
        ]

        self.input_joint_order = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'gripper_left_joint'
        ]

        self.reorder_indices = [
            self.input_joint_order.index(name)
            for name in self.output_joint_order
        ]

        self.get_logger().info('PolicyTrajectory node initialized.')

    def publish_action(self, action_tensor):
        if len(action_tensor) != len(self.input_joint_order):
            self.get_logger().warn(
                'Action length mismatch.'
            )
            return

        reordered_action = action_tensor[self.reorder_indices]

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.output_joint_order

        point = JointTrajectoryPoint()
        point.positions = reordered_action.tolist()
        point.time_from_start = Duration(sec=0, nanosec=0)

        traj_msg.points.append(point)
        self.publisher.publish(traj_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyTrajectory()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
