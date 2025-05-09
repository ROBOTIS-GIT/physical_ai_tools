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
        self.r_arm_publisher = self.create_publisher(
            JointTrajectory,
            '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
            100
        )
        self.l_arm_publisher = self.create_publisher(
            JointTrajectory,
            '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
            100
        )

        self.input_r_arm_order = [
            'arm_r_joint1',
            'arm_r_joint2',
            'arm_r_joint3',
            'arm_r_joint4',
            'arm_r_joint5',
            'arm_r_joint6',
            'arm_r_joint7',
            'r_rh_r1_joint'
        ]
        self.input_l_arm_order = [
            'arm_l_joint1',
            'arm_l_joint2',
            'arm_l_joint3',
            'arm_l_joint4',
            'arm_l_joint5',
            'arm_l_joint6',
            'arm_l_joint7',
            'l_rh_r1_joint'
        ]
        self.output_r_arm_order = [
            'arm_r_joint1',
            'arm_r_joint4',
            'arm_r_joint5',
            'arm_r_joint6',
            'arm_r_joint2',
            'arm_r_joint3',
            'arm_r_joint7',
            'r_rh_r1_joint'

        ]
        self.output_l_arm_order = [
            'arm_l_joint1',
            'arm_l_joint2',
            'arm_l_joint3',
            'arm_l_joint4',
            'arm_l_joint6',
            'arm_l_joint7',
            'arm_l_joint5',
            'l_rh_r1_joint'

        ]
        self.reorder_r_arm_indices = [
            self.input_r_arm_order.index(name)
            for name in self.output_r_arm_order
        ]
        self.reorder_l_arm_indices = [
            self.input_l_arm_order.index(name)
            for name in self.output_l_arm_order
        ]

        self.get_logger().info('PolicyTrajectory node initialized.')

    def publish_action(self, action_tensor):
        r_arm_action = action_tensor[:8]
        reordered_r_arm_action = r_arm_action[self.reorder_r_arm_indices]
        traj_r_msg = JointTrajectory()
        traj_r_msg.joint_names = self.output_r_arm_order
        point_r = JointTrajectoryPoint()
        point_r.positions = reordered_r_arm_action.tolist()
        point_r.time_from_start = Duration(sec=0, nanosec=0)
        traj_r_msg.points.append(point_r)
        self.r_arm_publisher.publish(traj_r_msg)

        self.get_logger().info(f'Published trajectory: {point_r.positions}')

        l_arm_action = action_tensor[8:]
        reordered_l_arm_action = l_arm_action[self.reorder_l_arm_indices]
        traj_l_msg = JointTrajectory()
        traj_l_msg.joint_names = self.output_l_arm_order
        point_l = JointTrajectoryPoint()
        point_l.positions = reordered_l_arm_action.tolist()
        point_l.time_from_start = Duration(sec=0, nanosec=0)
        traj_l_msg.points.append(point_l)
        self.l_arm_publisher.publish(traj_l_msg)

        self.get_logger().info(f'Published trajectory: {point_l.positions}')


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
