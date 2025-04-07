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

from typing import Optional

from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
import torch
from trajectory_msgs.msg import JointTrajectory


class DataCollector(Node):

    def __init__(self):
        super().__init__('data_collector')

        self.declare_parameter(
           'joint_order_follower', [
                'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
                'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7',
                'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
                'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7'
            ]
        )
        self.declare_parameter(
            'joint_order_leader_r_hand', [
                'right_little_1_joint', 'right_ring_1_joint',
                'right_middle_1_joint', 'right_index_1_joint',
                'right_thumb_2_joint', 'right_thumb_1_joint',
            ]
        )
        self.declare_parameter(
            'joint_order_leader_l_hand', [
                'left_little_1_joint', 'left_ring_1_joint',
                'left_middle_1_joint', 'left_index_1_joint',
                'left_thumb_2_joint', 'left_thumb_1_joint',
            ]
        )
        self.declare_parameter(
            'joint_order_leader_r_arm', [
                'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
                'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7',
            ]
        )
        self.declare_parameter(
            'joint_order_leader_l_arm', [
                'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
                'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7',
            ]
        )

        self.joint_order_follower = self.get_parameter('joint_order_follower').value

        self.joint_order_leader_r_hand = self.get_parameter('joint_order_leader_r_hand').value
        self.joint_order_leader_l_hand = self.get_parameter('joint_order_leader_l_hand').value
        self.joint_order_leader_r_arm = self.get_parameter('joint_order_leader_r_arm').value
        self.joint_order_leader_l_arm = self.get_parameter('joint_order_leader_l_arm').value

        self.follower_hand_right = Subscriber(self, Int32MultiArray, '/follower/right_hand_angles')
        self.follower_hand_left = Subscriber(self, Int32MultiArray, '/follower/left_hand_angles')
        self.follower = Subscriber(
            self,
            JointState,
            '/joint_states'
        )

        self.leader_hand_right = Subscriber(
            self,
            JointTrajectory,
            '/leader/right_hand_with_timestamp'
        )
        self.leader_hand_left = Subscriber(
            self,
            JointTrajectory,
            '/leader/left_hand_with_timestamp'
        )
        self.leader_arm_right = Subscriber(
            self, JointTrajectory,
            '/leader/right_arm_with_timestamp'
        )
        self.leader_arm_left = Subscriber(
            self,
            JointTrajectory,
            '/leader/left_arm_with_timestamp'
        )

        self.sync = ApproximateTimeSynchronizer(
            [
                self.follower_hand_right,
                self.follower_hand_left,
                self.follower,
                self.leader_hand_right,
                self.leader_hand_left,
                self.leader_arm_right,
                self.leader_arm_left
            ],
            queue_size=100,
            slop=0.5,
            allow_headerless=True
        )
        self.sync.registerCallback(self.synced_callback)

        self.latest_observation = None
        self.latest_action = None

    def synced_callback(
        self,
        follower_hand_right_msg: Int32MultiArray,
        follower_hand_left_msg: Int32MultiArray,
        follower_msg: JointState,
        leader_hand_right_msg: JointTrajectory,
        leader_hand_left_msg: JointTrajectory,
        leader_arm_right_msg: JointTrajectory,
        leader_arm_left_msg: JointTrajectory
    ):
        try:
            follower_pos_map = dict(zip(
                follower_msg.name,
                follower_msg.position
            ))
            ordered_follower = [
                follower_pos_map[name]
                for name in self.joint_order_follower
            ]
            obs_pos = (
                list(map(float, follower_hand_right_msg.data)) +
                list(map(float, follower_hand_left_msg.data)) +
                ordered_follower
            )
            obs_tensor = torch.tensor(obs_pos, dtype=torch.float32)

            r_hand_pos_map = dict(zip(
                leader_hand_right_msg.joint_names,
                leader_hand_right_msg.points[0].positions
            ))
            ordered_leader_r_hand = [
                r_hand_pos_map[name]
                for name in self.joint_order_leader_r_hand
            ]

            l_hand_pos_map = dict(zip(
                leader_hand_left_msg.joint_names,
                leader_hand_left_msg.points[0].positions
            ))
            ordered_leader_l_hand = [
                l_hand_pos_map[name]
                for name in self.joint_order_leader_l_hand
            ]

            r_arm_pos_map = dict(zip(
                leader_arm_right_msg.joint_names,
                leader_arm_right_msg.points[0].positions
            ))
            ordered_leader_r_arm = [
                r_arm_pos_map[name] for
                name in self.joint_order_leader_r_arm
            ]

            l_arm_pos_map = dict(zip(
                leader_arm_left_msg.joint_names,
                leader_arm_left_msg.points[0].positions
            ))
            ordered_leader_l_arm = [
                l_arm_pos_map[name]
                for name in self.joint_order_leader_l_arm
            ]

            act_pos = (
                ordered_leader_r_hand +
                ordered_leader_l_hand +
                ordered_leader_r_arm +
                ordered_leader_l_arm
            )
            act_tensor = torch.tensor(act_pos, dtype=torch.float32)
            self.get_logger().info(f'Follower JointState: {follower_msg.name}DataCollector')

            self.latest_observation = {'observation.state': obs_tensor}
            self.latest_action = {'action': act_tensor}
            self.get_logger().info('Synced data received.')

        except KeyError as e:
            self.get_logger().error(f'Joint name missing in incoming message: {e}')

    def get_latest_data(self) -> Optional[tuple[dict, dict]]:
        if self.latest_observation is not None and self.latest_action is not None:
            return self.latest_observation, self.latest_action
        return None, None


def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
