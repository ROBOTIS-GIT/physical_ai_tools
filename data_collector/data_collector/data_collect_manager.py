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
# Author: Dongyun Kim

from typing import Optional

from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import torch
from trajectory_msgs.msg import JointTrajectory




class DataCollectManager(Node):
    def __init__(self, model_type: str = 'ffw_1_0'):
        super().__init__('data_collect_manager')
        self.model_type = model_type

        self.declare_parameters(
            namespace=self.model_type,
            parameters=[
                ('camera_topic', []),
                ('joint_topic', []),
                ('observation', []),
                ('joint_list', [])
            ]
        )

        # Get the parameters
        self.camera_topic = self.get_parameter(f'{self.model_type}.camera_topic').value
        self.joint_topic = self.get_parameter(f'{self.model_type}.joint_topic').value
        self.observation = self.get_parameter(f'{self.model_type}.observation').value
        self.joint_list = self.get_parameter(f'{self.model_type}.joint_list').value








        self.leader_arm_right = Subscriber(
            self,
            JointTrajectory,
            '/leader/right_arm_with_timestamp'
        )
        self.leader_arm_left = Subscriber(
            self,
            JointTrajectory,
            '/leader/left_arm_with_timestamp'
        )

        self.sync = ApproximateTimeSynchronizer(
            [
                self.follower,
                self.leader_arm_right,
                self.leader_arm_left
            ],
            queue_size=10,
            slop=0.5,
            allow_headerless=True
        )
        self.sync.registerCallback(self.synced_callback)

        self.latest_observation = None
        self.latest_action = None

    def synced_callback(
        self,
        follower_msg: JointState,
        leader_arm_right_msg: JointTrajectory,
        leader_arm_left_msg: JointTrajectory
    ):
        try:
            follower_pos_map = dict(zip(
                follower_msg.name,
                follower_msg.position
            ))
            obs_pos = [
                follower_pos_map[name]
                for name in self.joint_order_follower
            ]
            obs_tensor = torch.tensor(obs_pos, dtype=torch.float32)

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
                ordered_leader_r_arm +
                ordered_leader_l_arm
            )
            act_tensor = torch.tensor(act_pos, dtype=torch.float32)

            self.latest_observation = {'observation.state': obs_tensor}
            self.latest_action = {'action': act_tensor}
            self.get_logger().info(f'Received observation: {obs_tensor}')
            self.get_logger().info(f'Received action: {act_tensor}')

        except KeyError as e:
            self.get_logger().error(
                f'Joint name {e} missing in incoming message: {follower_msg.name}'
            )

    def get_latest_data(self) -> Optional[tuple[dict, dict]]:
        if self.latest_observation is not None and self.latest_action is not None:
            return self.latest_observation, self.latest_action
        return None, None


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
