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

import torch


class DataCollector(Node):

    def __init__(self):
        super().__init__('data_collector_omx')

        self.declare_parameter(
            'joint_order_follower',
            ['joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint']
        )
        self.declare_parameter(
            'joint_order_leader',
            ['joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint']
        )
        self.joint_order_follower = self.get_parameter('joint_order_follower').value
        self.joint_order_leader = self.get_parameter('joint_order_leader').value
        self.latest_observation = None
        self.latest_action = None

        self.obs_sub = Subscriber(self, JointState, '/joint_states')
        self.act_sub = Subscriber(self, JointState, '/leader/joint_states')

        self.sync = ApproximateTimeSynchronizer(
            [self.obs_sub, self.act_sub],
            queue_size=100,
            slop=0.05
        )
        self.sync.registerCallback(self.synced_callback)

    def synced_callback(self, obs_msg: JointState, act_msg: JointState):
        obs_pos_map = dict(zip(obs_msg.name, obs_msg.position))
        act_pos_map = dict(zip(act_msg.name, act_msg.position))

        ordered_obs = [obs_pos_map[name] for name in self.joint_order_follower]
        ordered_act = [act_pos_map[name] for name in self.joint_order_leader]

        obs_tensor = torch.tensor(ordered_obs, dtype=torch.float32)
        act_tensor = torch.tensor(ordered_act, dtype=torch.float32)

        self.latest_observation = {
            'observation.state': obs_tensor,
        }
        self.latest_action = {
            'action': act_tensor,
        }

        self.get_logger().info(f'Received observation: {obs_tensor}')
        self.get_logger().info(f'Received action: {act_tensor}')

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
