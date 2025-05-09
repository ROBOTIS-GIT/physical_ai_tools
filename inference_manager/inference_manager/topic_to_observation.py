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

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
import torch


class ObservationCollector(Node):

    def __init__(self):
        super().__init__('topic_to_observation')

        self.declare_parameter(
           'joint_order_follower', [
                'arm_r_joint1', 'arm_r_joint2',
                'arm_r_joint3', 'arm_r_joint4',
                'arm_r_joint5', 'arm_r_joint6',
                'arm_r_joint7', 'r_rh_r1_joint',
                'arm_l_joint1', 'arm_l_joint2',
                'arm_l_joint3', 'arm_l_joint4',
                'arm_l_joint5', 'arm_l_joint6',
                'arm_l_joint7', 'l_rh_r1_joint'
            ]
        )
        self.joint_order_follower = self.get_parameter('joint_order_follower').value
        self.latest_observation = None
        self.sub = self.create_subscription(JointState, '/joint_states', self.callback, 100)

    def callback(self, msg: JointState):
        pos_map = dict(zip(msg.name, msg.position))
        ordered_pos = [pos_map[name] for name in self.joint_order_follower]
        obs_tensor = torch.tensor(ordered_pos, dtype=torch.float32)

        self.latest_observation = {
            'observation.state': obs_tensor,
        }

        self.get_logger().info(f'Received observation: {obs_tensor}')

    def get_latest_data(self) -> Optional[dict]:
        return self.latest_observation


def main(args=None):
    rclpy.init(args=args)
    node = ObservationCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
