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
        super().__init__('topic_to_observation_omx')

        self.declare_parameter(
            'joint_order_follower',
            ['joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint']
        )
        self.joint_order_follower = self.get_parameter('joint_order_follower').value
        self.latest_observation = None
        self.sub = self.create_subscription(JointState, '/joint_states', self.callback, 10)

    def callback(self, msg: JointState):
        pos_map = dict(zip(msg.name, msg.position))
        ordered_pos = [pos_map[name] for name in self.joint_order_follower]
        obs_tensor = torch.tensor(ordered_pos, dtype=torch.float32)

        self.latest_observation = {
            'observation.state': obs_tensor,
        }

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
