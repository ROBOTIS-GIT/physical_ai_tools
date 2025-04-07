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
# Author: Seongwoo Kim, Hyungyu Kim

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory


class TrajectoryStamper(Node):

    def __init__(self):
        super().__init__('trajectory_stamper')

        self.topics = {
            'right_hand': '/leader/joint_trajectory_right_hand/joint_trajectory',
            'left_hand': '/leader/joint_trajectory_left_hand/joint_trajectory',
            'right_arm': '/leader/joint_trajectory_right/joint_trajectory',
            'left_arm': '/leader/joint_trajectory_left/joint_trajectory',
        }

        self.pub_dict = {}

        for key in self.topics:
            stamped_topic = f'/leader/{key}_with_timestamp'

            self.pub_dict[key] = {
                'trajectory': self.create_publisher(JointTrajectory, stamped_topic, 10),
            }

            self.create_subscription(
                JointTrajectory,
                self.topics[key],
                lambda msg, k=key: self.callback(msg, k),
                10
            )

    def callback(self, msg: JointTrajectory, key: str):
        now = self.get_clock().now().to_msg()

        msg.header.stamp = now
        self.pub_dict[key]['trajectory'].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryStamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
