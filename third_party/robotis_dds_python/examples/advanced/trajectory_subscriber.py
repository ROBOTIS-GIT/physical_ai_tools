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
# Author: Taehyeong Kim, Dongyun Kim


from cyclonedds.core import Policy, Qos
from cyclonedds.util import duration

from robotis_dds_python.robotis_dds_core.idl.trajectory_msgs.msg import JointTrajectory_
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode


def trajectory_callback(msg):
    """Process incoming trajectory messages."""
    print(f'Received trajectory with {len(msg.points)} points')
    print(f'Joint names: {msg.joint_names}')
    if msg.points:
        print(f'First point positions: {msg.points[0].positions}')
    else:
        print('First point positions: No points')
    print('---')


qos = Qos(
    Policy.Reliability.Reliable(duration()),
    Policy.Durability.Volatile,
    Policy.History.KeepLast(10)
)

# Create DDSNode (ROS 2 Node style)
node = DDSNode(name='trajectory_subscriber_node')

# Create subscription with callback
reader = node.dds_create_subscription(
    topic_name='/joint_trajectory',
    topic_type=JointTrajectory_,
    callback=trajectory_callback,
    qos=qos
)

print(f'[{node.name}] Subscription created. Spinning...')

# Spin to keep callbacks active (like rclpy.spin)
node.dds_spin()

# Cleanup
node.dds_destroy_node()
