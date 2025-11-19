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

"""Example of managing multiple topics with a single DDSNode."""

from cyclonedds.core import Policy, Qos
from cyclonedds.util import duration

from robotis_dds_python.robotis_dds_core.idl.sensor_msgs.msg import (
    CompressedImage_,
    JointState_,
)
from robotis_dds_python.robotis_dds_core.idl.trajectory_msgs.msg import JointTrajectory_
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode


def trajectory_callback(msg):
    """Process joint trajectory messages."""
    print(f'[Trajectory] Received {len(msg.points)} points')


def joint_state_callback(msg):
    """Process joint state messages."""
    print(f'[JointState] Joints: {msg.name}, Positions: {msg.position}')


def image_callback(msg):
    """Process compressed image messages."""
    print(f'[Image] Format: {msg.format}, Size: {len(msg.data)} bytes')


# Create QoS profile
qos = Qos(
    Policy.Reliability.Reliable(duration()),
    Policy.Durability.Volatile,
    Policy.History.KeepLast(10)
)

# Create a single DDSNode (like a ROS 2 Node)
node = DDSNode(name='multi_topic_node')

# Create multiple subscriptions on the same node
print('Creating subscriptions...')
sub1 = node.dds_create_subscription(
    topic_name='/joint_trajectory',
    topic_type=JointTrajectory_,
    callback=trajectory_callback,
    qos=qos
)

sub2 = node.dds_create_subscription(
    topic_name='/joint_states',
    topic_type=JointState_,
    callback=joint_state_callback,
    qos=qos
)

sub3 = node.dds_create_subscription(
    topic_name='/camera/image/compressed',
    topic_type=CompressedImage_,
    callback=image_callback,
    qos=qos
)

# Create publishers
print('Creating publishers...')
pub1 = node.dds_create_publisher(
    topic_name='/cmd_trajectory',
    topic_type=JointTrajectory_,
    qos=qos
)

pub2 = node.dds_create_publisher(
    topic_name='/cmd_joint_states',
    topic_type=JointState_,
    qos=qos
)

print(f'\n{node.name} is ready!')
print(f'Subscribers: {list(node.subscribers.keys())}')
print(f'Publishers: {list(node.publishers.keys())}')
print('\nSpinning... Press Ctrl+C to stop.\n')

# Spin to keep all callbacks active (like rclpy.spin())
node.dds_spin()

# Cleanup
node.dds_destroy_node()
