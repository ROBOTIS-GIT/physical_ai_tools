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
# Author: Taehyeong Kim, Dongyun Kim, Heewon Lee

from cyclonedds.core import Policy, Qos
from cyclonedds.util import duration

from robotis_dds_python.robotis_dds_core.idl.sensor_msgs.msg import JointState_
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode


def joint_state_callback(msg: JointState_):
    """Process incoming JointState messages."""
    print(f"[{msg.header.frame_id}]")
    print(f"  Names: {msg.name}")
    print(f"  Positions: {msg.position}")
    print(f"  Velocities: {msg.velocity}")
    print(f"  Efforts: {msg.effort}")
    print("---")


# QoS 설정 (Reliable + KeepLast 10)
qos = Qos(
    Policy.Reliability.Reliable(duration()),
    Policy.Durability.Volatile,
    Policy.History.KeepLast(10)
)

# DDS 노드 생성
node = DDSNode(name="joint_state_subscriber_node")

# 구독자 생성
reader = node.dds_create_subscription(
    topic_name="/joint_states",
    topic_type=JointState_,
    callback=joint_state_callback,
    qos=qos
)

print(f"[{node.name}] Subscription created. Listening to /joint_states ...")

try:
    node.dds_spin()
except KeyboardInterrupt:
    print(f"\n[{node.name}] Shutting down...")

node.dds_destroy_node()
print(f"[{node.name}] Node destroyed.")
