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
# Author: Heewon Lee

from cyclonedds.core import Policy, Qos
from cyclonedds.util import duration

from robotis_dds_python.robotis_dds_core.idl.geometry_msgs.msg import Twist_
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode


def cmd_vel_callback(msg: Twist_):
    """Process incoming cmd_vel messages."""
    lin = msg.linear.x
    ang = msg.angular.z
    print(f"[cmd_vel] Linear.x = {lin:.3f} m/s, Angular.z = {ang:.3f} rad/s")
    print("---")


# QoS 설정 (Reliable + KeepLast 10)
qos = Qos(
    Policy.Reliability.Reliable(duration()),
    Policy.Durability.Volatile,
    Policy.History.KeepLast(10)
)

# DDS 노드 생성
node = DDSNode(name="cmd_vel_subscriber_node")

# 구독자 생성
reader = node.dds_create_subscription(
    topic_name="/cmd_vel",
    topic_type=Twist_,
    callback=cmd_vel_callback,
    qos=qos
)

print(f"[{node.name}] Subscription created. Listening to /cmd_vel ...")

try:
    node.dds_spin()
except KeyboardInterrupt:
    print(f"\n[{node.name}] Shutting down...")

node.dds_destroy_node()
print(f"[{node.name}] Node destroyed.")
