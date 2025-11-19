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

import math
import time

from cyclonedds.core import Policy, Qos
from cyclonedds.util import duration

from robotis_dds_python.robotis_dds_core.idl.builtin_interfaces.msg import Time_
from robotis_dds_python.robotis_dds_core.idl.std_msgs.msg import Header_
from robotis_dds_python.robotis_dds_core.idl.sensor_msgs.msg import JointState_
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode


# QoS configuration: Reliable + KeepLast(1)
qos = Qos(
    Policy.Reliability.Reliable(duration()),
    Policy.Durability.Volatile,
    Policy.History.KeepLast(1)
)

# Create DDS Node
node = DDSNode(name='joint_state_publisher_node')

# Create publisher
writer = node.dds_create_publisher(
    topic_name='/joint_states',
    topic_type=JointState_,
    qos=qos
)


def create_joint_state(t: float) -> JointState_:
    """Create a JointState message with positions, velocities, and efforts."""
    joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

    positions = [math.sin(t + i) for i in range(len(joint_names))]
    velocities = [math.cos(t + i) for i in range(len(joint_names))]
    efforts = [0.0 for _ in range(len(joint_names))]

    now = time.time()
    sec = int(now)
    nsec = int((now - sec) * 1e9)

    header = Header_(stamp=Time_(sec=sec, nanosec=nsec), frame_id='base_link')

    return JointState_(
        header=header,
        name=joint_names,
        position=positions,
        velocity=velocities,
        effort=efforts
    )


t = 0.0
print(f'[{node.name}] Publisher created. Publishing JointState messages...')

try:
    while True:
        t += 0.1
        msg = create_joint_state(t)
        writer.publish(msg)
        print(f'[{node.name}] Published positions={msg.position}')
        time.sleep(1.0)

except KeyboardInterrupt:
    print(f'\n[{node.name}] Shutting down...')

# Cleanup
node.dds_destroy_node()
print(f'[{node.name}] Node destroyed.')
