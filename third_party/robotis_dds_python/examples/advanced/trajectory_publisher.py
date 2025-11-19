#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Taehyeong Kim, Dongyun Kim

import math
import time

from cyclonedds.core import Policy, Qos
from cyclonedds.util import duration

from robotis_dds_python.robotis_dds_core.idl.builtin_interfaces.msg import Duration_, Time_
from robotis_dds_python.robotis_dds_core.idl.std_msgs.msg import Header_
from robotis_dds_python.robotis_dds_core.idl.trajectory_msgs.msg import (
    JointTrajectory_,
    JointTrajectoryPoint_,
)
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode


qos = Qos(
    Policy.Reliability.Reliable(duration()),
    Policy.Durability.Volatile,
    Policy.History.KeepLast(1)
)

# Create DDSNode (ROS 2 Node style)
node = DDSNode(name='trajectory_publisher_node')

# Create publisher
writer = node.dds_create_publisher(
    topic_name='/joint_trajectory',
    topic_type=JointTrajectory_,
    qos=qos
)


def create_trajectory(t: float) -> JointTrajectory_:
    joint_names = ['j1', 'j2', 'j3']
    points = []

    for i in range(5):
        sec = i + 1
        positions = [math.sin(t + i), math.cos(t + i), math.sin(t * 0.5 + i)]
        point = JointTrajectoryPoint_(
            positions=positions,
            velocities=[],
            accelerations=[],
            effort=[],
            time_from_start=Duration_(sec=sec, nanosec=0)
        )
        points.append(point)

    now = time.time()
    sec = int(now)
    nsec = int((now - sec) * 1e9)

    header = Header_(stamp=Time_(sec=sec, nanosec=nsec), frame_id='base_link')

    return JointTrajectory_(header=header, joint_names=joint_names, points=points)


t = 0.0

print(f'[{node.name}] Publisher created. Publishing messages...')

try:
    while True:
        t += 0.1
        msg = create_trajectory(t)
        writer.publish(msg)
        print(f'Published {len(msg.points)} points')
        time.sleep(1.0)
except KeyboardInterrupt:
    print(f'\n[{node.name}] Shutting down...')

# Cleanup
node.dds_destroy_node()
