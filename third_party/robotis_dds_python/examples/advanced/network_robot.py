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
# Author: Dongyun Kim

"""Network Configuration Example - Robot connecting to Cloud Server.

This example shows how to configure DDSNode to communicate with a remote cloud server
without using XML configuration files.
"""

import time

from robotis_dds_python.robotis_dds_core.idl.std_msgs.msg import String_
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode


def main():
    print('=== Robot Node (Edge Device) ===')
    print('This node will connect to cloud server')
    print()

    # Create node to connect to ROS 2 system
    # ROS 2 uses DDS port 7400 by default
    node = DDSNode(
        name='robot_node',
        peers=['192.168.50.128:7400'],  # ROS 2 system IP
        allow_multicast=True,  # Enable multicast for LAN
        network_interface='auto'  # Auto-detect network interface
    )

    print()
    print('Creating publisher and subscriber...')

    # Create publisher for robot status
    status_pub = node.dds_create_publisher('/robot/status', String_)

    # Create subscriber for robot commands
    def command_callback(msg):
        print(f'[Robot] Received command: {msg.data}')

    node.dds_create_subscription('/robot/command', String_, command_callback)

    print('Robot node ready! Publishing status every 2 seconds...')
    print('Press Ctrl+C to stop.')
    print()

    # Publish status periodically
    counter = 0
    try:
        while True:
            status_msg = String_(data=f'Robot online - counter: {counter}')
            status_pub.publish(status_msg)
            print(f'[Robot] Published: {status_msg.data}')

            counter += 1
            time.sleep(2.0)
    except KeyboardInterrupt:
        print('\n[Robot] Shutting down...')

    node.dds_destroy_node()


if __name__ == '__main__':
    main()
