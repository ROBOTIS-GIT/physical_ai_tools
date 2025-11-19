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

"""Network Configuration Example - Cloud Server.

This example shows how to configure DDSNode as a cloud server to receive
connections from remote robots.
"""

import time

from robotis_dds_python.robotis_dds_core.idl.std_msgs.msg import String_
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode


def main():
    print('=== Cloud Server Node ===')
    print('This node will receive connections from robots')
    print()

    # Create cloud server node
    # Listen on all interfaces for incoming connections
    node = DDSNode(
        name='cloud_node',
        network_interface='auto',  # Auto-detect network interface (use auto instead of 0.0.0.0)
        allow_multicast=False,  # Disable multicast for WAN
        port_base=7400  # Standard DDS port
    )

    print()
    print('Creating publisher and subscriber...')

    # Create subscriber for robot status
    def status_callback(msg):
        print(f'[Cloud] Received robot status: {msg.data}')
    node.dds_create_subscription('/robot/status', String_, status_callback)

    # Create publisher for robot commands
    cmd_pub = node.dds_create_publisher('/robot/command', String_)

    print('Cloud server ready! Listening for robot connections...')
    print('Will send commands every 5 seconds.')
    print('Press Ctrl+C to stop.')
    print()

    # Send commands periodically
    commands = [
        'move_forward',
        'turn_left',
        'stop',
        'scan_area',
        'return_home'
    ]

    cmd_index = 0
    try:
        while True:
            time.sleep(5.0)

            cmd_msg = String_(data=commands[cmd_index])
            cmd_pub.publish(cmd_msg)
            print(f'[Cloud] Sent command: {cmd_msg.data}')

            cmd_index = (cmd_index + 1) % len(commands)

    except KeyboardInterrupt:
        print('\n[Cloud] Shutting down...')

    node.dds_destroy_node()


if __name__ == '__main__':
    main()
