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

"""Connect to ROS 2 System Example.

This example shows how to connect to a ROS 2 system running on a remote machine
and subscribe to topics published by ROS 2.
"""

import time

from robotis_dds_python.robotis_dds_core.idl.std_msgs.msg import String_
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode


def main():
    print('=== Connecting to ROS 2 System ===')
    print('Target: 192.168.50.128')
    print()

    # Create node to connect to ROS 2 system
    # IMPORTANT: Make sure both systems are on the same ROS_DOMAIN_ID (default: 0)
    node = DDSNode(
        name='dds_client',
        domain_id=0,  # Must match ROS 2 system's ROS_DOMAIN_ID
        peers=['192.168.50.128:7400'],  # ROS 2 system IP address
        allow_multicast=True,  # Enable for LAN communication
        network_interface='auto'
    )

    print()
    print('Creating subscriber to listen to ROS 2 topics...')

    # Subscribe to a ROS 2 topic
    # Example: If ROS 2 is publishing String messages on /chatter
    def ros2_callback(msg):
        print(f'[Received from ROS 2] {msg.data}')

    # Subscribe to ROS 2 topic
    # Change '/chatter' to your actual ROS 2 topic name
    node.dds_create_subscription('/chatter', String_, ros2_callback)

    # Optionally, you can also publish to ROS 2 topics
    publisher = node.dds_create_publisher('/feedback', String_)

    print('Listening for ROS 2 messages...')
    print('Publishing feedback to ROS 2 every 5 seconds...')
    print('Press Ctrl+C to stop.')
    print()

    # Publish feedback periodically
    counter = 0
    try:
        while True:
            # Send feedback to ROS 2
            feedback_msg = String_(data=f'DDS client alive: {counter}')
            publisher.publish(feedback_msg)
            print(f'[Sent to ROS 2] {feedback_msg.data}')

            counter += 1
            time.sleep(5.0)

    except KeyboardInterrupt:
        print('\n[DDS Client] Shutting down...')

    node.dds_destroy_node()


if __name__ == '__main__':
    main()
