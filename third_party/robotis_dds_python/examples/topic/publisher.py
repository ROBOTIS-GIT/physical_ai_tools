#!/usr/bin/env python3
"""
Topic Publisher Example.

Demonstrates how to publish String messages from DDS to ROS 2.
Messages published to /test_topic can be received by ROS 2 subscribers.

Usage:
    python publisher.py
"""

import time

from robotis_dds_python.robotis_dds_core.idl.std_msgs.msg import String_
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode


def main():
    """Run DDS publisher example."""
    print('=== DDS Publisher Example ===')
    print('Publishing to /test_topic')
    print('ROS 2 subscribers can receive these messages with:')
    print('  ros2 topic echo /test_topic std_msgs/msg/String')
    print()
    print('Press Ctrl+C to stop')
    print()

    # Create DDS node with automatic network configuration
    node = DDSNode(
        name='publisher_example',
        domain_id=30,  # Must match ROS_DOMAIN_ID in ROS 2
        network_interface='auto',  # Auto-select network interface
        allow_multicast=True  # Enable multicast for automatic discovery
    )

    # Create publisher for String messages
    pub = node.dds_create_publisher('/test_topic', String_)

    print('Publisher ready! Starting to publish messages...')
    print()

    counter = 0
    try:
        while True:
            # Create and publish message
            msg = String_(data=f'DDS Message {counter}')
            pub.publish(msg)
            print(f'[Published] {msg.data}')

            counter += 1
            time.sleep(1.0)
    except KeyboardInterrupt:
        print('\nStopping publisher...')
    finally:
        node.dds_destroy_node()
        print('Publisher stopped.')


if __name__ == '__main__':
    main()
