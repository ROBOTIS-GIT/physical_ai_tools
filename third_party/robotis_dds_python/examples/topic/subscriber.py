#!/usr/bin/env python3
"""
Topic Subscriber Example.

Demonstrates how to receive ROS 2 String messages in DDS.
Subscribes to messages from ROS 2 publishers on /ros2_test topic.

Usage:
    python subscriber.py

Test with ROS 2:
    ros2 topic pub /ros2_test std_msgs/msg/String "data: 'Hello from ROS 2'"
"""

from robotis_dds_python.robotis_dds_core.idl.std_msgs.msg import String_
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode


def main():
    """Run DDS subscriber example."""
    print('=== DDS Subscriber Example ===')
    print('Listening to /ros2_test from ROS 2')
    print('Test with ROS 2:')
    print('  ros2 topic pub /ros2_test std_msgs/msg/String "data: \'Hello from ROS 2\'"')
    print()
    print('Press Ctrl+C to stop')
    print()

    # Create DDS node
    node = DDSNode(
        name='subscriber_example',
        domain_id=30,  # Must match ROS_DOMAIN_ID in ROS 2
        network_interface='auto',  # Auto-select network interface
        allow_multicast=True  # Enable multicast for automatic discovery
    )

    # Callback function for received messages
    def callback(msg):
        """Process received message."""
        print(f'[Received from ROS 2] {msg.data}')

    # Create subscriber for String messages
    node.dds_create_subscription('/ros2_test', String_, callback)

    print('Subscriber ready! Waiting for messages...')
    print()

    try:
        # Keep the node running
        node.dds_spin()
    except KeyboardInterrupt:
        print('\nStopping subscriber...')
    finally:
        node.dds_destroy_node()
        print('Subscriber stopped.')


if __name__ == '__main__':
    main()
