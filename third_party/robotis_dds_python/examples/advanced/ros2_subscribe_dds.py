#!/usr/bin/env python3
"""
ROS 2 Subscriber Example for DDS Node Topics.

This is a simple ROS 2 node that subscribes to topics published by DDS Node.
Run this on your ROS 2 system (192.168.50.128) to receive messages from DDS.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DDSTopicSubscriber(Node):
    def __init__(self):
        super().__init__('dds_topic_subscriber')

        # Subscribe to topic published by DDS Node
        # This subscribes to /feedback topic that DDS Node publishes
        self.subscription = self.create_subscription(
            String,
            '/feedback',  # Topic name from DDS Node
            self.feedback_callback,
            10
        )

        self.get_logger().info('=== ROS 2 DDS Topic Subscriber ===')
        self.get_logger().info('Listening to /feedback from DDS Node...')
        self.get_logger().info('Press Ctrl+C to stop')

    def feedback_callback(self, msg):
        """Process incoming message from DDS Node."""
        self.get_logger().info(f'📩 Received from DDS: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = DDSTopicSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
