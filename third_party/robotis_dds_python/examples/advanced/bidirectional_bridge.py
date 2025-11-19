#!/usr/bin/env python3
"""
ROS 2 Bidirectional Bridge Example.

This ROS 2 node demonstrates bidirectional communication with DDS Node:
- Subscribes to topics FROM DDS Node
- Publishes topics TO DDS Node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ROS2DDSBridge(Node):
    def __init__(self):
        super().__init__('ros2_dds_bridge')

        # Subscribe to status messages FROM DDS Node
        self.status_subscription = self.create_subscription(
            String,
            '/robot/status',  # DDS publishes here
            self.status_callback,
            10
        )

        # Publish command messages TO DDS Node
        self.command_publisher = self.create_publisher(
            String,
            '/robot/command',  # DDS subscribes here
            10
        )

        # Timer to send commands periodically
        self.timer = self.create_timer(2.0, self.send_command)
        self.command_counter = 0

        self.get_logger().info('=== ROS 2 <-> DDS Bidirectional Bridge ===')
        self.get_logger().info('📥 Subscribing to: /robot/status (from DDS)')
        self.get_logger().info('📤 Publishing to: /robot/command (to DDS)')
        self.get_logger().info('Press Ctrl+C to stop')

    def status_callback(self, msg):
        """Receive status updates from DDS Node."""
        self.get_logger().info(f'📩 [FROM DDS] Status: {msg.data}')

    def send_command(self):
        """Send commands to DDS Node."""
        msg = String()

        # Example commands
        commands = [
            'move_forward',
            'turn_left',
            'stop',
            'scan_area',
            'return_home'
        ]

        msg.data = commands[self.command_counter % len(commands)]
        self.command_publisher.publish(msg)

        self.get_logger().info(f'📤 [TO DDS] Command: {msg.data}')
        self.command_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = ROS2DDSBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
