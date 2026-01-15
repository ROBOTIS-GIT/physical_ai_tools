#!/usr/bin/env python3
"""
Test script for EncodingStatus message pub/sub functionality.

This script tests:
1. EncodingStatus message can be published
2. EncodingStatus message can be subscribed
3. The callback mechanism works correctly
"""

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rosbag_recorder.msg import EncodingStatus


class EncodingStatusTester(Node):
    def __init__(self):
        super().__init__("encoding_status_tester")

        self.received_messages = []
        self.test_passed = False

        # Create publisher
        self.publisher = self.create_publisher(
            EncodingStatus, "rosbag_recorder/encoding_status", 10
        )

        # Create subscriber
        self.subscriber = self.create_subscription(
            EncodingStatus,
            "rosbag_recorder/encoding_status",
            self.encoding_status_callback,
            10,
        )

        self.get_logger().info("EncodingStatusTester initialized")

    def encoding_status_callback(self, msg: EncodingStatus):
        self.get_logger().info(
            f"Received EncodingStatus: success={msg.success}, "
            f"bag_path={msg.bag_path}, message={msg.message}"
        )
        self.received_messages.append(msg)

    def publish_test_message(self, success: bool, bag_path: str, message: str):
        msg = EncodingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.success = success
        msg.bag_path = bag_path
        msg.message = message

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published test message: success={success}, bag_path={bag_path}"
        )

    def run_tests(self):
        self.get_logger().info("Starting tests...")

        # Test 1: Publish success message
        self.get_logger().info("Test 1: Publishing success message")
        self.publish_test_message(
            success=True,
            bag_path="/workspace/test_rosbag/episode_001",
            message="Encoding completed successfully",
        )

        # Wait for message to be received
        time.sleep(1.0)

        # Test 2: Publish failure message
        self.get_logger().info("Test 2: Publishing failure message")
        self.publish_test_message(
            success=False,
            bag_path="/workspace/test_rosbag/episode_002",
            message="Encoding failed: disk full",
        )

        # Wait for message to be received
        time.sleep(1.0)

        # Verify results
        self.get_logger().info(f"Received {len(self.received_messages)} messages")

        if len(self.received_messages) >= 2:
            # Check first message
            msg1 = self.received_messages[0]
            if msg1.success and msg1.bag_path == "/workspace/test_rosbag/episode_001":
                self.get_logger().info(
                    "Test 1 PASSED: Success message received correctly"
                )
            else:
                self.get_logger().error("Test 1 FAILED: Success message mismatch")
                return False

            # Check second message
            msg2 = self.received_messages[1]
            if (
                not msg2.success
                and msg2.bag_path == "/workspace/test_rosbag/episode_002"
            ):
                self.get_logger().info(
                    "Test 2 PASSED: Failure message received correctly"
                )
            else:
                self.get_logger().error("Test 2 FAILED: Failure message mismatch")
                return False

            self.test_passed = True
            self.get_logger().info("All tests PASSED!")
            return True
        else:
            self.get_logger().error(
                f"FAILED: Expected 2 messages, got {len(self.received_messages)}"
            )
            return False


def main():
    rclpy.init()

    tester = EncodingStatusTester()

    # Spin in background thread
    spin_thread = threading.Thread(target=lambda: rclpy.spin(tester), daemon=True)
    spin_thread.start()

    # Give time for connections to establish
    time.sleep(0.5)

    # Run tests
    success = tester.run_tests()

    # Cleanup
    tester.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
