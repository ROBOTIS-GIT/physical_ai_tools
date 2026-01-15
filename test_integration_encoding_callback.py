#!/usr/bin/env python3
"""
Integration test for the encoding status callback mechanism.

This script simulates:
1. physical_ai_server receiving encoding_status message
2. _on_encoding_complete callback being invoked
3. robot_config.yaml metadata being saved (deferred)

This test does NOT require the actual rosbag_recorder or physical_ai_server
to be running with the new code. It tests the callback mechanism in isolation.
"""

import os
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from rosbag_recorder.msg import EncodingStatus


class MockPhysicalAIServer(Node):
    """
    Mock of PhysicalAIServer that tests the encoding callback mechanism.
    """

    def __init__(self, test_rosbag_path: str):
        super().__init__("mock_physical_ai_server")

        self.test_rosbag_path = test_rosbag_path
        self._pending_metadata_rosbag_path = None
        self.metadata_saved = False
        self.callback_invoked = False

        # Simulate params and joint_order for metadata saving
        self.params = {
            "camera_topic_list": [
                "cam_left:/camera/left/image",
                "cam_right:/camera/right/image",
            ],
            "joint_topic_list": [
                "follower_arm:/follower/joint_states",
                "leader_arm:/leader/joint_trajectory",
            ],
        }
        self.joint_order = {
            "follower_arm": ["joint1", "joint2", "joint3"],
            "leader_arm": ["joint1", "joint2", "joint3"],
        }
        self.total_joint_order = ["joint1", "joint2", "joint3"]
        self.robot_type = "test_robot"

        # Create subscriber (mimics communicator's subscriber)
        self.encoding_status_subscriber = self.create_subscription(
            EncodingStatus,
            "rosbag_recorder/encoding_status",
            self._on_encoding_complete,
            10,
        )

        self.get_logger().info("MockPhysicalAIServer initialized")

    def simulate_stop_transition(self):
        """Simulate _handle_stop_transition - defer metadata saving."""
        self.get_logger().info("Simulating STOP transition...")
        self._pending_metadata_rosbag_path = self.test_rosbag_path
        self.get_logger().info(
            f"Deferred metadata saving for: {self.test_rosbag_path} (waiting for encoding)"
        )

    def _on_encoding_complete(self, msg: EncodingStatus):
        """
        Callback invoked when rosbag_recorder finishes encoding videos.
        This is the actual implementation from physical_ai_server.py.
        """
        self.callback_invoked = True

        if msg.success:
            self.get_logger().info(
                f"Encoding completed successfully for: {msg.bag_path}"
            )
        else:
            self.get_logger().warning(
                f"Encoding failed for {msg.bag_path}: {msg.message}"
            )

        # Save metadata if we have a pending path
        rosbag_path = (
            msg.bag_path if msg.bag_path else self._pending_metadata_rosbag_path
        )

        if rosbag_path:
            self._save_rosbag_metadata(rosbag_path)
            self._pending_metadata_rosbag_path = None
        else:
            self.get_logger().warning(
                "Encoding complete but no rosbag path available for metadata saving"
            )

    def _save_rosbag_metadata(self, rosbag_path: str):
        """Save robot_config.yaml metadata to rosbag directory."""
        import yaml

        if not rosbag_path or not os.path.exists(rosbag_path):
            self.get_logger().warning(
                f"Cannot save metadata: rosbag path does not exist: {rosbag_path}"
            )
            return

        try:
            metadata = {
                "robot_type": self.robot_type,
                "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "camera_topics": {
                    "cam_left": "/camera/left/image",
                    "cam_right": "/camera/right/image",
                },
                "state_topics": {
                    "follower_arm": "/follower/joint_states",
                },
                "action_topics": {
                    "leader_arm": "/leader/joint_trajectory",
                },
                "joint_order": {
                    "follower_arm": ["joint1", "joint2", "joint3"],
                    "leader_arm": ["joint1", "joint2", "joint3"],
                },
                "total_joint_order": self.total_joint_order,
            }

            metadata_path = os.path.join(rosbag_path, "robot_config.yaml")
            with open(metadata_path, "w", encoding="utf-8") as f:
                yaml.dump(metadata, f, default_flow_style=False, allow_unicode=True)

            self.get_logger().info(f"Saved robot config metadata to: {metadata_path}")
            self.metadata_saved = True

        except Exception as e:
            self.get_logger().error(f"Failed to save rosbag metadata: {e}")


class EncodingStatusPublisher(Node):
    """Publisher that simulates rosbag_recorder publishing encoding status."""

    def __init__(self):
        super().__init__("encoding_status_publisher")

        self.publisher = self.create_publisher(
            EncodingStatus, "rosbag_recorder/encoding_status", 10
        )

        self.get_logger().info("EncodingStatusPublisher initialized")

    def publish_encoding_complete(self, success: bool, bag_path: str, message: str):
        msg = EncodingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.success = success
        msg.bag_path = bag_path
        msg.message = message

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published encoding status: success={success}, bag_path={bag_path}"
        )


def main():
    rclpy.init()

    # Create test directory
    test_dir = tempfile.mkdtemp(prefix="rosbag_test_")
    print(f"Test directory: {test_dir}")

    # Create nodes
    mock_server = MockPhysicalAIServer(test_dir)
    publisher = EncodingStatusPublisher()

    # Create executor for both nodes
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    executor.add_node(mock_server)
    executor.add_node(publisher)

    # Spin in background
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Give time for connections
    time.sleep(0.5)

    print("\n=== Integration Test: Encoding Callback Mechanism ===\n")

    # Step 1: Simulate STOP transition (defer metadata saving)
    print("Step 1: Simulating STOP transition...")
    mock_server.simulate_stop_transition()
    assert mock_server._pending_metadata_rosbag_path == test_dir
    print("  - Pending metadata path set correctly")

    # Step 2: Verify metadata NOT saved yet
    metadata_path = os.path.join(test_dir, "robot_config.yaml")
    assert not os.path.exists(metadata_path), "Metadata should not be saved yet!"
    print("  - Verified: metadata NOT saved yet (deferred)")

    # Step 3: Publish encoding complete message
    print("\nStep 2: Publishing encoding complete message...")
    publisher.publish_encoding_complete(
        success=True, bag_path=test_dir, message="All videos encoded successfully"
    )

    # Wait for callback
    time.sleep(1.0)

    # Step 4: Verify callback was invoked
    print("\nStep 3: Verifying callback and metadata...")
    assert mock_server.callback_invoked, "Callback should have been invoked!"
    print("  - Callback was invoked")

    # Step 5: Verify metadata was saved
    assert mock_server.metadata_saved, "Metadata should have been saved!"
    assert os.path.exists(metadata_path), (
        f"robot_config.yaml should exist at {metadata_path}"
    )
    print(f"  - Metadata saved to: {metadata_path}")

    # Step 6: Verify pending path was cleared
    assert mock_server._pending_metadata_rosbag_path is None, (
        "Pending path should be cleared!"
    )
    print("  - Pending metadata path cleared")

    # Step 7: Read and verify metadata content
    import yaml

    with open(metadata_path, "r") as f:
        saved_metadata = yaml.safe_load(f)

    assert saved_metadata["robot_type"] == "test_robot"
    assert "camera_topics" in saved_metadata
    assert "state_topics" in saved_metadata
    assert "action_topics" in saved_metadata
    assert "joint_order" in saved_metadata
    print("  - Metadata content verified")

    print("\n=== All Integration Tests PASSED! ===\n")

    # Cleanup
    mock_server.destroy_node()
    publisher.destroy_node()
    executor.shutdown()
    rclpy.shutdown()

    # Cleanup test directory
    import shutil

    shutil.rmtree(test_dir)
    print(f"Cleaned up test directory: {test_dir}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
