#!/usr/bin/env python3
"""
Odometry Subscriber Example (ROS 2 → DDS)

Receives nav_msgs/Odometry messages published from ROS 2.
Example:
    python odom_subscriber.py

Test with ROS 2:
    ros2 topic pub /odom nav_msgs/msg/Odometry "{header: {frame_id: 'odom'}, child_frame_id: 'base_link', pose: {pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
"""

from robotis_dds_python.robotis_dds_core.idl.nav_msgs.msg import Odometry_
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode


def main():
    """Run DDS Odometry subscriber example."""
    print("=== DDS Odometry Subscriber Example ===")
    print("Listening to /odom topic from ROS 2...")
    print("Test with ROS 2:")
    print("  ros2 topic pub /odom nav_msgs/msg/Odometry \"{header: {frame_id: 'odom'}, child_frame_id: 'base_link', pose: {pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}\"")
    print("\nPress Ctrl+C to stop.\n")

    node = DDSNode(
        name="odom_subscriber",
        domain_id=30,
        network_interface="auto",
        allow_multicast=True,
    )

    def callback(msg: Odometry_):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        print(
            f"[Received /odom]\n"
            f"  x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}\n"
            f"  vx={vel.x:.2f}, vy={vel.y:.2f}, vz={vel.z:.2f}\n"
            f"  frame_id={msg.header.frame_id}, child={msg.child_frame_id}\n"
        )

    node.dds_create_subscription("/odom", Odometry_, callback)

    print("Subscriber ready! Waiting for Odometry messages...\n")

    try:
        node.dds_spin()
    except KeyboardInterrupt:
        print("\nStopping subscriber...")
    finally:
        node.dds_destroy_node()
        print("Subscriber stopped.")


if __name__ == "__main__":
    main()
