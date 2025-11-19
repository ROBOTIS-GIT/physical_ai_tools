#!/usr/bin/env python3
"""
Odometry Publisher Example (CycloneDDS → ROS 2)

Publishes nav_msgs/Odometry messages to the /odom topic.

Usage:
    python odom_publisher.py
    ros2 topic echo /odom nav_msgs/msg/Odometry
"""

import time
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode

# === IDL message imports ===
from robotis_dds_python.robotis_dds_core.idl.std_msgs.msg import Header_
from robotis_dds_python.robotis_dds_core.idl.builtin_interfaces.msg import Time_
from robotis_dds_python.robotis_dds_core.idl.geometry_msgs.msg import (
    PoseWithCovariance_, TwistWithCovariance_,
    Pose_, Point_, Quaternion_, Twist_, Vector3_
)
from robotis_dds_python.robotis_dds_core.idl.nav_msgs.msg import Odometry_


def main():
    print("=== DDS Odometry Publisher Example ===")
    print("Publishing to /odom")
    print("ROS 2 subscribers can receive these messages with:")
    print("  ros2 topic echo /odom nav_msgs/msg/Odometry\n")
    print("Press Ctrl+C to stop\n")

    node = DDSNode(
        name="odom_publisher",
        domain_id=30,
        network_interface="auto",
        allow_multicast=True,
    )

    pub = node.dds_create_publisher("/odom", Odometry_)

    print("Publisher ready! Publishing odometry messages...\n")

    seq = 0
    try:
        while True:
            now = time.time()
            sec = int(now)
            nanosec = int((now - sec) * 1e9)
            header = Header_(
                stamp=Time_(sec=sec, nanosec=nanosec),
                frame_id="odom"
            )
            pose = PoseWithCovariance_(
                pose=Pose_(
                    position=Point_(x=seq * 0.1, y=0.0, z=0.0),
                    orientation=Quaternion_(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                covariance=[0.0] * 36,
            )

            twist = TwistWithCovariance_(
                twist=Twist_(
                    linear=Vector3_(x=0.1, y=0.0, z=0.0),
                    angular=Vector3_(x=0.0, y=0.0, z=0.0)
                ),
                covariance=[0.0] * 36
            )

            msg = Odometry_(
                header=header,
                child_frame_id="base_link",
                pose=pose,
                twist=twist,
            )

            pub.publish(msg)
            print(f"[Published] seq={seq} x={pose.pose.position.x:.2f}")

            seq += 1
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopping publisher...")

    finally:
        node.dds_destroy_node()
        print("Publisher stopped.")


if __name__ == "__main__":
    main()
