#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DDS Publisher Example — Publish RAW camera frames as sensor_msgs/Image (bgr8)
Compatible with RobotisDDSSDK.get_image()

Usage:
    python3 examples/sdk/image_publisher_raw.py
"""

import cv2
import time
import numpy as np
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode
from robotis_dds_python.robotis_dds_core.idl.sensor_msgs.msg import Image_
from robotis_dds_python.robotis_dds_core.idl.std_msgs.msg import Header_
from robotis_dds_python.robotis_dds_core.idl.builtin_interfaces.msg import Time_


def make_stamp():
    """Convert current time to ROS2 Time_ structure"""
    now = time.time()
    sec = int(now)
    nanosec = int((now - sec) * 1e9)
    return Time_(sec=sec, nanosec=nanosec)


def main():
    # === Create DDS Node ===
    node = DDSNode(name="image_publisher_raw", domain_id=30)
    pub = node.dds_create_publisher("/camera/image", Image_)

    # === Open Camera ===
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("❌ Failed to open camera. Check device connection or permissions.")

    # === Camera Settings (640x480 @ 30fps) ===
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS,          30)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    print("🧩 Publishing RAW frames on /camera/image (sensor_msgs/Image)...  [Ctrl+C to stop]")

    frame_count = 0
    fps_interval = 1.0 / 60.0  # target: 30 FPS
    last_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            frame_count += 1
            height, width, _ = frame.shape

            # === Message Metadata ===
            encoding = "bgr8"
            step = width * 3
            header = Header_(stamp=make_stamp(), frame_id="camera")

            # === Create sensor_msgs/Image ===
            msg = Image_(
                header=header,
                height=height,
                width=width,
                encoding=encoding,
                is_bigendian=0,
                step=step,
                data=frame.tobytes(),
            )

            # === DDS Publish ===
            pub.publish(msg)

            now = time.time()
            if now - last_time >= 1.0:
                print(f"📸 Sent frame #{frame_count} ({width}x{height})")
                last_time = now

            # === Maintain FPS (30fps) ===
            time.sleep(fps_interval)

    except KeyboardInterrupt:
        print("\n🧩 Stopped publishing (KeyboardInterrupt).")

    finally:
        cap.release()
        node.dds_destroy_node()
        print("✅ Camera released & DDS node destroyed.")


if __name__ == "__main__":
    main()
