#!/usr/bin/env python3
"""
DDS → ROS2: Publish camera frames as sensor_msgs/CompressedImage

python3 image_publisher.py
ros2 topic echo /camera/image/compressed sensor_msgs/msg/CompressedImage
rqt_image_view /camera/image/compressed
"""

import cv2
import time
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode
from robotis_dds_python.robotis_dds_core.idl.sensor_msgs.msg import CompressedImage_
from robotis_dds_python.robotis_dds_core.idl.std_msgs.msg import Header_
from robotis_dds_python.robotis_dds_core.idl.builtin_interfaces.msg import Time_

node = DDSNode(name="image_publisher", domain_id=30)
pub = node.dds_create_publisher("/camera/image/compressed", CompressedImage_)

cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    _, buffer = cv2.imencode(".jpg", frame)
    now = time.time()
    sec, nanosec = int(now), int((now - int(now)) * 1e9)

    header = Header_(stamp=Time_(sec=sec, nanosec=nanosec), frame_id="camera")
    msg = CompressedImage_(header=header, format="jpeg", data=buffer.tobytes())

    pub.publish(msg)
    print(f"[Published frame] size={len(buffer)} bytes")
    time.sleep(0.01)
