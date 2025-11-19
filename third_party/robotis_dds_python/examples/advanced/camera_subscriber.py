#!/usr/bin/env python3
"""
ROS2 → DDS: Receive sensor_msgs/CompressedImage and display

ros2 run image_publisher image_publisher_node
python3 camera_ros_publisher.py
python3 camera_subscriber.py
"""

import numpy as np
import cv2
import matplotlib.pyplot as plt
from robotis_dds_python.robotis_dds_core.idl.sensor_msgs.msg import CompressedImage_
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode

def main():
    print("=== DDS Image Subscriber ===")
    print("Listening to /camera/image/compressed from ROS2")
    print("Press Ctrl+C to stop.\n")
# init
    node = DDSNode(
        name="camera_subscriber",
        domain_id=30,
        network_interface="auto",
        allow_multicast=True,
    )
    node.dds_create_subscription("/camera/image/compressed", CompressedImage_, image_callback)
    spin 

    self.images = Dict

    def image_callback(msg: CompressedImage_):
        data_bytes = bytes(msg.data) if isinstance(msg.data, list) else msg.data
        img = np.frombuffer(data_bytes, dtype=np.uint8)
        frame = cv2.imdecode(img, cv2.IMREAD_COLOR)

        images = {'topic_name': frame}

        if frame is not None:
            plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            plt.axis('off')
            plt.pause(0.001)
    
    def get_images():
        return self.images

    

    try:
        node.dds_spin()
    except KeyboardInterrupt:
        print("\n🛑 KeyboardInterrupt received. Closing windows...")
        plt.close('all')   
    finally:
        cv2.destroyAllWindows()
        plt.close('all') 
        node.dds_destroy_node()
        print("✅ Subscriber stopped cleanly.")

if __name__ == "__main__":
    main()
