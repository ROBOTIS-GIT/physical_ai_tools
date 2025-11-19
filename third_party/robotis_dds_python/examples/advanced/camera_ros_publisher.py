#!/usr/bin/env python3
"""
ROS2 → DDS: Receive sensor_msgs/CompressedImage and display

python3 camera_ros_publisher.py
python3 camera_subscriber.py
"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_ros_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/camera/image/compressed', 10)
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("❌ Cannot open webcam")
            exit(1)

        self.get_logger().info("📸 ROS2 CompressedImage Publisher started")

        self.timer = self.create_timer(0.1, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        success, buffer = cv2.imencode('.jpg', frame)
        if not success:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.format = 'jpeg'
        msg.data = buffer.tobytes()

        self.publisher_.publish(msg)
        self.get_logger().info(f"[ROS→DDS] Published CompressedImage ({len(msg.data)} bytes)")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
