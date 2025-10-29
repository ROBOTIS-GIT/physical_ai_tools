#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageRotatorNode(Node):
    """
    ROS2 node that subscribes to compressed images, rotates them 270 degrees,
    and publishes the rotated images back to a compressed topic.
    """
    
    def __init__(self):
        super().__init__('image_rotator_node')
        
        # Initialize CV bridge for converting between ROS messages and OpenCV images
        self.bridge = CvBridge()
        
        # Create subscriber for compressed images
        self.subscription = self.create_subscription(
            CompressedImage,
            'input_compressed_image',  # Input topic name
            self.image_callback,
            10
        )
        
        # Create publisher for rotated compressed images
        self.publisher = self.create_publisher(
            CompressedImage,
            'output_compressed_image',  # Output topic name
            10
        )
        
        # Get rotation angle from parameter (default: 270 degrees)
        self.declare_parameter('rotation_angle', 270)
        self.rotation_angle = self.get_parameter('rotation_angle').value
        
        self.get_logger().info(f'Image rotator node initialized with rotation angle: {self.rotation_angle} degrees')
        self.get_logger().info('Subscribing to: input_compressed_image')
        self.get_logger().info('Publishing to: output_compressed_image')
    
    def image_callback(self, msg):
        """
        Callback function that processes incoming compressed images.
        
        Args:
            msg (CompressedImage): The incoming compressed image message
        """
        try:
            # Convert compressed image to OpenCV format
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Rotate the image
            rotated_image = self.rotate_image(cv_image, self.rotation_angle)
            
            # Convert back to compressed image message
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(rotated_image, dst_format='jpg')
            
            # Copy header information from original message
            compressed_msg.header = msg.header
            compressed_msg.format = 'jpeg'
            
            # Publish the rotated image
            self.publisher.publish(compressed_msg)
            
            self.get_logger().debug(f'Processed and published rotated image: {msg.header.stamp}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def rotate_image(self, image, angle):
        """
        Rotate an image by the specified angle.
        
        Args:
            image: OpenCV image (numpy array)
            angle (int): Rotation angle in degrees (90, 180, or 270)
            
        Returns:
            numpy.ndarray: Rotated image
        """
        if angle == 90:
            # Rotate 90 degrees clockwise
            return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        elif angle == 180:
            # Rotate 180 degrees
            return cv2.rotate(image, cv2.ROTATE_180)
        elif angle == 270:
            # Rotate 270 degrees clockwise (equivalent to 90 degrees counter-clockwise)
            return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        else:
            # For other angles, use affine transformation
            height, width = image.shape[:2]
            center = (width // 2, height // 2)
            
            # Calculate rotation matrix
            rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
            
            # Perform rotation
            rotated = cv2.warpAffine(image, rotation_matrix, (width, height))
            return rotated


def main(args=None):
    """
    Main function to initialize and run the image rotator node.
    """
    rclpy.init(args=args)
    
    node = ImageRotatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
