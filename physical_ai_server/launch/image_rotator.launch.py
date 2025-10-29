#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for the image rotator nodes.
    
    This launch file starts two image rotator nodes for ZED camera left and right feeds:
    - rotation_angle: The angle to rotate images (default: 270 degrees)
    - left_cam_input: Left camera input topic (default: /camera_left/camera_left/color/image_rect_raw/compressed)
    - left_cam_output: Left camera output topic (default: /camera_left/camera_left/color/image_rect_raw/rotated/compressed)
    - right_cam_input: Right camera input topic (default: /camera_right/camera_right/color/image_rect_raw/compressed)
    - right_cam_output: Right camera output topic (default: /camera_right/camera_right/color/image_rect_raw/rotated/compressed)
    """
    
    # Declare launch arguments
    rotation_angle_arg = DeclareLaunchArgument(
        'rotation_angle',
        default_value='270',
        description='Rotation angle in degrees (90, 180, or 270)'
    )
    left_cam_input_arg = DeclareLaunchArgument(
        'left_cam_input',
        default_value='/camera_left/camera_left/color/image_rect_raw/compressed',
        description='Left camera input compressed image topic name'
    )
    
    left_cam_output_arg = DeclareLaunchArgument(
        'left_cam_output',
        default_value='/camera_left/camera_left/color/image_rect_raw/rotated/compressed',
        description='Left camera output compressed image topic name'
    )
    
    right_cam_input_arg = DeclareLaunchArgument(
        'right_cam_input',
        default_value='/camera_right/camera_right/color/image_rect_raw/compressed',
        description='Right camera input compressed image topic name'
    )
    
    right_cam_output_arg = DeclareLaunchArgument(
        'right_cam_output',
        default_value='/camera_right/camera_right/color/image_rect_raw/rotated/compressed',
        description='Right camera output compressed image topic name'
    )
    
    # Create the left camera image rotator node
    left_cam_rotator_node = Node(
        package='physical_ai_server',
        executable='image_rotator_node',
        name='left_cam_image_rotator',
        output='screen',
        parameters=[{
            'rotation_angle': LaunchConfiguration('rotation_angle'),
        }],
        remappings=[
            ('input_compressed_image', LaunchConfiguration('left_cam_input')),
            ('output_compressed_image', LaunchConfiguration('left_cam_output')),
        ]
    )
    
    # Create the right camera image rotator node
    right_cam_rotator_node = Node(
        package='physical_ai_server',
        executable='image_rotator_node',
        name='right_cam_image_rotator',
        output='screen',
        parameters=[{
            'rotation_angle': LaunchConfiguration('rotation_angle'),
        }],
        remappings=[
            ('input_compressed_image', LaunchConfiguration('right_cam_input')),
            ('output_compressed_image', LaunchConfiguration('right_cam_output')),
        ]
    )
    
    return LaunchDescription([
        rotation_angle_arg,
        left_cam_input_arg,
        left_cam_output_arg,
        right_cam_input_arg,
        right_cam_output_arg,
        left_cam_rotator_node,
        right_cam_rotator_node,
    ])
