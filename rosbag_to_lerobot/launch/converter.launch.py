#!/usr/bin/env python3

"""
Launch file for the rosbag to LeRobot dataset converter.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""

    # Declare launch arguments
    rosbag_dir_arg = DeclareLaunchArgument(
        'rosbag_dir',
        default_value='/workspace/physical_ai_server/test/ffw_sg2_rev1_pickcoffeepat9',
        description='Directory containing rosbag episode data'
    )

    output_repo_id_arg = DeclareLaunchArgument(
        'output_repo_id',
        default_value='test/ffw_sg2_rev1_pickcoffeepat9_converted',
        description='Output repository ID for the LeRobot dataset'
    )

    task_name_arg = DeclareLaunchArgument(
        'task_name',
        default_value='pick_coffee_pat9',
        description='Task name for the dataset'
    )

    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='10',
        description='Frames per second for the dataset'
    )

    use_videos_arg = DeclareLaunchArgument(
        'use_videos',
        default_value='true',
        description='Whether to use video format for images'
    )

    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='mobile_robot',
        description='Robot type for the dataset'
    )

    # Camera rotation arguments
    cam_head_rotation_arg = DeclareLaunchArgument(
        'cam_head_rotation',
        default_value='0',
        description='Rotation angle for head camera (0, 90, 180, 270 degrees)'
    )

    cam_wrist_left_rotation_arg = DeclareLaunchArgument(
        'cam_wrist_left_rotation',
        default_value='270',
        description='Rotation angle for left wrist camera (0, 90, 180, 270 degrees)'
    )

    cam_wrist_right_rotation_arg = DeclareLaunchArgument(
        'cam_wrist_right_rotation',
        default_value='270',
        description='Rotation angle for right wrist camera (0, 90, 180, 270 degrees)'
    )

    # Create the node
    converter_node = Node(
        package='rosbag_to_lerobot',
        executable='rosbag_to_lerobot_converter',
        name='rosbag_to_lerobot_converter',
        parameters=[{
            'rosbag_dir': LaunchConfiguration('rosbag_dir'),
            'output_repo_id': LaunchConfiguration('output_repo_id'),
            'task_name': LaunchConfiguration('task_name'),
            'fps': LaunchConfiguration('fps'),
            'use_videos': LaunchConfiguration('use_videos'),
            'robot_type': LaunchConfiguration('robot_type'),
            'cam_head_rotation': LaunchConfiguration('cam_head_rotation'),
            'cam_wrist_left_rotation': LaunchConfiguration('cam_wrist_left_rotation'),
            'cam_wrist_right_rotation': LaunchConfiguration('cam_wrist_right_rotation'),
        }],
        output='screen'
    )

    return LaunchDescription([
        rosbag_dir_arg,
        output_repo_id_arg,
        task_name_arg,
        fps_arg,
        use_videos_arg,
        robot_type_arg,
        cam_head_rotation_arg,
        cam_wrist_left_rotation_arg,
        cam_wrist_right_rotation_arg,
        converter_node,
    ])
