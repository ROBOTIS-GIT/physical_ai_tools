#!/usr/bin/env python3

"""
ROS2 node to convert rosbag data to LeRobot dataset format.

This node reads rosbag files from a directory and converts them to the LeRobot dataset format.
The rosbag recorder saves episodes in subdirectories with episode indices.
"""

import os
import sys
import argparse
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import numpy as np
import cv2
import torch
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState, CompressedImage
from std_msgs.msg import String

# Add the lerobot path to sys.path
sys.path.append('/root/ros2_ws/src/lerobot/src')
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# Import our rosbag reader
from .rosbag_reader import read_episode_from_bag


class RosbagToLeRobotConverter(Node):
    """ROS2 node for converting rosbag data to LeRobot dataset format."""

    def __init__(self):
        super().__init__('rosbag_to_lerobot_converter')

        # Declare parameters
        self.declare_parameter('rosbag_dir', '/workspace/physical_ai_server/test/ffw_sg2_rev1_pickcoffeepat9')
        self.declare_parameter('output_repo_id', 'test/ffw_sg2_rev1_pickcoffeepat9_converted')
        self.declare_parameter('task_name', 'pick_coffee_pat9')
        self.declare_parameter('fps', 30)
        self.declare_parameter('use_videos', True)
        self.declare_parameter('robot_type', 'mobile_robot')

        # Get parameters
        self.rosbag_dir = Path(self.get_parameter('rosbag_dir').value)
        self.output_repo_id = self.get_parameter('output_repo_id').value
        self.task_name = self.get_parameter('task_name').value
        self.fps = self.get_parameter('fps').value
        self.use_videos = self.get_parameter('use_videos').value
        self.robot_type = self.get_parameter('robot_type').value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Joint names for the robot
        self.joint_names = [
            'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3', 'arm_l_joint4',
            'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7', 'gripper_l_joint1',
            'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3', 'arm_r_joint4',
            'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7', 'gripper_r_joint1',
            'head_joint1', 'head_joint2', 'lift_joint', 'linear_x', 'linear_y', 'angular_z'
        ]

        # Camera topics
        self.camera_topics = {
            'cam_head': '/zed/zed_node/left/image_rect_color/compressed',
            'cam_wrist_left': '/camera_left/camera_left/color/image_rect_raw/compressed',
            'cam_wrist_right': '/camera_right/camera_right/color/image_rect_raw/compressed'
        }

        # Joint state topics
        self.joint_state_topics = {
            'joints': '/joint_states',
            'odometry': '/odom'
        }

        # Leader action topics (example mapping; adjust as needed)
        self.action_topics = {
            'leader_joints_left': '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
            'leader_joints_right': '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
            'leader_head': '/leader/joystick_controller_left/joint_trajectory',
            'leader_lift': '/leader/joystick_controller_right/joint_trajectory',
            'leader_mobile': '/cmd_vel',
        }

        self.get_logger().info(f'RosbagToLeRobotConverter initialized')
        self.get_logger().info(f'Rosbag directory: {self.rosbag_dir}')
        self.get_logger().info(f'Output repo ID: {self.output_repo_id}')
        self.get_logger().info(f'Target frequency: {self.fps} Hz')

    def create_dataset_features(self, image_shapes: Dict[str, Tuple[int, int, int]] = None) -> Dict:
        """Create the features dictionary for the LeRobot dataset."""
        features = {
            "observation.state": {
                "dtype": "float32",
                "shape": (len(self.joint_names),),
                "names": self.joint_names,
            },
            "action": {
                "dtype": "float32",
                "shape": (len(self.joint_names),),
                "names": self.joint_names,
            },
        }

        # Add camera features with actual image shapes or defaults
        for camera_name in self.camera_topics.keys():
            if image_shapes and camera_name in image_shapes:
                # Use actual image shape
                height, width, channels = image_shapes[camera_name]
                features[f"observation.images.{camera_name}"] = {
                    "dtype": "video" if self.use_videos else "image",
                    "shape": (channels, height, width),
                    "names": ["channels", "height", "width"],
                }
            else:
                # Use default shape
                features[f"observation.images.{camera_name}"] = {
                    "dtype": "video" if self.use_videos else "image",
                    "shape": (3, 480, 640),  # Default shape
                    "names": ["channels", "height", "width"],
                }

        return features

    def determine_image_shapes(self) -> Dict[str, Tuple[int, int, int]]:
        """Determine actual image shapes from the first episode."""
        episode_dirs = self.find_episode_directories()
        if not episode_dirs:
            self.get_logger().warning("No episodes found to determine image shapes")
            return {}

        # Read the first episode to get image shapes
        first_episode_dir = episode_dirs[0]
        self.get_logger().info(f"Reading first episode {first_episode_dir} to determine image shapes")

        try:
            episode_data, num_frames = self.read_rosbag_episode(first_episode_dir)

            if num_frames == 0:
                self.get_logger().warning("No frames in first episode")
                return {}

            image_shapes = {}
            for camera_name, images in episode_data['images'].items():
                if images and len(images) > 0:
                    # Find the first non-None image
                    for img in images:
                        if img is not None:
                            height, width, channels = img.shape
                            image_shapes[camera_name] = (height, width, channels)
                            self.get_logger().info(f"Camera {camera_name}: {height}x{width}x{channels}")
                            break
                    else:
                        self.get_logger().warning(f"No valid images found for camera {camera_name}")
                else:
                    self.get_logger().warning(f"No images found for camera {camera_name}")

            return image_shapes

        except Exception as e:
            raise e
            self.get_logger().error(f"Error determining image shapes: {e}")
            return {}

    def find_episode_directories(self) -> List[Path]:
        """Find all episode directories in the rosbag directory."""
        if not self.rosbag_dir.exists():
            self.get_logger().error(f"Rosbag directory does not exist: {self.rosbag_dir}")
            return []

        episode_dirs = []
        for item in self.rosbag_dir.iterdir():
            if item.is_dir() and item.name.isdigit():
                episode_dirs.append(item)

        episode_dirs.sort(key=lambda x: int(x.name))
        self.get_logger().info(f"Found {len(episode_dirs)} episode directories")
        return episode_dirs

    def read_rosbag_episode(self, episode_dir: Path) -> Tuple[Dict, int]:
        """Read a single episode from rosbag directory."""
        self.get_logger().info(f"Reading episode from {episode_dir}")

        # Use our rosbag reader to extract data
        episode_data = read_episode_from_bag(
            episode_dir=episode_dir,
            joint_names=self.joint_names,
            camera_topics=self.camera_topics,
            joint_state_topics=self.joint_state_topics,
            action_topics=self.action_topics,
            fps=self.fps
        )

        num_frames = len(episode_data['joint_states'])
        self.get_logger().info(f"Read {num_frames} frames from episode (sampled at {self.fps} Hz)")

        if num_frames > 0:
            duration = episode_data['timestamps'][-1] - episode_data['timestamps'][0]
            self.get_logger().info(f"Episode duration: {duration:.2f} seconds")
            actual_fps = num_frames / duration if duration > 0 else 0
            self.get_logger().info(f"Actual sampling rate: {actual_fps:.2f} Hz")

        return episode_data, num_frames


    def create_lerobot_dataset(self, image_shapes: Dict[str, Tuple[int, int, int]] = None) -> LeRobotDataset:
        """Create a new LeRobot dataset."""
        features = self.create_dataset_features(image_shapes)

        # Remove existing dataset if it exists
        dataset_path = Path.home() / '.cache' / 'huggingface' / 'lerobot' / self.output_repo_id
        if dataset_path.exists():
            import shutil
            shutil.rmtree(dataset_path)
            self.get_logger().info(f"Removed existing dataset at {dataset_path}")

        dataset = LeRobotDataset.create(
            repo_id=self.output_repo_id,
            fps=self.fps,
            robot_type=self.robot_type,
            features=features,
            use_videos=self.use_videos,
            image_writer_processes=1,
            image_writer_threads=1,
        )

        self.get_logger().info(f"Created LeRobot dataset: {self.output_repo_id}")
        return dataset

    def convert_episode(self, dataset: LeRobotDataset, episode_dir: Path, episode_index: int):
        """Convert a single episode to LeRobot dataset format."""
        self.get_logger().info(f"Converting episode {episode_index} from {episode_dir}")

        episode_data, num_frames = self.read_rosbag_episode(episode_dir)

        if num_frames == 0:
            self.get_logger().warning(f"No data found in episode {episode_index}")
            return

        for i in range(num_frames):
            frame = {
                "observation.state": episode_data['joint_states'][i],
                "action": episode_data['actions'][i] if 'actions' in episode_data and len(episode_data['actions']) > i else episode_data['joint_states'][i],
            }

            # Add images
            for camera_name, images in episode_data['images'].items():
                if i < len(images) and images[i] is not None:
                    # Validate image shape consistency
                    img = images[i]
                    if hasattr(self, 'image_shapes') and camera_name in self.image_shapes:
                        expected_height, expected_width, expected_channels = self.image_shapes[camera_name]
                        actual_height, actual_width, actual_channels = img.shape
                        if (actual_height, actual_width, actual_channels) != (expected_height, expected_width, expected_channels):
                            self.get_logger().warning(
                                f"Image shape mismatch for camera {camera_name} at frame {i}: "
                                f"expected {expected_height}x{expected_width}x{expected_channels}, "
                                f"got {actual_height}x{actual_width}x{actual_channels}"
                            )
                    frame[f"observation.images.{camera_name}"] = img
                else:
                    error_msg = f"No image found for camera {camera_name} at frame {i}"
                    self.get_logger().warning(error_msg)
                    raise ValueError(error_msg)

            # Add frame to dataset
            dataset.add_frame(frame, task=self.task_name)

        # Save the episode
        dataset.save_episode()
        self.get_logger().info(f"Saved episode {episode_index} with {num_frames} frames")

    def convert_all_episodes(self):
        """Convert all episodes from rosbag directory to LeRobot dataset."""
        episode_dirs = self.find_episode_directories()

        if not episode_dirs:
            self.get_logger().error("No episode directories found")
            return

        # Determine image shapes from first episode
        self.image_shapes = self.determine_image_shapes()
        if self.image_shapes:
            self.get_logger().info(f"Determined image shapes: {self.image_shapes}")
        else:
            self.get_logger().warning("Could not determine image shapes, using defaults")

        # Create the dataset with proper image shapes
        dataset = self.create_lerobot_dataset(self.image_shapes)

        # Convert each episode
        for episode_dir in episode_dirs:
            episode_index = int(episode_dir.name)
            self.convert_episode(dataset, episode_dir, episode_index)
            if episode_index == 1:
                break

        # Consolidate the dataset
        self.get_logger().info("Dataset conversion completed")

        # Optionally push to hub
        # dataset.push_to_hub()

    def run(self):
        """Main run method."""
        self.get_logger().info("Starting rosbag to LeRobot dataset conversion")
        self.convert_all_episodes()
        self.get_logger().info("Conversion completed")


def main(args=None):
    rclpy.init(args=args)

    converter = RosbagToLeRobotConverter()

    try:
        converter.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback
        traceback.print_exc()
    finally:
        converter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
