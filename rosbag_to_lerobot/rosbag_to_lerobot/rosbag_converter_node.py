#!/usr/bin/env python3

"""
ROS2 node to convert rosbag data to LeRobot dataset format.

This node reads rosbag files from a directory and converts them to the LeRobot dataset format.
The rosbag recorder saves episodes in subdirectories with episode indices.
"""

import os
import gc
import sys
import argparse
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
import numpy as np
import cv2
import torch
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor
import yaml

from sensor_msgs.msg import JointState, CompressedImage
from std_msgs.msg import String

from lerobot.datasets.lerobot_dataset import LeRobotDataset

from physical_ai_server.data_processing.lerobot_dataset_wrapper import LeRobotDatasetWrapper

# Import our rosbag reader
from .rosbag_reader import read_episode_from_bag

import time


class RosbagToLeRobotConverter(Node):
    """ROS2 node for converting rosbag data to LeRobot dataset format."""

    def __init__(self):
        super().__init__('rosbag_to_lerobot_converter')

        # Declare parameters without defaults, using dynamic typing to avoid warnings
        dyn = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('rosbag_dir', descriptor=dyn)
        self.declare_parameter('output_repo_id', descriptor=dyn)
        self.declare_parameter('task_name', descriptor=dyn)
        self.declare_parameter('fps', descriptor=dyn)
        self.declare_parameter('use_videos', descriptor=dyn)
        self.declare_parameter('robot_type', descriptor=dyn)
        self.declare_parameter('camera_topics', descriptor=dyn)
        self.declare_parameter('camera_rotations', descriptor=dyn)
        self.declare_parameter('joint_state_topics', descriptor=dyn)
        self.declare_parameter('action_topics', descriptor=dyn)
        self.declare_parameter('joint_names', descriptor=dyn)
        self.declare_parameter('joint_order', descriptor=dyn)
        self.declare_parameter('config_yaml_path', descriptor=dyn)
        self.declare_parameter('use_optimized_save_mode', descriptor=dyn)

        # Helper parsers to support both YAML dict/list and JSON strings
        def parse_dict_param(param_name: str, required: bool = True) -> Dict[str, Any]:
            value = self.get_parameter(param_name).value
            if value is None:
                # Try to fallback to reading from YAML file if provided
                config_yaml_path = self.get_parameter('config_yaml_path').value
                if config_yaml_path and os.path.isfile(config_yaml_path):
                    try:
                        with open(config_yaml_path, 'r') as f:
                            data = yaml.safe_load(f) or {}
                        # ROS2 YAML structure: <node_name>: ros__parameters: {...}
                        node_name = self.get_name()
                        node_section = data.get(node_name, {})
                        ros_params = node_section.get('ros__parameters', {})
                        fallback = ros_params.get(param_name)
                        if isinstance(fallback, dict):
                            return fallback
                    except Exception as e:
                        pass
                if required:
                    raise ValueError(f"Missing required parameter: {param_name}")
                return {}

        def parse_list_param(param_name: str, required: bool = True) -> List[Any]:
            value = self.get_parameter(param_name).value
            if value is None:
                # Try to fallback to reading from YAML file if provided
                config_yaml_path = self.get_parameter('config_yaml_path').value
                if config_yaml_path and os.path.isfile(config_yaml_path):
                    try:
                        with open(config_yaml_path, 'r') as f:
                            data = yaml.safe_load(f) or {}
                        node_name = self.get_name()
                        node_section = data.get(node_name, {})
                        ros_params = node_section.get('ros__parameters', {})
                        fallback = ros_params.get(param_name)
                        if isinstance(fallback, list):
                            return fallback
                    except Exception:
                        pass
                if required:
                    raise ValueError(f"Missing required parameter: {param_name}")
                return []

        # Get parameters and validate them
        self.rosbag_dir = Path(self.get_parameter('rosbag_dir').value)
        self.output_repo_id = self.get_parameter('output_repo_id').value
        self.task_name = self.get_parameter('task_name').value
        self.fps = self.get_parameter('fps').value
        self.use_videos = self.get_parameter('use_videos').value
        self.robot_type = self.get_parameter('robot_type').value
        self.use_optimized_save_mode = self.get_parameter('use_optimized_save_mode').value

        # Validate basic parameters
        if not self.rosbag_dir.exists():
            raise ValueError(f"Rosbag directory does not exist: {self.rosbag_dir}")
        if not self.output_repo_id:
            raise ValueError("output_repo_id parameter is required")
        if not self.task_name:
            raise ValueError("task_name parameter is required")
        if self.fps <= 0:
            raise ValueError(f"fps must be positive, got: {self.fps}")
        if not isinstance(self.use_videos, bool):
            raise ValueError(f"use_videos must be boolean, got: {self.use_videos}")
        if not self.robot_type:
            raise ValueError("robot_type parameter is required")
        if not isinstance(self.use_optimized_save_mode, bool):
            raise ValueError(f"use_optimized_save_mode must be boolean, got: {self.use_optimized_save_mode}")

        # Parse and validate camera rotation parameters (dict)
        self.camera_rotations = parse_dict_param('camera_rotations', required=False)

        # Parse and validate camera topics parameters (dict)
        self.camera_topics = parse_dict_param('camera_topics', required=True)
        if not self.camera_topics:
            raise ValueError("camera_topics cannot be empty")

        # Parse and validate joint state topics parameters (dict)
        self.joint_state_topics = parse_dict_param('joint_state_topics', required=True)
        if not self.joint_state_topics:
            raise ValueError("joint_state_topics cannot be empty")

        # Parse and validate action topics parameters (dict)
        self.action_topics = parse_dict_param('action_topics', required=True)
        if not self.action_topics:
            raise ValueError("action_topics cannot be empty")

        # Prefer joint_names (list). If absent, derive from joint_order (nested dict of lists)
        joint_names_param = self.get_parameter('joint_names').value
        if joint_names_param is not None:
            # Allow list or JSON string
            if isinstance(joint_names_param, list):
                self.joint_names = joint_names_param
            elif isinstance(joint_names_param, str):
                import json
                try:
                    self.joint_names = json.loads(joint_names_param)
                except json.JSONDecodeError as e:
                    raise ValueError(f"Invalid JSON for joint_names: {e}")
            else:
                raise ValueError(f"joint_names must be a list or JSON string, got: {type(joint_names_param)}")
            if not self.joint_names:
                raise ValueError("joint_names cannot be empty")
        else:
            # Fallback to nested joint_order mapping
            joint_order = parse_dict_param('joint_order', required=True)
            ordered_names: List[str] = []
            for group_name, names in joint_order.items():
                if not isinstance(names, list) or not all(isinstance(n, str) and n.strip() for n in names):
                    raise ValueError(f"joint_order['{group_name}'] must be a non-empty list of strings")
                ordered_names.extend(names)
            if not ordered_names:
                raise ValueError("Derived joint_names from joint_order is empty")
            self.joint_names = ordered_names

        # Validate rotation parameters
        for camera_name, rotation in self.camera_rotations.items():
            if camera_name not in self.camera_topics:
                raise ValueError(f"Camera '{camera_name}' in camera_rotations not found in camera_topics")
            if rotation not in [0, 90, 180, 270]:
                raise ValueError(f"Invalid rotation for {camera_name}: {rotation}. Must be 0, 90, 180, or 270 degrees")

        # Validate that all camera topics are valid ROS topic names
        for camera_name, topic in self.camera_topics.items():
            if not topic.startswith('/'):
                raise ValueError(f"Invalid ROS topic for camera '{camera_name}': {topic}. Must start with '/'")

        # Validate that all joint state topics are valid ROS topic names
        for topic_name, topic in self.joint_state_topics.items():
            if not topic.startswith('/'):
                raise ValueError(f"Invalid ROS topic for joint state '{topic_name}': {topic}. Must start with '/'")

        # Validate that all action topics are valid ROS topic names
        for action_name, topic in self.action_topics.items():
            if not topic.startswith('/'):
                raise ValueError(f"Invalid ROS topic for action '{action_name}': {topic}. Must start with '/'")

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Validate that all joint names are non-empty strings
        for joint_name in self.joint_names:
            if not isinstance(joint_name, str) or not joint_name.strip():
                raise ValueError(f"Invalid joint name: {joint_name}. Must be a non-empty string")

        self.get_logger().info(f'RosbagToLeRobotConverter initialized')
        self.get_logger().info(f'Rosbag directory: {self.rosbag_dir}')
        self.get_logger().info(f'Output repo ID: {self.output_repo_id}')
        self.get_logger().info(f'Task name: {self.task_name}')
        self.get_logger().info(f'Target frequency: {self.fps} Hz')
        self.get_logger().info(f'Use videos: {self.use_videos}')
        self.get_logger().info(f'Robot type: {self.robot_type}')
        self.get_logger().info(f'Configured cameras: {list(self.camera_topics.keys())}')
        for camera_name, topic in self.camera_topics.items():
            rotation = self.camera_rotations.get(camera_name, 0)
            self.get_logger().info(f'  {camera_name}: {topic} (rotation: {rotation}°)')
        self.get_logger().info(f'Joint state topics: {list(self.joint_state_topics.keys())}')
        self.get_logger().info(f'Action topics: {list(self.action_topics.keys())}')
        self.get_logger().info(f'Joint names ({len(self.joint_names)}): {self.joint_names}')

    def rotate_image(self, image: np.ndarray, camera_name: str) -> np.ndarray:
        """Rotate image by the specified angle for the given camera."""
        rotation = self.camera_rotations.get(camera_name, 0)
        if rotation == 0:
            return image
        elif rotation == 90:
            return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        elif rotation == 180:
            return cv2.rotate(image, cv2.ROTATE_180)
        elif rotation == 270:
            return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        else:
            return image  # Should not happen due to validation

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

        # Add camera features with actual image shapes or defaults, accounting for rotation
        for camera_name in self.camera_topics.keys():
            if image_shapes and camera_name in image_shapes:
                # Use actual image shape
                height, width, channels = image_shapes[camera_name]
                # Apply rotation to dimensions for this specific camera
                rotation = self.camera_rotations.get(camera_name, 0)
                if rotation in [90, 270]:
                    # Width and height swap for 90° and 270° rotations
                    height, width = width, height
                # 0° and 180° rotations don't change dimensions
                features[f"observation.images.{camera_name}"] = {
                    "dtype": "video" if self.use_videos else "image",
                    "shape": (channels, height, width),
                    "names": ["channels", "height", "width"],
                }
            else:
                raise ValueError(f"Image shape not found for camera {camera_name}")

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
        else:
            raise ValueError(f"No frames in episode {episode_dir}")

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

        dataset = LeRobotDatasetWrapper.create(
            repo_id=self.output_repo_id,
            fps=self.fps,
            robot_type=self.robot_type,
            features=features,
            use_videos=self.use_videos,
        )
        dataset.set_robot_type(self.robot_type)

        self.get_logger().info(f"Created LeRobot dataset: {self.output_repo_id}")
        return dataset

    def convert_episode(self, dataset: LeRobotDataset, episode_dir: Path, episode_index: int):
        """Convert a single episode to LeRobot dataset format."""
        self.get_logger().info(f"Converting episode {episode_index} from {episode_dir}")

        episode_data, num_frames = self.read_rosbag_episode(episode_dir)

        if num_frames == 0:
            raise ValueError(f"No frames in episode {episode_dir}")

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
                    # Apply rotation if specified for this camera
                    img = self.rotate_image(img, camera_name)
                    frame[f"observation.images.{camera_name}"] = img
                else:
                    error_msg = f"No image found for camera {camera_name} at frame {i}"
                    self.get_logger().warning(error_msg)
                    raise ValueError(error_msg)

            # Add frame to dataset using optimized or standard mode
            if self.use_optimized_save_mode:
                dataset.add_frame_without_write_image(frame, task=self.task_name)
            else:
                dataset.add_frame(frame, task=self.task_name)

        # Save the episode using optimized or standard mode
        if self.use_optimized_save_mode:
            dataset.save_episode_without_write_image()
            while not dataset.check_video_encoding_completed():
                self.get_logger().info("Waiting for video encoding to complete")
                time.sleep(0.1)
        else:
            dataset.save_episode()
        self.get_logger().info(f"Saved episode {episode_index} with {num_frames} frames")
        self._episode_reset(dataset)

    def convert_all_episodes(self):
        """Convert all episodes from rosbag directory to LeRobot dataset."""
        episode_dirs = self.find_episode_directories()

        if not episode_dirs:
            raise ValueError("No episode directories found")

        # Determine image shapes from first episode
        self.image_shapes = self.determine_image_shapes()
        if self.image_shapes:
            self.get_logger().info(f"Determined image shapes: {self.image_shapes}")
        else:
            raise ValueError("Could not determine image shapes")

        # Create the dataset with proper image shapes
        dataset = self.create_lerobot_dataset(self.image_shapes)

        # Log the save mode being used
        if self.use_optimized_save_mode:
            self.get_logger().info("Using optimized save mode: images will be kept in RAM and encoded at the end")
        else:
            self.get_logger().info("Using standard save mode: images will be written to disk immediately")

        # Convert each episode
        for episode_dir in episode_dirs:
            episode_index = int(episode_dir.name)
            self.convert_episode(dataset, episode_dir, episode_index)

        # Consolidate the dataset
        self.get_logger().info("Dataset conversion completed")

        # Optionally push to hub
        # dataset.push_to_hub()

    def run(self):
        """Main run method."""
        self.get_logger().info("Starting rosbag to LeRobot dataset conversion")
        self.convert_all_episodes()
        self.get_logger().info("Conversion completed")

    def _episode_reset(self, dataset: LeRobotDataset) -> None:
        """Clear dataset's in-memory episode buffer after saving, freeing RAM.

        Mirrors physical_ai_server DataManager._episode_reset behavior.
        """
        if dataset.episode_buffer is not None:
            for key, value in dataset.episode_buffer.items():
                if isinstance(value, list):
                    value.clear()
                del value
            dataset.episode_buffer.clear()
        dataset.episode_buffer = None
        gc.collect()


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
