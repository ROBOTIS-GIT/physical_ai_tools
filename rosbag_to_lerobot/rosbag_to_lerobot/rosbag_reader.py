#!/usr/bin/env python3

"""
Rosbag reader module for extracting data from ROS2 bag files using MCAP format.
"""

from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
import numpy as np
from cv_bridge import CvBridge

# Message types
from sensor_msgs.msg import JointState, CompressedImage, Image
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist

import rosbag2_py
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


import traceback


class RosbagReader:
    """Reader for ROS2 bag files using MCAP format."""

    def __init__(self, fps: float = 30.0):
        self.bridge = CvBridge()
        self.fps = fps

    def read_bag_file(self, bag_path: Path) -> Dict[str, List[Any]]:
        """
        Read a ROS2 bag file and extract messages using MCAP format.

        Args:
            bag_path: Path to the bag file (can be .mcap file or directory)

        Returns:
            Dictionary containing lists of messages for each topic
        """
        if not bag_path.exists():
            raise FileNotFoundError(f"Bag file not found: {bag_path}")

        messages = {
            'joint_states': {},
            'images': {},
            'actions': {},
            'timestamps': []
        }

        # Configure storage options for MCAP
        storage_options = StorageOptions(
            uri=str(bag_path),
            storage_id="mcap"  # Use MCAP format instead of sqlite3
        )

        # Configure converter options
        converter_options = ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )

        # Create reader
        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        # Get topic types
        topic_types = reader.get_all_topics_and_types()
        topic_type_map = {topic.name: topic.type for topic in topic_types}

        # Read messages
        while reader.has_next():
            msg_tuple = reader.read_next()
            topic_name = msg_tuple[0]  # First element is topic name
            msg = msg_tuple[1]    # Second element is message data
            timestamp_ns = msg_tuple[2]  # Third element is timestamp in nanoseconds
            msg_type = get_message(topic_type_map[topic_name])
            msg_data = deserialize_message(msg, msg_type)

            # Convert timestamp from nanoseconds to seconds
            current_timestamp = timestamp_ns / 1e9

            # Check message type using isinstance for proper type checking
            if isinstance(msg_data, JointState):
                if topic_name not in messages['joint_states']:
                    messages['joint_states'][topic_name] = []
                messages['joint_states'][topic_name].append((current_timestamp, msg_data))

            elif isinstance(msg_data, Odometry):
                if topic_name not in messages['joint_states']:
                    messages['joint_states'][topic_name] = []
                messages['joint_states'][topic_name].append((current_timestamp, msg_data))

            elif isinstance(msg_data, JointTrajectory):
                if topic_name not in messages['actions']:
                    messages['actions'][topic_name] = []
                messages['actions'][topic_name].append((current_timestamp, msg_data))

            elif isinstance(msg_data, Twist):
                if topic_name not in messages['actions']:
                    messages['actions'][topic_name] = []
                messages['actions'][topic_name].append((current_timestamp, msg_data))

            elif isinstance(msg_data, CompressedImage):
                if topic_name not in messages['images']:
                    messages['images'][topic_name] = []
                messages['images'][topic_name].append((current_timestamp, msg_data))

            elif isinstance(msg_data, Image):
                if topic_name not in messages['images']:
                    messages['images'][topic_name] = []
                messages['images'][topic_name].append((current_timestamp, msg_data))

            else:
                raise ValueError(f"Unknown message type: {type(msg_data)}")

        return messages

    def extract_joint_state(self, msg: JointState, joint_names: List[str]) -> np.ndarray:
        """
        Extract joint positions from a JointState message.

        Args:
            msg: JointState message
            joint_names: List of joint names to extract

        Returns:
            Array of joint positions in the order of joint_names
        """
        joint_positions = np.zeros(len(joint_names), dtype=np.float32)

        if msg.name and msg.position:
            for i, joint_name in enumerate(joint_names):
                try:
                    idx = msg.name.index(joint_name)
                    joint_positions[i] = msg.position[idx]
                except ValueError:
                    # print(f'{joint_name} not found in msg {msg}')
                    pass

        return joint_positions

    def extract_odometry_velocity(self, msg: Odometry, velocity_names: List[str]) -> np.ndarray:
        """
        Extract velocity data from an Odometry message.

        Args:
            msg: Odometry message
            velocity_names: List of velocity names to extract (e.g., ['linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z'])

        Returns:
            Array of velocity values in the order of velocity_names
        """
        velocity_values = np.zeros(len(velocity_names), dtype=np.float32)

        # Extract linear velocity (x, y, z)
        linear_vel = msg.twist.twist.linear
        # Extract angular velocity (x, y, z)
        angular_vel = msg.twist.twist.angular

        for i, vel_name in enumerate(velocity_names):
            if vel_name == 'linear_x':
                velocity_values[i] = linear_vel.x
            elif vel_name == 'linear_y':
                velocity_values[i] = linear_vel.y
            elif vel_name == 'linear_z':
                velocity_values[i] = linear_vel.z
            elif vel_name == 'angular_x':
                velocity_values[i] = angular_vel.x
            elif vel_name == 'angular_y':
                velocity_values[i] = angular_vel.y
            elif vel_name == 'angular_z':
                velocity_values[i] = angular_vel.z
            else:
                # print(f'{vel_name} not found in msg {msg}')
                pass

        return velocity_values

    def extract_image(self, msg: Image) -> np.ndarray:
        """
        Extract image from Image message.

        Args:
            msg: Image message

        Returns:
            Numpy array of the image (H, W, C)
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            return cv_image
        except Exception as e:
            print(f"Error converting image: {e}")
            return None

    def extract_compressed_image(self, msg: CompressedImage) -> np.ndarray:
        """
        Extract image from CompressedImage message.

        Args:
            msg: CompressedImage message

        Returns:
            Numpy array of the image (H, W, C)
        """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='rgb8')
            return cv_image
        except Exception as e:
            print(f"Error converting compressed image: {e}")
            return None

    def extract_joint_trajectory(self, msg: JointTrajectory, joint_names: List[str]) -> Dict[str, float]:
        """Extract desired positions from JointTrajectory (take last point) mapped by joint name."""
        action_map: Dict[str, float] = {}
        # print(f"msg: {msg}")
        # print(f"joint_names: {joint_names}")
        if msg.points and msg.joint_names:
            point = msg.points[-1]
            for name in msg.joint_names:
                if name in joint_names:
                    idx = msg.joint_names.index(name)
                    if idx < len(point.positions):
                        action_map[name] = float(point.positions[idx])
        return action_map

    def extract_twist(self, msg: Twist, velocity_names: List[str]) -> Dict[str, float]:
        values: Dict[str, float] = {}
        for vel in velocity_names:
            if vel == 'linear_x':
                values[vel] = float(msg.linear.x)
            elif vel == 'linear_y':
                values[vel] = float(msg.linear.y)
            elif vel == 'angular_z':
                values[vel] = float(msg.angular.z)
        return values

    def synchronize_data(self,
                        joint_states: Dict[str, List[Tuple[float, np.ndarray]]],
                        images: Dict[str, List[Tuple[float, np.ndarray]]],
                        joint_names: List[str],
                        actions: Dict[str, List[Tuple[float, np.ndarray]]],
                        target_fps: float = 30.0) -> Dict[str, List[np.ndarray]]:
        """
        Synchronize joint states, images, and leader actions by timestamp and sample at target frequency.
        Takes the latest data that is not in the future relative to the target timestamp.
        Merges joint states from different sources into a single combined state.
        """
        synchronized_data = {
            'joint_states': [],
            'images': {cam: [] for cam in images.keys()},
            'actions': [],
            'timestamps': []
        }

        if not joint_states:
            raise ValueError(f"No joint state data found for synchronization")
        if not images:
            raise ValueError(f"No image data found for synchronization")
        if not actions:
            raise ValueError(f"No action data found for synchronization")

        # Sort all data by timestamp
        for state_name in joint_states:
            joint_states[state_name].sort(key=lambda x: x[0])
        for cam in images:
            images[cam].sort(key=lambda x: x[0])
        for name in actions:
            actions[name].sort(key=lambda x: x[0])

        # Find the time range from all joint state sources
        all_timestamps = []
        for state_data in joint_states.values():
            if state_data:
                all_timestamps.extend([ts for ts, _ in state_data])
        if not all_timestamps:
            raise ValueError(f"No timestamps found for synchronization")

        start_time = min(all_timestamps)
        end_time = max(all_timestamps)

        # Calculate target timestamps based on desired frequency
        target_interval = 1.0 / target_fps
        target_timestamps = []
        current_time = start_time
        while current_time <= end_time:
            target_timestamps.append(current_time)
            current_time += target_interval

        total_joint_states = sum(len(state_data) for state_data in joint_states.values())
        print(f"Original data: {total_joint_states} joint states from {len(joint_states)} sources, time range: {start_time:.2f}s to {end_time:.2f}s")
        print(f"Target sampling: {len(target_timestamps)} frames at {target_fps} Hz (interval: {target_interval:.3f}s)")
        print(f"Using latest data approach (no tolerance threshold)")

        # For each target timestamp, find the latest data that is not in the future
        for target_ts in target_timestamps:
            # Find latest joint states from all sources that are not in the future
            latest_joint_states = {}
            for state_name, state_data in joint_states.items():
                latest_state = None
                for ts, js in state_data:
                    if latest_state is None:
                        latest_state = js
                    elif ts <= target_ts:
                        latest_state = js
                    else:
                        break  # Stop when we find a timestamp in the future
                latest_joint_states[state_name] = latest_state

            # Find latest images that are not in the future
            latest_images = {}
            for cam, cam_data in images.items():
                latest_img = None
                for ts, img in cam_data:
                    if latest_img is None:
                        latest_img = img
                    elif ts <= target_ts:
                        latest_img = img
                    else:
                        break  # Stop when we find a timestamp in the future
                latest_images[cam] = latest_img

            # Merge joint states from all sources
            merged_joint_state = self.merge_joint_states(latest_joint_states, joint_names)

            # Only add if we have at least some joint state data
            if merged_joint_state is not None:
                synchronized_data['joint_states'].append(merged_joint_state)
                synchronized_data['timestamps'].append(target_ts)

                for cam, img in latest_images.items():
                    if img is not None:
                        synchronized_data['images'][cam].append(img)
                    else:
                        raise ValueError(f"No image found for camera {cam} at timestamp {target_ts}")

                # Compute actions if requested
                if actions is not None:
                    action_vec = np.zeros(len(joint_names), dtype=np.float32)
                    # Build a name->index map once
                    name_to_idx = {n: i for i, n in enumerate(joint_names)}
                    for name, series in actions.items():
                        latest_map: Optional[Dict[str, float]] = None
                        for ts, a_map in series:
                            if latest_map is None:
                                latest_map = a_map
                            elif ts <= target_ts:
                                latest_map = a_map
                            else:
                                break
                        if latest_map is None:
                            continue
                        for jn, val in latest_map.items():
                            idx = name_to_idx.get(jn)
                            if idx is not None:
                                action_vec[idx] = val
                    synchronized_data['actions'].append(action_vec)
            else:
                raise ValueError(f"No joint state data found for target timestamp {target_ts}")

        return synchronized_data

    def merge_joint_states(self, latest_joint_states: Dict[str, np.ndarray], joint_names: List[str]) -> np.ndarray:
        """
        Merge joint states from different sources into a single combined state.

        Args:
            latest_joint_states: Dictionary mapping state names to joint state arrays
            joint_names: List of joint names to help with mapping

        Returns:
            Combined joint state array, or None if no valid states found
        """

        # Use the joint_names list to determine the correct size
        combined_state = np.zeros(len(joint_names), dtype=np.float32)
        # print(f"Creating combined state with {len(joint_names)} joints: {joint_names}")

        # Count how many states we're actually merging
        merged_count = 0

        # Merge states from all sources
        for state_name, state in latest_joint_states.items():
            if state is not None:
                if state_name == 'joints':
                    # Joint states contain arm, head, gripper joints (excluding mobile base)
                    # Use the same logic as action processing: filter out mobile base joints
                    joint_state_names = [name for name in joint_names if name not in ['linear_x', 'linear_y', 'angular_z']]
                    if len(state) == len(joint_state_names):
                        # Map joint states to the first part of the combined state
                        combined_state[:len(state)] = state
                    else:
                        raise ValueError(f"Joint state from {state_name} size ({len(state)}) doesn't match expected joint names ({len(joint_state_names)})")

                elif state_name == 'odometry':
                    # Odometry contains mobile base velocity (linear_x, linear_y, angular_z)
                    # Use the same logic as action processing: specifically look for mobile base velocity names
                    velocity_names = ['linear_x', 'linear_y', 'angular_z']
                    if len(state) == len(velocity_names):
                        # Check if mobile base joints exist in joint_names
                        mobile_base_indices = []
                        for vel_name in velocity_names:
                            if vel_name in joint_names:
                                mobile_base_indices.append(joint_names.index(vel_name))

                        if len(mobile_base_indices) == len(velocity_names):
                            # Map odometry to the correct positions in combined state
                            for i, vel_idx in enumerate(mobile_base_indices):
                                combined_state[vel_idx] = state[i]
                        else:
                            raise ValueError(f"Not all mobile base joints found in joint_names")
                    else:
                        raise ValueError(f"Odometry from {state_name} has unexpected size ({len(state)})")

                else:
                    raise ValueError(f"Unknown joint state source: {state_name}")

                merged_count += 1

        if merged_count != len(latest_joint_states):
            raise ValueError(f"No joint state data found for all target")

        return combined_state


def read_episode_from_bag(episode_dir: Path,
                         joint_names: List[str],
                         camera_topics: Dict[str, str],
                         joint_state_topics: Dict[str, str],
                         action_topics: Dict[str, str] = None,
                         fps: float = 30.0) -> Dict[str, List[np.ndarray]]:
    """
    Read an episode from a directory containing rosbag files.
    """
    reader = RosbagReader(fps=fps)

    # Find bag files (support .mcap files)
    bag_files = list(episode_dir.glob("*.mcap"))
    if not bag_files:
        print(f"No bag files found in {episode_dir}")
        return {
            'joint_states': [],
            'images': {cam: [] for cam in camera_topics.keys()},
            'actions': [],
            'timestamps': []
        }

    # Read all bag files and combine data
    all_joint_states = {state_name: [] for state_name in joint_state_topics.keys()}
    all_images = {cam: [] for cam in camera_topics.keys()}
    all_actions = ({k: [] for k in action_topics.keys()} if action_topics else None)

    for bag_file in bag_files:
        try:
            messages = reader.read_bag_file(bag_file)

            # Process joint states from different topics
            for state_name, topic_name in joint_state_topics.items():
                if topic_name in messages.get('joint_states', {}):
                    for timestamp, msg in messages['joint_states'][topic_name]:
                        if isinstance(msg, JointState):
                            joint_state_names = [name for name in joint_names if name not in ['linear_x', 'linear_y', 'angular_z']]
                            joint_state = reader.extract_joint_state(msg, joint_state_names)
                            all_joint_states[state_name].append((timestamp, joint_state))
                        elif isinstance(msg, Odometry):
                            velocity_names = ['linear_x', 'linear_y', 'angular_z']
                            velocity_state = reader.extract_odometry_velocity(msg, velocity_names)
                            all_joint_states[state_name].append((timestamp, velocity_state))
                else:
                    error_msg = f"Topic {topic_name} not found in bag file"
                    print(error_msg)
                    raise ValueError(error_msg)

            # Process images
            for cam_name, topic_name in camera_topics.items():
                if topic_name in messages.get('images', {}):
                    for timestamp, msg in messages['images'][topic_name]:
                        if hasattr(msg, 'format'):
                            image = reader.extract_compressed_image(msg)
                        else:
                            image = reader.extract_image(msg)

                        if image is not None:
                            all_images[cam_name].append((timestamp, image))

            # Process actions
            if action_topics:
                for name, topic in action_topics.items():
                    if topic in messages.get('actions', {}):
                        # Ensure the list exists
                        all_actions.setdefault(name, [])
                        for timestamp, msg in messages['actions'][topic]:
                            if isinstance(msg, JointTrajectory):
                                joint_state_names = [n for n in joint_names if n not in ['linear_x', 'linear_y', 'angular_z']]
                                act_map = reader.extract_joint_trajectory(msg, joint_state_names)
                                all_actions[name].append((timestamp, act_map))
                            elif isinstance(msg, Twist):
                                vel_names = ['linear_x', 'linear_y', 'angular_z']
                                act_map = reader.extract_twist(msg, vel_names)
                                all_actions[name].append((timestamp, act_map))
        except Exception as e:
            print(f"Error reading bag file {bag_file}")
            traceback.print_exc()
            continue

    # Synchronize data at target frequency
    synchronized_data = reader.synchronize_data(all_joint_states, all_images, joint_names, all_actions, target_fps=fps)

    return synchronized_data
