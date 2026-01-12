# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Dongyun Kim

"""Replay data handler for ROSbag visualization."""

import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

try:
    from rosbag_recorder.msg import ImageMetadata
    HAS_IMAGE_METADATA = True
except ImportError:
    HAS_IMAGE_METADATA = False


class ReplayDataHandler:
    """Handler for extracting replay data from ROSbag files."""

    def __init__(self, logger=None):
        """Initialize ReplayDataHandler."""
        self.logger = logger

    def _log_info(self, msg: str):
        if self.logger:
            self.logger.info(msg)

    def _log_error(self, msg: str):
        if self.logger:
            self.logger.error(msg)

    def _load_metadata(self, bag_path: Path) -> Optional[Dict]:
        """
        Load robot_config.yaml from rosbag directory if available.

        Returns metadata dict or None if not found.
        """
        metadata_path = bag_path / 'robot_config.yaml'
        if metadata_path.exists():
            try:
                with open(metadata_path, 'r', encoding='utf-8') as f:
                    metadata = yaml.safe_load(f)
                self._log_info(f'Loaded robot config from: {metadata_path}')
                return metadata
            except Exception as e:
                self._log_error(f'Failed to load robot config: {e}')
        return None

    def _get_recording_date(self, bag_path: Path) -> Optional[str]:
        """
        Get recording date from metadata.yaml.

        Returns ISO format datetime string or None.
        """
        metadata_path = bag_path / 'metadata.yaml'
        if not metadata_path.exists():
            return None

        try:
            with open(metadata_path, 'r', encoding='utf-8') as f:
                metadata = yaml.safe_load(f)

            if metadata and 'rosbag2_bagfile_information' in metadata:
                bag_info = metadata['rosbag2_bagfile_information']
                starting_time = bag_info.get('starting_time', {})
                ns_since_epoch = starting_time.get('nanoseconds_since_epoch', 0)
                if ns_since_epoch > 0:
                    dt = datetime.fromtimestamp(ns_since_epoch / 1e9)
                    return dt.isoformat()
        except Exception as e:
            self._log_error(f'Failed to get recording date: {e}')

        return None

    def _get_directory_size(self, bag_path: Path) -> int:
        """
        Get total size of bag directory in bytes.
        """
        total_size = 0
        try:
            for item in bag_path.rglob('*'):
                if item.is_file():
                    total_size += item.stat().st_size
        except Exception as e:
            self._log_error(f'Failed to get directory size: {e}')
        return total_size

    def _get_task_markers(self, bag_path: Path) -> List[Dict]:
        """
        Get task markers from robot_config.yaml.

        Returns list of task markers or empty list.
        """
        config_path = bag_path / 'robot_config.yaml'
        if not config_path.exists():
            return []

        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            return config.get('task_markers', [])
        except Exception as e:
            self._log_error(f'Failed to get task markers: {e}')
        return []

    def _get_trim_points(self, bag_path: Path) -> Optional[Dict]:
        """
        Get trim points from robot_config.yaml.

        Returns trim points dict or None.
        """
        config_path = bag_path / 'robot_config.yaml'
        if not config_path.exists():
            return None

        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            return config.get('trim_points', None)
        except Exception as e:
            self._log_error(f'Failed to get trim points: {e}')
        return None

    def _get_exclude_regions(self, bag_path: Path) -> List[Dict]:
        """
        Get exclude regions from robot_config.yaml.

        Returns list of exclude regions or empty list.
        """
        config_path = bag_path / 'robot_config.yaml'
        if not config_path.exists():
            return []

        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            return config.get('exclude_regions', [])
        except Exception as e:
            self._log_error(f'Failed to get exclude regions: {e}')
        return []

    def update_task_markers(
        self,
        bag_path: str,
        task_markers: List[Dict],
        trim_points: Optional[Dict] = None,
        exclude_regions: Optional[List[Dict]] = None
    ) -> Dict:
        """
        Update task markers, trim points, and exclude regions in robot_config.yaml.

        Args:
            bag_path: Path to the ROSbag directory
            task_markers: List of task marker dictionaries
            trim_points: Optional dict with 'start' and 'end' trim point info
            exclude_regions: Optional list of exclude region dicts

        Returns:
            Result dictionary with success status
        """
        result = {'success': False, 'message': ''}
        config_path = Path(bag_path) / 'robot_config.yaml'

        if not config_path.exists():
            # Create new config file if it doesn't exist
            config = {}
        else:
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f) or {}
            except Exception as e:
                result['message'] = f'Failed to read config: {e}'
                return result

        # Sort markers by frame (frame-based ordering) and ensure frame is primary field
        sorted_markers = sorted(task_markers, key=lambda m: m.get('frame', 0))
        config['task_markers'] = sorted_markers

        # Update trim points if provided
        if trim_points:
            config['trim_points'] = trim_points
        elif 'trim_points' in config and trim_points is None:
            # Remove trim_points if explicitly set to None
            del config['trim_points']

        # Update exclude regions if provided
        if exclude_regions:
            # Sort by start time
            sorted_regions = sorted(exclude_regions, key=lambda r: r.get('start', {}).get('time', 0))
            config['exclude_regions'] = sorted_regions
        elif 'exclude_regions' in config and exclude_regions is None:
            # Remove exclude_regions if explicitly set to None
            del config['exclude_regions']

        try:
            with open(config_path, 'w', encoding='utf-8') as f:
                yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
            result['success'] = True
            result['message'] = f'Saved {len(task_markers)} task markers'
            if trim_points:
                result['message'] += ', trim points'
            if exclude_regions:
                result['message'] += f', {len(exclude_regions)} exclude regions'
            self._log_info(result['message'])
        except Exception as e:
            result['message'] = f'Failed to write config: {e}'
            self._log_error(result['message'])

        return result

    def _extract_camera_name_from_topic(self, topic: str) -> str:
        """
        Extract a meaningful camera name from a topic path.

        Examples:
            /zed/zed_node/left/image_rect_color/compressed -> cam_head
            /camera_left/camera_left/color/image_rect_raw/compressed -> cam_wrist_left
            /camera_right/camera_right/color/image_rect_raw/compressed -> cam_wrist_right

        Args:
            topic: Full ROS topic path

        Returns:
            Human-readable camera name
        """
        topic_lower = topic.lower()

        # Check for known camera patterns
        if 'zed' in topic_lower:
            return 'cam_head'
        elif 'camera_left' in topic_lower or 'cam_left' in topic_lower:
            return 'cam_wrist_left'
        elif 'camera_right' in topic_lower or 'cam_right' in topic_lower:
            return 'cam_wrist_right'
        elif 'head' in topic_lower:
            return 'cam_head'
        elif 'wrist' in topic_lower:
            if 'left' in topic_lower:
                return 'cam_wrist_left'
            elif 'right' in topic_lower:
                return 'cam_wrist_right'
            return 'cam_wrist'

        # Fallback: extract first meaningful segment
        parts = [p for p in topic.split('/') if p and p not in [
            'compressed', 'image_rect_raw', 'image_rect_color', 'color', 'rgb'
        ]]
        if parts:
            return parts[0]

        return topic

    def _get_action_topic_order(self, metadata: Optional[Dict]) -> List[str]:
        """
        Get ordered list of action topics from metadata or use default.

        Returns list of action topic names for consistent joint ordering.
        """
        if metadata and 'action_topics' in metadata:
            # Extract topic paths from action_topics dict
            action_topics = metadata['action_topics']
            # Sort by key to ensure consistent ordering
            return [action_topics[k] for k in sorted(action_topics.keys())]

        # Default order if no metadata available
        return [
            '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
            '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
            '/leader/joystick_controller_left/joint_trajectory',
            '/leader/joystick_controller_right/joint_trajectory',
        ]

    def _is_action_topic(self, topic: str, metadata: Optional[Dict]) -> bool:
        """
        Determine if a topic is an action topic based on metadata or naming.
        """
        if metadata and 'action_topics' in metadata:
            action_topic_paths = list(metadata['action_topics'].values())
            return topic in action_topic_paths

        # Fallback to name-based detection
        return 'action' in topic.lower() or topic.startswith('/leader')

    def get_replay_data(self, bag_path: str) -> Dict:
        """
        Extract replay data from a ROSbag directory.

        Args:
            bag_path: Path to the ROSbag directory

        Returns:
            Dictionary containing video info, frame timestamps, and joint data
        """
        result = {
            'success': False,
            'message': '',
            'video_files': [],
            'video_topics': [],
            'video_names': [],  # Human-readable camera names from config
            'video_fps': [],
            'frame_indices': [],
            'frame_timestamps': [],
            'joint_timestamps': [],
            'joint_names': [],
            'joint_positions': [],
            'action_timestamps': [],
            'action_names': [],
            'action_values': [],
            'start_time': 0.0,
            'end_time': 0.0,
            'duration': 0.0,
            'robot_type': '',
            'metadata': None,
            # Extended metadata fields
            'recording_date': None,
            'file_size_bytes': 0,
            'task_markers': [],
            'trim_points': None,
            'exclude_regions': [],
            'frame_counts': {},
        }

        # Validate bag path
        bag_path = Path(bag_path)
        if not bag_path.exists():
            result['message'] = f'Bag path does not exist: {bag_path}'
            return result

        # Load metadata if available
        metadata = self._load_metadata(bag_path)
        if metadata:
            result['metadata'] = metadata
            result['robot_type'] = metadata.get('robot_type', '')

        # Get extended metadata
        result['recording_date'] = self._get_recording_date(bag_path)
        result['file_size_bytes'] = self._get_directory_size(bag_path)
        result['task_markers'] = self._get_task_markers(bag_path)
        result['trim_points'] = self._get_trim_points(bag_path)
        result['exclude_regions'] = self._get_exclude_regions(bag_path)

        # Find MCAP file
        mcap_files = list(bag_path.glob('*.mcap'))
        if not mcap_files:
            # Try db3 format
            mcap_files = list(bag_path.glob('*.db3'))

        if not mcap_files:
            result['message'] = f'No bag file found in: {bag_path}'
            return result

        try:
            # Read bag file
            reader = SequentialReader()
            storage_options = StorageOptions(
                uri=str(bag_path),
                storage_id='mcap' if mcap_files[0].suffix == '.mcap' else 'sqlite3'
            )
            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
            reader.open(storage_options, converter_options)

            # Get topic information
            topic_types = reader.get_all_topics_and_types()
            topic_type_map = {t.name: t.type for t in topic_types}

            # Collect data
            image_metadata_by_topic: Dict[str, List[Tuple[int, float]]] = {}
            joint_data: List[Tuple[float, List[str], List[float]]] = []
            # Store action data per topic for proper merging
            action_data_by_topic: Dict[str, List[Tuple[float, List[str], List[float]]]] = {}
            min_time = float('inf')
            max_time = float('-inf')

            while reader.has_next():
                topic, data, timestamp = reader.read_next()
                timestamp_sec = timestamp / 1e9

                min_time = min(min_time, timestamp_sec)
                max_time = max(max_time, timestamp_sec)

                topic_type = topic_type_map.get(topic, '')

                # Handle ImageMetadata messages
                if HAS_IMAGE_METADATA and 'ImageMetadata' in topic_type:
                    try:
                        msg = deserialize_message(data, ImageMetadata)
                        # Extract source topic from metadata topic name
                        source_topic = topic.replace('/metadata', '')

                        if source_topic not in image_metadata_by_topic:
                            image_metadata_by_topic[source_topic] = []

                        image_metadata_by_topic[source_topic].append(
                            (msg.frame_index, timestamp_sec)
                        )
                    except Exception as e:
                        self._log_error(f'Failed to deserialize ImageMetadata: {e}')

                # Handle JointState messages - check if it's action or state
                elif topic_type == 'sensor_msgs/msg/JointState':
                    try:
                        msg = deserialize_message(data, JointState)
                        # Check if topic name contains 'action' or 'leader'
                        if 'action' in topic.lower() or topic.startswith('/leader'):
                            if topic not in action_data_by_topic:
                                action_data_by_topic[topic] = []
                            action_data_by_topic[topic].append((
                                timestamp_sec,
                                list(msg.name),
                                list(msg.position)
                            ))
                        else:
                            joint_data.append((
                                timestamp_sec,
                                list(msg.name),
                                list(msg.position)
                            ))
                    except Exception as e:
                        self._log_error(f'Failed to deserialize JointState: {e}')

                # Handle JointTrajectory messages (leader topics are action)
                elif topic_type == 'trajectory_msgs/msg/JointTrajectory':
                    try:
                        msg = deserialize_message(data, JointTrajectory)
                        # Leader topics are action data - store per topic
                        if msg.points and len(msg.points) > 0:
                            if topic not in action_data_by_topic:
                                action_data_by_topic[topic] = []
                            action_data_by_topic[topic].append((
                                timestamp_sec,
                                list(msg.joint_names),
                                list(msg.points[0].positions)
                            ))
                    except Exception as e:
                        self._log_error(f'Failed to deserialize JointTrajectory: {e}')

            # Build camera name mapping from metadata
            camera_name_map = {}  # topic -> camera_name
            if metadata and 'camera_topics' in metadata:
                for cam_name, topic_path in metadata['camera_topics'].items():
                    camera_name_map[topic_path] = cam_name

            # Process video files
            videos_dir = bag_path / 'videos'
            if videos_dir.exists():
                for video_file in sorted(videos_dir.glob('*.mp4')):
                    result['video_files'].append(f'videos/{video_file.name}')

                    # Find matching topic
                    video_name = video_file.stem
                    matching_topic = None
                    for topic in image_metadata_by_topic.keys():
                        sanitized = topic.replace('/', '_').lstrip('_')
                        if sanitized == video_name:
                            matching_topic = topic
                            break

                    result['video_topics'].append(matching_topic or video_name)

                    # Find camera name from metadata or extract from topic/filename
                    camera_name = None
                    if matching_topic:
                        camera_name = camera_name_map.get(matching_topic)
                        if not camera_name:
                            # Fallback: extract meaningful name from topic path
                            camera_name = self._extract_camera_name_from_topic(matching_topic)
                    else:
                        # No matching topic, try to extract from video filename
                        camera_name = self._extract_camera_name_from_topic(video_name)
                    result['video_names'].append(camera_name or video_name)

                    # Calculate FPS from metadata
                    if matching_topic and matching_topic in image_metadata_by_topic:
                        metadata = image_metadata_by_topic[matching_topic]
                        if len(metadata) > 1:
                            # Sort by frame index
                            metadata.sort(key=lambda x: x[0])
                            time_diffs = [
                                metadata[i + 1][1] - metadata[i][1]
                                for i in range(len(metadata) - 1)
                            ]
                            avg_diff = sum(time_diffs) / len(time_diffs)
                            fps = 1.0 / avg_diff if avg_diff > 0 else 30.0
                            result['video_fps'].append(fps)
                        else:
                            result['video_fps'].append(30.0)
                    else:
                        result['video_fps'].append(30.0)

            # Process frame timestamps (use first video topic)
            if image_metadata_by_topic:
                first_topic = list(image_metadata_by_topic.keys())[0]
                metadata_list = sorted(
                    image_metadata_by_topic[first_topic],
                    key=lambda x: x[0]
                )
                for frame_idx, timestamp in metadata_list:
                    result['frame_indices'].append(frame_idx)
                    result['frame_timestamps'].append(timestamp - min_time)

            # Build frame counts per camera
            for i, video_name in enumerate(result['video_names']):
                topic = result['video_topics'][i] if i < len(result['video_topics']) else None
                if topic and topic in image_metadata_by_topic:
                    result['frame_counts'][video_name] = len(image_metadata_by_topic[topic])
                else:
                    result['frame_counts'][video_name] = 0

            # Process joint data
            if joint_data:
                joint_data.sort(key=lambda x: x[0])
                joint_names = joint_data[0][1] if joint_data else []
                result['joint_names'] = joint_names

                for timestamp, names, positions in joint_data:
                    result['joint_timestamps'].append(timestamp - min_time)
                    result['joint_positions'].extend(positions)

            # Process action data from all topics
            if action_data_by_topic:
                # Get topic order from metadata or use default
                topic_order = self._get_action_topic_order(metadata)

                # Also include any topics not in the predefined order
                for topic in action_data_by_topic.keys():
                    if topic not in topic_order:
                        topic_order.append(topic)

                # Collect all action names in order
                all_action_names = []
                topic_joint_names = {}
                for topic in topic_order:
                    if topic in action_data_by_topic and action_data_by_topic[topic]:
                        names = action_data_by_topic[topic][0][1]
                        topic_joint_names[topic] = names
                        all_action_names.extend(names)

                result['action_names'] = all_action_names

                # Merge action data by timestamp
                # Group data by approximate timestamp (within 10ms)
                timestamp_data = {}
                for topic in topic_order:
                    if topic not in action_data_by_topic:
                        continue
                    for ts, names, values in action_data_by_topic[topic]:
                        # Round to 10ms for grouping
                        ts_key = round(ts * 100) / 100
                        if ts_key not in timestamp_data:
                            timestamp_data[ts_key] = {}
                        timestamp_data[ts_key][topic] = (names, values)

                # Build action values in correct order (with forward fill for missing data)
                last_values_by_topic = {}  # Store last known values for each topic
                for ts_key in sorted(timestamp_data.keys()):
                    result['action_timestamps'].append(ts_key - min_time)
                    for topic in topic_order:
                        if topic in timestamp_data[ts_key]:
                            _, values = timestamp_data[ts_key][topic]
                            result['action_values'].extend(values)
                            last_values_by_topic[topic] = values  # Remember last values
                        elif topic in topic_joint_names:
                            # Forward fill: use last known values instead of zeros
                            if topic in last_values_by_topic:
                                result['action_values'].extend(last_values_by_topic[topic])
                            else:
                                # No previous data yet, use zeros as fallback
                                result['action_values'].extend([0.0] * len(topic_joint_names[topic]))

            # Set duration info
            if min_time != float('inf') and max_time != float('-inf'):
                result['start_time'] = 0.0
                result['end_time'] = max_time - min_time
                result['duration'] = max_time - min_time

            result['success'] = True
            result['message'] = 'Replay data loaded successfully'
            self._log_info(
                f'Loaded replay data: {len(result["video_files"])} videos, '
                f'{len(result["frame_timestamps"])} frames, '
                f'{len(result["joint_timestamps"])} joint samples, '
                f'{len(result["action_timestamps"])} action samples'
            )

        except Exception as e:
            result['message'] = f'Failed to read bag file: {str(e)}'
            self._log_error(result['message'])

        return result

    def get_video_file_path(self, bag_path: str, video_file: str) -> Optional[str]:
        """
        Get the full path to a video file.

        Args:
            bag_path: Path to the ROSbag directory
            video_file: Relative path to the video file

        Returns:
            Full path to the video file or None if not found
        """
        full_path = Path(bag_path) / video_file
        if full_path.exists():
            return str(full_path)
        return None

    def get_rosbag_list(self, folder_path: str) -> Dict:
        """
        Get list of ROSbag directories in a folder.

        A directory is considered a ROSbag if it contains:
        - metadata.yaml file (rosbag2 metadata)
        - .mcap or .db3 file

        Args:
            folder_path: Path to the parent folder

        Returns:
            Dictionary with rosbag list info
        """
        result = {
            'success': False,
            'message': '',
            'rosbags': [],
            'parent_path': '',
        }

        folder = Path(folder_path)
        if not folder.exists():
            result['message'] = f'Folder not found: {folder_path}'
            return result

        if not folder.is_dir():
            result['message'] = f'Path is not a directory: {folder_path}'
            return result

        result['parent_path'] = str(folder)
        rosbags = []

        # Check all subdirectories
        for item in sorted(folder.iterdir()):
            if not item.is_dir():
                continue

            # Check if it's a valid ROSbag directory
            has_metadata = (item / 'metadata.yaml').exists()
            has_mcap = any(item.glob('*.mcap'))
            has_db3 = any(item.glob('*.db3'))
            has_videos = (item / 'videos').exists()

            if has_metadata and (has_mcap or has_db3):
                bag_info = {
                    'name': item.name,
                    'path': str(item),
                    'has_videos': has_videos,
                }

                # Try to get duration from metadata
                try:
                    metadata_path = item / 'metadata.yaml'
                    with open(metadata_path, 'r', encoding='utf-8') as f:
                        metadata = yaml.safe_load(f)
                        if metadata and 'rosbag2_bagfile_information' in metadata:
                            bag_info['duration_ns'] = metadata['rosbag2_bagfile_information'].get(
                                'duration', {}).get('nanoseconds', 0)
                except Exception:
                    pass

                rosbags.append(bag_info)

        result['rosbags'] = rosbags
        result['success'] = True
        result['message'] = f'Found {len(rosbags)} ROSbag(s)'
        self._log_info(result['message'])

        return result
