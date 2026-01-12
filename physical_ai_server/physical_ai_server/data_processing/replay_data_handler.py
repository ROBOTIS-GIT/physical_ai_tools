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
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

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
        }

        # Validate bag path
        bag_path = Path(bag_path)
        if not bag_path.exists():
            result['message'] = f'Bag path does not exist: {bag_path}'
            return result

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
            action_data: List[Tuple[float, List[str], List[float]]] = []
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
                        # Check if topic name contains 'action'
                        if 'action' in topic.lower():
                            action_data.append((
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
                metadata = sorted(
                    image_metadata_by_topic[first_topic],
                    key=lambda x: x[0]
                )
                for frame_idx, timestamp in metadata:
                    result['frame_indices'].append(frame_idx)
                    result['frame_timestamps'].append(timestamp - min_time)

            # Process joint data
            if joint_data:
                joint_data.sort(key=lambda x: x[0])
                joint_names = joint_data[0][1] if joint_data else []
                result['joint_names'] = joint_names

                for timestamp, names, positions in joint_data:
                    result['joint_timestamps'].append(timestamp - min_time)
                    result['joint_positions'].extend(positions)

            # Process action data
            if action_data:
                action_data.sort(key=lambda x: x[0])
                action_names = action_data[0][1] if action_data else []
                result['action_names'] = action_names

                for timestamp, names, values in action_data:
                    result['action_timestamps'].append(timestamp - min_time)
                    result['action_values'].extend(values)

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
