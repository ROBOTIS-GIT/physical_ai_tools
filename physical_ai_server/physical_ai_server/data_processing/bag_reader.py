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

"""ROSbag file reader for extracting messages and metadata."""

from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Tuple

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

try:
    from rosbag_recorder.msg import ImageMetadata

    HAS_IMAGE_METADATA = True
except ImportError:
    HAS_IMAGE_METADATA = False


class BagReader:
    """
    ROSbag file reader for extracting messages.

    Handles both MCAP and SQLite3 (db3) formats.
    """

    def __init__(self, bag_path: Path, logger=None):
        """
        Initialize BagReader.

        Args:
            bag_path: Path to the ROSbag directory
            logger: Optional logger instance
        """
        self.bag_path = Path(bag_path)
        self.logger = logger
        self._reader: Optional[SequentialReader] = None
        self._topic_type_map: Dict[str, str] = {}
        self._storage_id: str = "mcap"

    def _log_info(self, msg: str):
        if self.logger:
            self.logger.info(msg)

    def _log_error(self, msg: str):
        if self.logger:
            self.logger.error(msg)

    def _detect_storage_format(self) -> Optional[str]:
        """
        Detect the storage format of the bag file.

        Returns:
            'mcap' or 'sqlite3' if found, None otherwise
        """
        mcap_files = list(self.bag_path.glob("*.mcap"))
        if mcap_files:
            return "mcap"

        db3_files = list(self.bag_path.glob("*.db3"))
        if db3_files:
            return "sqlite3"

        return None

    def open(self) -> bool:
        """
        Open the bag file for reading.

        Returns:
            True if successful, False otherwise
        """
        storage_id = self._detect_storage_format()
        if storage_id is None:
            self._log_error(f"No bag file found in: {self.bag_path}")
            return False

        self._storage_id = storage_id

        try:
            self._reader = SequentialReader()
            storage_options = StorageOptions(
                uri=str(self.bag_path), storage_id=storage_id
            )
            converter_options = ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            )
            self._reader.open(storage_options, converter_options)

            # Build topic type map
            topic_types = self._reader.get_all_topics_and_types()
            self._topic_type_map = {t.name: t.type for t in topic_types}

            return True
        except Exception as e:
            self._log_error(f"Failed to open bag file: {e}")
            return False

    def get_topic_types(self) -> Dict[str, str]:
        """
        Get mapping of topic names to their message types.

        Returns:
            Dictionary mapping topic names to type strings
        """
        return self._topic_type_map.copy()

    def read_messages(
        self, topic_filter: Optional[List[str]] = None
    ) -> Iterator[Tuple[str, Any, float]]:
        """
        Read messages from the bag file.

        Args:
            topic_filter: Optional list of topic names to filter

        Yields:
            Tuple of (topic_name, deserialized_message, timestamp_sec)
        """
        if self._reader is None:
            self._log_error("Bag file not opened. Call open() first.")
            return

        while self._reader.has_next():
            topic, data, timestamp = self._reader.read_next()

            # Apply topic filter if specified
            if topic_filter and topic not in topic_filter:
                continue

            timestamp_sec = timestamp / 1e9
            topic_type = self._topic_type_map.get(topic, "")

            # Deserialize based on message type
            msg = self._deserialize_message(data, topic_type, topic)
            if msg is not None:
                yield topic, msg, timestamp_sec

    def read_raw_messages(self) -> Iterator[Tuple[str, bytes, float, str]]:
        """
        Read raw (serialized) messages from the bag file.

        Yields:
            Tuple of (topic_name, raw_data, timestamp_sec, topic_type)
        """
        if self._reader is None:
            self._log_error("Bag file not opened. Call open() first.")
            return

        while self._reader.has_next():
            topic, data, timestamp = self._reader.read_next()
            timestamp_sec = timestamp / 1e9
            topic_type = self._topic_type_map.get(topic, "")
            yield topic, data, timestamp_sec, topic_type

    def _deserialize_message(
        self, data: bytes, topic_type: str, topic: str
    ) -> Optional[Any]:
        """
        Deserialize a message based on its type.

        Args:
            data: Raw serialized message data
            topic_type: Message type string
            topic: Topic name (for logging)

        Returns:
            Deserialized message or None if unsupported/failed
        """
        try:
            if HAS_IMAGE_METADATA and "ImageMetadata" in topic_type:
                return deserialize_message(data, ImageMetadata)
            elif topic_type == "sensor_msgs/msg/JointState":
                return deserialize_message(data, JointState)
            elif topic_type == "trajectory_msgs/msg/JointTrajectory":
                return deserialize_message(data, JointTrajectory)
            else:
                # Return None for unsupported types
                return None
        except Exception as e:
            self._log_error(f"Failed to deserialize {topic_type} from {topic}: {e}")
            return None

    def get_time_range(self) -> Tuple[float, float]:
        """
        Get the time range of the bag file.

        Note: This requires reading through the entire bag file.
        Consider using metadata.yaml for faster access.

        Returns:
            Tuple of (start_time, end_time) in seconds
        """
        min_time = float("inf")
        max_time = float("-inf")

        if self._reader is None:
            return (0.0, 0.0)

        # Read through all messages to find time range
        for _, _, timestamp_sec, _ in self.read_raw_messages():
            min_time = min(min_time, timestamp_sec)
            max_time = max(max_time, timestamp_sec)

        if min_time == float("inf"):
            return (0.0, 0.0)

        return (min_time, max_time)

    def close(self):
        """Close the bag reader."""
        self._reader = None
        self._topic_type_map = {}

    def __enter__(self):
        """Context manager entry."""
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
        return False
