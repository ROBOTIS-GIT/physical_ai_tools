#!/usr/bin/env python3
#
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

"""
ROSbag + MP4 to LeRobot v2.1 Dataset Converter.

Converts recorded robot data (ROSbag with joint states + MP4 videos) to
LeRobot v2.1 dataset format for training with LeRobot framework.

LeRobot v2.1 Dataset Structure:
    dataset_name/
    ├── data/
    │   └── chunk-{chunk:03d}/
    │       └── episode_{episode:06d}.parquet
    ├── meta/
    │   ├── info.json
    │   ├── episodes.jsonl
    │   ├── episodes_stats.jsonl
    │   └── tasks.jsonl
    └── videos/
        └── chunk-{chunk:03d}/
            └── observation.images.{camera}/
                └── episode_{episode:06d}.mp4
"""

import json
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from .bag_reader import BagReader
from .metadata_manager import MetadataManager
from .quality_analyzer import QualityAnalyzer, QualityConfig, QualityReport
from .video_metadata_extractor import VideoMetadataExtractor


CODEBASE_VERSION = "v2.1"
DEFAULT_CHUNK_SIZE = 1000
DEFAULT_FPS = 30


@dataclass
class StalenessMetrics:
    """Metrics for tracking data staleness during causal sync resampling."""

    topic: str
    total_samples: int = 0
    stale_warning_count: int = 0
    stale_error_count: int = 0
    max_staleness_ms: float = 0.0
    mean_staleness_ms: float = 0.0
    stale_samples: List[Dict[str, Any]] = field(default_factory=list)

    @property
    def warning_ratio(self) -> float:
        if self.total_samples == 0:
            return 0.0
        return self.stale_warning_count / self.total_samples

    @property
    def error_ratio(self) -> float:
        if self.total_samples == 0:
            return 0.0
        return self.stale_error_count / self.total_samples

    @property
    def status(self) -> str:
        if self.stale_error_count > 0:
            return "ERROR"
        if self.stale_warning_count > 0:
            return "WARNING"
        return "GOOD"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "topic": self.topic,
            "total_samples": self.total_samples,
            "staleness": {
                "warning_count": self.stale_warning_count,
                "error_count": self.stale_error_count,
                "warning_ratio": round(self.warning_ratio * 100, 2),
                "error_ratio": round(self.error_ratio * 100, 2),
                "max_ms": round(self.max_staleness_ms, 2),
                "mean_ms": round(self.mean_staleness_ms, 2),
            },
            "status": self.status,
            "stale_samples": self.stale_samples[:20],  # Limit to first 20
        }


@dataclass
class ConversionConfig:
    """Configuration for ROSbag to LeRobot conversion."""

    repo_id: str
    output_dir: Path
    fps: int = DEFAULT_FPS
    robot_type: str = "unknown"
    use_videos: bool = True
    chunks_size: int = DEFAULT_CHUNK_SIZE

    # Topic mappings (can be overridden from robot_config.yaml)
    state_topics: List[str] = field(default_factory=list)
    action_topics: List[str] = field(default_factory=list)

    # Trim settings
    apply_trim: bool = True
    apply_exclude_regions: bool = True

    # Quality analysis settings
    enable_quality_report: bool = True
    quality_warning_multiplier: float = 2.0
    quality_error_multiplier: float = 4.0


@dataclass
class EpisodeData:
    """Data container for a single episode."""

    episode_index: int
    timestamps: List[float] = field(default_factory=list)
    observation_state: List[np.ndarray] = field(default_factory=list)
    action: List[np.ndarray] = field(default_factory=list)
    video_files: Dict[str, Path] = field(default_factory=dict)
    tasks: List[str] = field(default_factory=list)
    length: int = 0


class RosbagToLerobotConverter:
    """
    Converts ROSbag recordings with MP4 videos to LeRobot v2.1 dataset format.

    This converter handles:
    - Reading joint states from ROSbag (observation.state)
    - Reading action commands from ROSbag (action)
    - Copying/linking MP4 video files to proper LeRobot structure
    - Generating metadata files (info.json, episodes.jsonl, tasks.jsonl)
    - Computing and storing episode statistics
    - Supporting trim points and exclude regions from robot_config.yaml
    """

    def __init__(self, config: ConversionConfig, logger=None):
        self.config = config
        self.logger = logger
        self._metadata_manager = MetadataManager(logger)
        self._video_extractor = VideoMetadataExtractor(logger)

        quality_config = QualityConfig(
            warning_multiplier=config.quality_warning_multiplier,
            error_multiplier=config.quality_error_multiplier,
        )
        self._quality_analyzer = QualityAnalyzer(quality_config, logger)

        self._features: Dict[str, Dict] = {}
        self._tasks: Dict[int, str] = {}
        self._task_to_index: Dict[str, int] = {}
        self._episodes: Dict[int, Dict] = {}
        self._episodes_stats: Dict[int, Dict] = {}
        self._total_frames = 0
        self._total_episodes = 0
        self._quality_reports: Dict[int, QualityReport] = {}
        self._staleness_reports: Dict[int, Dict[str, StalenessMetrics]] = {}

        self._state_joint_names: List[str] = []
        self._action_joint_names: List[str] = []
        self._camera_mapping: Dict[str, str] = {}  # topic -> camera_name
        self._joint_order: List[str] = []  # Ordered list of joints to include

    def _log_info(self, msg: str):
        if self.logger:
            self.logger.info(msg)
        else:
            print(f"[INFO] {msg}")

    def _log_error(self, msg: str):
        if self.logger:
            self.logger.error(msg)
        else:
            print(f"[ERROR] {msg}")

    def _log_warning(self, msg: str):
        if self.logger:
            self.logger.warning(msg)
        else:
            print(f"[WARNING] {msg}")

    def _log_staleness_summary(self, staleness_metrics: Dict[str, StalenessMetrics]):
        for topic, metrics in staleness_metrics.items():
            if metrics.status == "GOOD":
                continue
            self._log_warning(
                f"Staleness {metrics.status} for {topic}: "
                f"warnings={metrics.stale_warning_count}, errors={metrics.stale_error_count}, "
                f"max={metrics.max_staleness_ms:.1f}ms, mean={metrics.mean_staleness_ms:.1f}ms"
            )

    def _analyze_rosbag_quality(self, bag_path: Path) -> QualityReport:
        reader = BagReader(bag_path, self.logger)
        if not reader.open():
            self._log_error(f"Failed to open rosbag for quality analysis: {bag_path}")
            return QualityReport(
                source_bag=str(bag_path),
                duration_sec=0.0,
                total_messages=0,
            )

        topic_timestamps: Dict[str, List[float]] = {}
        topic_types = reader.get_topic_types()

        for topic, msg, timestamp in reader.read_messages():
            if topic not in topic_timestamps:
                topic_timestamps[topic] = []
            topic_timestamps[topic].append(timestamp)

        return self._quality_analyzer.analyze_rosbag(
            bag_path=bag_path,
            topic_timestamps=topic_timestamps,
            topic_types=topic_types,
        )

    def convert_single_rosbag(
        self,
        bag_path: Path,
        episode_index: int,
    ) -> Optional[EpisodeData]:
        bag_path = Path(bag_path)
        if not bag_path.exists():
            self._log_error(f"Bag path does not exist: {bag_path}")
            return None

        self._log_info(f"Converting rosbag: {bag_path} (episode {episode_index})")

        robot_config = self._metadata_manager.load_robot_config(bag_path)
        if robot_config:
            self._update_config_from_robot_config(robot_config)

        if self.config.enable_quality_report:
            quality_report = self._analyze_rosbag_quality(bag_path)
            self._quality_reports[episode_index] = quality_report
            self._quality_analyzer.print_summary(quality_report)

        trim_points = None
        exclude_regions = []
        if self.config.apply_trim:
            trim_points = self._metadata_manager.get_trim_points(bag_path)
        if self.config.apply_exclude_regions:
            exclude_regions = self._metadata_manager.get_exclude_regions(bag_path)

        episode_data = self._extract_joint_data(
            bag_path, episode_index, trim_points, exclude_regions
        )
        if episode_data is None:
            return None

        video_files = self._find_video_files(bag_path)
        episode_data.video_files = video_files

        task_markers = self._metadata_manager.get_task_markers(bag_path)
        if task_markers:
            episode_data.tasks = list(
                set(m.get("instruction", "default_task") for m in task_markers)
            )
        else:
            episode_data.tasks = ["default_task"]

        return episode_data

    def _update_config_from_robot_config(self, robot_config: Dict):
        """Update conversion config from robot_config.yaml."""
        if "robot_type" in robot_config:
            self.config.robot_type = robot_config["robot_type"]

        if "state_topics" in robot_config:
            topics = robot_config["state_topics"]
            if isinstance(topics, dict):
                self.config.state_topics = list(topics.values())
            elif isinstance(topics, list):
                self.config.state_topics = topics

        if "action_topics" in robot_config:
            topics = robot_config["action_topics"]
            if isinstance(topics, dict):
                self.config.action_topics = list(topics.values())
            elif isinstance(topics, list):
                self.config.action_topics = topics

        if "fps" in robot_config:
            self.config.fps = robot_config["fps"]

        if "camera_mapping" in robot_config:
            self._camera_mapping = robot_config["camera_mapping"]
            self._log_info(f"Loaded camera mapping: {self._camera_mapping}")

        # Prefer total_joint_order (flat list) over joint_order (nested dict)
        if "total_joint_order" in robot_config:
            self._joint_order = robot_config["total_joint_order"]
            self._log_info(
                f"Loaded total_joint_order with {len(self._joint_order)} joints"
            )
        elif "joint_order" in robot_config:
            joint_order = robot_config["joint_order"]
            # Handle nested dict structure (flatten values)
            if isinstance(joint_order, dict):
                flattened = []
                for key, joints in joint_order.items():
                    if isinstance(joints, list):
                        flattened.extend(joints)
                    else:
                        flattened.append(joints)
                self._joint_order = flattened
                self._log_info(
                    f"Loaded joint_order (flattened from dict) with {len(self._joint_order)} joints"
                )
            else:
                self._joint_order = joint_order
                self._log_info(
                    f"Loaded joint_order with {len(self._joint_order)} joints"
                )

    def _extract_joint_data(
        self,
        bag_path: Path,
        episode_index: int,
        trim_points: Optional[Dict],
        exclude_regions: List[Dict],
    ) -> Optional[EpisodeData]:
        """Extract joint state and action data from ROSbag."""
        reader = BagReader(bag_path, self.logger)
        if not reader.open():
            self._log_error(f"Failed to open rosbag: {bag_path}")
            return None

        episode = EpisodeData(episode_index=episode_index)

        # Determine time bounds from trim points
        trim_start = (
            trim_points.get("start", {}).get("time", 0.0) if trim_points else 0.0
        )
        trim_end = (
            trim_points.get("end", {}).get("time", float("inf"))
            if trim_points
            else float("inf")
        )

        # Collect state and action messages
        state_messages: List[Tuple[float, np.ndarray]] = []
        # Group action messages by topic to handle multiple action sources
        action_messages_by_topic: Dict[str, List[Tuple[float, np.ndarray]]] = {}
        action_joint_names_by_topic: Dict[str, List[str]] = {}

        topic_types = reader.get_topic_types()

        for topic, msg, timestamp in reader.read_messages():
            # Skip if outside trim bounds
            if timestamp < trim_start or timestamp > trim_end:
                continue

            # Skip if in exclude region
            if self._is_in_exclude_region(timestamp, exclude_regions):
                continue

            # Process state topics (JointState)
            if self._is_state_topic(topic, topic_types):
                if hasattr(msg, "position") and msg.position:
                    msg_names = (
                        list(msg.name) if hasattr(msg, "name") and msg.name else []
                    )
                    positions = np.array(msg.position, dtype=np.float32)

                    # Filter by joint_order if specified
                    if self._joint_order and msg_names:
                        filtered_positions = self._filter_positions_by_joint_order(
                            positions, msg_names, self._joint_order
                        )
                        if filtered_positions is not None:
                            state_messages.append((timestamp, filtered_positions))
                            # Set state joint names from joint_order
                            if not self._state_joint_names:
                                self._state_joint_names = list(self._joint_order)
                    else:
                        state_messages.append((timestamp, positions))
                        # Capture joint names on first message
                        if not self._state_joint_names and msg_names:
                            self._state_joint_names = msg_names

            # Process action topics (JointTrajectory or JointState)
            elif self._is_action_topic(topic, topic_types):
                positions = self._extract_action_positions(msg)
                if positions is not None:
                    if topic not in action_messages_by_topic:
                        action_messages_by_topic[topic] = []
                    action_messages_by_topic[topic].append((timestamp, positions))

                    # Capture action joint names per topic
                    if topic not in action_joint_names_by_topic:
                        names = self._extract_joint_names(msg)
                        if names:
                            action_joint_names_by_topic[topic] = names

        if not state_messages:
            self._log_warning(f"No state messages found in {bag_path}")
            return None

        action_messages = self._merge_action_messages(
            action_messages_by_topic, action_joint_names_by_topic
        )

        episode, staleness_metrics = self._resample_to_fps(
            episode, state_messages, action_messages, trim_start
        )

        self._staleness_reports[episode_index] = staleness_metrics
        self._log_staleness_summary(staleness_metrics)

        return episode

    def _is_state_topic(self, topic: str, topic_types: Dict[str, str]) -> bool:
        """Check if topic is a state topic."""
        if self.config.state_topics:
            return topic in self.config.state_topics

        # Default heuristics
        topic_type = topic_types.get(topic, "")
        if "JointState" in topic_type:
            if "follower" in topic.lower() or "state" in topic.lower():
                return True
        return False

    def _is_action_topic(self, topic: str, topic_types: Dict[str, str]) -> bool:
        """Check if topic is an action topic."""
        if self.config.action_topics:
            return topic in self.config.action_topics

        # Default heuristics
        topic_type = topic_types.get(topic, "")
        if "JointTrajectory" in topic_type or "JointState" in topic_type:
            if (
                "leader" in topic.lower()
                or "action" in topic.lower()
                or "command" in topic.lower()
            ):
                return True
        return False

    def _extract_action_positions(self, msg) -> Optional[np.ndarray]:
        """Extract position values from action message."""
        # JointTrajectory message
        if hasattr(msg, "points") and msg.points:
            point = msg.points[0]
            if hasattr(point, "positions") and point.positions:
                return np.array(point.positions, dtype=np.float32)

        # JointState message
        if hasattr(msg, "position") and msg.position:
            return np.array(msg.position, dtype=np.float32)

        return None

    def _extract_joint_names(self, msg) -> List[str]:
        """Extract joint names from message."""
        if hasattr(msg, "joint_names") and msg.joint_names:
            return list(msg.joint_names)
        if hasattr(msg, "name") and msg.name:
            return list(msg.name)
        return []

    def _filter_positions_by_joint_order(
        self,
        positions: np.ndarray,
        msg_names: List[str],
        joint_order: List[str],
    ) -> Optional[np.ndarray]:
        """
        Filter positions array to only include joints in joint_order.

        Args:
            positions: Array of joint positions from message
            msg_names: Joint names from message (same order as positions)
            joint_order: Ordered list of joints to include in output

        Returns:
            Filtered positions array with only joints in joint_order,
            or None if any joint in joint_order is missing from msg_names.
        """
        if len(positions) != len(msg_names):
            self._log_warning(
                f"Position/name length mismatch: {len(positions)} vs {len(msg_names)}"
            )
            return None

        # Build name-to-index mapping
        name_to_idx = {name: idx for idx, name in enumerate(msg_names)}

        # Extract positions in joint_order
        filtered = []
        for joint_name in joint_order:
            if joint_name not in name_to_idx:
                self._log_warning(
                    f"Joint '{joint_name}' from joint_order not found in message"
                )
                return None
            filtered.append(positions[name_to_idx[joint_name]])

        return np.array(filtered, dtype=np.float32)

    def _is_in_exclude_region(
        self, timestamp: float, exclude_regions: List[Dict]
    ) -> bool:
        """Check if timestamp falls within any exclude region."""
        for region in exclude_regions:
            start = region.get("start", {}).get("time", 0)
            end = region.get("end", {}).get("time", 0)
            if start <= timestamp <= end:
                return True
        return False

    def _merge_action_messages(
        self,
        action_messages_by_topic: Dict[str, List[Tuple[float, np.ndarray]]],
        action_joint_names_by_topic: Dict[str, List[str]],
    ) -> List[Tuple[float, np.ndarray]]:
        """Merge action messages from multiple topics into a single action vector."""
        if not action_messages_by_topic:
            return []

        # Sort topics for consistent ordering
        sorted_topics = sorted(action_messages_by_topic.keys())

        # Build combined joint names
        combined_names = []
        for topic in sorted_topics:
            names = action_joint_names_by_topic.get(topic, [])
            combined_names.extend(names)
        self._action_joint_names = combined_names

        # Get all unique timestamps
        all_timestamps = set()
        for msgs in action_messages_by_topic.values():
            for t, _ in msgs:
                all_timestamps.add(t)

        # For each timestamp, concatenate actions from all topics
        # Only include timestamps where ALL topics have valid previous values
        merged_messages: List[Tuple[float, np.ndarray]] = []

        for timestamp in sorted(all_timestamps):
            combined_action = []
            all_topics_have_data = True

            for topic in sorted_topics:
                msgs = action_messages_by_topic[topic]
                prev_value, _ = self._find_previous_value_in_list(
                    msgs, timestamp, tolerance=0.05
                )
                if prev_value is not None:
                    combined_action.extend(prev_value.tolist())
                else:
                    all_topics_have_data = False
                    break

            if all_topics_have_data and combined_action:
                merged_messages.append(
                    (timestamp, np.array(combined_action, dtype=np.float32))
                )

        return merged_messages

    def _find_previous_value_in_list(
        self,
        messages: List[Tuple[float, np.ndarray]],
        target_time: float,
        tolerance: float = float("inf"),
    ) -> Tuple[Optional[np.ndarray], float]:
        """
        Find the most recent message value at or before target time (causal sync).

        Returns:
            Tuple of (value, staleness_ms) where staleness_ms is how old the value is.
            Returns (None, 0.0) if no valid previous value exists.
        """
        if not messages:
            return None, 0.0

        best_time: Optional[float] = None
        best_value: Optional[np.ndarray] = None

        for msg_time, value in messages:
            if msg_time <= target_time:
                if best_time is None or msg_time > best_time:
                    best_time = msg_time
                    best_value = value

        if best_value is None or best_time is None:
            return None, 0.0

        staleness_ms = (target_time - best_time) * 1000.0
        if staleness_ms > tolerance * 1000.0:
            return None, staleness_ms

        return best_value, staleness_ms

    def _resample_to_fps(
        self,
        episode: EpisodeData,
        state_messages: List[Tuple[float, np.ndarray]],
        action_messages: List[Tuple[float, np.ndarray]],
        start_time: float,
    ) -> Tuple[EpisodeData, Dict[str, StalenessMetrics]]:
        """Resample messages to target FPS using causal sync (previous value only)."""
        staleness_metrics: Dict[str, StalenessMetrics] = {
            "observation.state": StalenessMetrics(topic="observation.state"),
            "action": StalenessMetrics(topic="action"),
        }

        if not state_messages:
            return episode, staleness_metrics

        state_times = [t for t, _ in state_messages]
        min_time = min(state_times)
        max_time = max(state_times)

        # Find the first valid start time where both state AND action have data
        # This avoids zero-filled frames at the beginning
        effective_min_time = min_time
        if action_messages:
            action_times = [t for t, _ in action_messages]
            first_action_time = min(action_times)
            # Start from the later of first state or first action
            effective_min_time = max(min_time, first_action_time)
            if effective_min_time > min_time:
                self._log_info(
                    f"Adjusted start time: state_start={min_time:.3f}, "
                    f"action_start={first_action_time:.3f}, "
                    f"effective_start={effective_min_time:.3f}"
                )

        frame_duration = 1.0 / self.config.fps
        num_frames = int((max_time - effective_min_time) * self.config.fps) + 1

        state_staleness_values: List[float] = []
        action_staleness_values: List[float] = []

        action_dim = 0
        if action_messages:
            # Find first valid action message to get dimension
            for _, action_arr in action_messages:
                if len(action_arr) > 0:
                    action_dim = len(action_arr)
                    break

        warning_threshold_ms = (
            1000.0 / self.config.fps
        ) * self.config.quality_warning_multiplier
        error_threshold_ms = (
            1000.0 / self.config.fps
        ) * self.config.quality_error_multiplier

        for frame_idx in range(num_frames):
            target_time = effective_min_time + frame_idx * frame_duration
            # Relative time is from effective start
            relative_time = target_time - effective_min_time

            state, state_staleness_ms = self._find_previous_value(
                state_messages, target_time, frame_duration
            )
            if state is None:
                continue

            staleness_metrics["observation.state"].total_samples += 1
            state_staleness_values.append(state_staleness_ms)
            self._track_staleness(
                staleness_metrics["observation.state"],
                frame_idx,
                state_staleness_ms,
                warning_threshold_ms,
                error_threshold_ms,
            )

            if action_messages and action_dim > 0:
                action, action_staleness_ms = self._find_previous_value(
                    action_messages, target_time, frame_duration
                )
                # Skip this frame if no valid action data
                if action is None:
                    self._log_warning(
                        f"Frame {frame_idx}: No action data at t={target_time:.3f}"
                    )
                    continue

                staleness_metrics["action"].total_samples += 1
                action_staleness_values.append(action_staleness_ms)
                self._track_staleness(
                    staleness_metrics["action"],
                    frame_idx,
                    action_staleness_ms,
                    warning_threshold_ms,
                    error_threshold_ms,
                )
            else:
                action = np.zeros(len(state), dtype=np.float32)

            episode.timestamps.append(relative_time)
            episode.observation_state.append(state)
            episode.action.append(action)

        episode.length = len(episode.timestamps)

        if state_staleness_values:
            staleness_metrics["observation.state"].mean_staleness_ms = float(
                np.mean(state_staleness_values)
            )
            staleness_metrics["observation.state"].max_staleness_ms = float(
                np.max(state_staleness_values)
            )

        if action_staleness_values:
            staleness_metrics["action"].mean_staleness_ms = float(
                np.mean(action_staleness_values)
            )
            staleness_metrics["action"].max_staleness_ms = float(
                np.max(action_staleness_values)
            )

        return episode, staleness_metrics

    def _track_staleness(
        self,
        metrics: StalenessMetrics,
        frame_idx: int,
        staleness_ms: float,
        warning_threshold_ms: float,
        error_threshold_ms: float,
    ):
        if staleness_ms > error_threshold_ms:
            metrics.stale_error_count += 1
            metrics.stale_samples.append(
                {
                    "frame_index": frame_idx,
                    "staleness_ms": round(staleness_ms, 2),
                    "severity": "error",
                }
            )
        elif staleness_ms > warning_threshold_ms:
            metrics.stale_warning_count += 1
            metrics.stale_samples.append(
                {
                    "frame_index": frame_idx,
                    "staleness_ms": round(staleness_ms, 2),
                    "severity": "warning",
                }
            )

    def _find_previous_value(
        self,
        messages: List[Tuple[float, np.ndarray]],
        target_time: float,
        expected_interval_sec: float,
    ) -> Tuple[Optional[np.ndarray], float]:
        """
        Find the most recent message value at or before target time (causal sync).

        Args:
            messages: List of (timestamp, value) tuples
            target_time: Target time to find previous value for
            expected_interval_sec: Expected interval between messages (for staleness calc)

        Returns:
            Tuple of (value, staleness_ms). Returns (None, 0.0) if no previous value.
        """
        if not messages:
            return None, 0.0

        best_time: Optional[float] = None
        best_value: Optional[np.ndarray] = None

        for msg_time, value in messages:
            if msg_time <= target_time:
                if best_time is None or msg_time > best_time:
                    best_time = msg_time
                    best_value = value

        if best_value is None or best_time is None:
            return None, 0.0

        staleness_ms = (target_time - best_time) * 1000.0
        return best_value, staleness_ms

    def _find_video_files(self, bag_path: Path) -> Dict[str, Path]:
        """Find MP4 video files in the rosbag directory."""
        video_files = {}

        search_paths = [bag_path, bag_path / "videos"]

        for search_path in search_paths:
            if not search_path.exists():
                continue

            for mp4_file in search_path.glob("*_compressed.mp4"):
                camera_name = self._get_camera_name_for_video(mp4_file.stem)
                if camera_name not in video_files:
                    video_files[camera_name] = mp4_file

            for mp4_file in search_path.glob("*.mp4"):
                if "_compressed" not in mp4_file.stem:
                    camera_name = self._get_camera_name_for_video(mp4_file.stem)
                    if camera_name not in video_files:
                        video_files[camera_name] = mp4_file

        return video_files

    def _get_camera_name_for_video(self, filename: str) -> str:
        """Get camera name from video filename using camera_mapping if available."""
        name = filename.replace("_compressed", "")

        if self._camera_mapping:
            for topic, camera_name in self._camera_mapping.items():
                sanitized_topic = topic.replace("/", "_").lstrip("_")
                if sanitized_topic in name or name in sanitized_topic:
                    self._log_info(
                        f"Mapped video '{filename}' to camera '{camera_name}'"
                    )
                    return camera_name

        return self._extract_camera_name_fallback(name)

    def _extract_camera_name_fallback(self, filename: str) -> str:
        """Fallback: extract camera name from video filename heuristically."""
        parts = filename.split("_")
        if "camera" in parts:
            idx = parts.index("camera")
            if idx + 1 < len(parts):
                return parts[idx + 1]

        return filename.replace("/", "_").replace(".", "_")

    def _get_video_dimensions(self, video_path: Path) -> Tuple[int, int]:
        """Get video height and width using OpenCV."""
        try:
            import cv2

            cap = cv2.VideoCapture(str(video_path))
            if cap.isOpened():
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                cap.release()
                if width > 0 and height > 0:
                    return height, width
        except Exception as e:
            self._log_warning(f"Failed to get video dimensions: {e}")
        return 480, 640

    def convert_multiple_rosbags(
        self,
        bag_paths: List[Path],
    ) -> bool:
        """
        Convert multiple ROSbag recordings to a single LeRobot dataset.

        Args:
            bag_paths: List of paths to ROSbag directories

        Returns:
            True if successful, False otherwise
        """
        self._log_info(f"Converting {len(bag_paths)} rosbags to LeRobot dataset")

        # Initialize output directory
        output_dir = Path(self.config.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        episodes_data: List[EpisodeData] = []

        # Convert each rosbag
        for idx, bag_path in enumerate(bag_paths):
            episode_data = self.convert_single_rosbag(Path(bag_path), idx)
            if episode_data is not None:
                episodes_data.append(episode_data)

        if not episodes_data:
            self._log_error("No episodes were successfully converted")
            return False

        self._build_features(episodes_data)
        self._write_dataset(episodes_data)

        if self.config.enable_quality_report and self._quality_reports:
            self._write_quality_reports(output_dir)

        self._log_info(f"Successfully converted {len(episodes_data)} episodes")
        return True

    def _write_quality_reports(self, output_dir: Path):
        meta_dir = output_dir / "meta"
        meta_dir.mkdir(parents=True, exist_ok=True)

        combined_report: Dict[str, Any] = {
            "overall_status": "GOOD",
            "total_warnings": 0,
            "total_errors": 0,
            "episodes": {},
        }

        has_error = False
        has_warning = False

        for episode_idx, report in self._quality_reports.items():
            episode_key = f"episode_{episode_idx:06d}"
            episode_report = report.to_dict()

            staleness = self._staleness_reports.get(episode_idx, {})
            if staleness:
                episode_report["staleness"] = {
                    name: metrics.to_dict() for name, metrics in staleness.items()
                }
                for metrics in staleness.values():
                    combined_report["total_warnings"] += metrics.stale_warning_count
                    combined_report["total_errors"] += metrics.stale_error_count
                    if metrics.status == "ERROR":
                        has_error = True
                    elif metrics.status == "WARNING":
                        has_warning = True

            combined_report["episodes"][episode_key] = episode_report
            combined_report["total_warnings"] += report.total_warnings
            combined_report["total_errors"] += report.total_errors

            if report.overall_status == "ERROR":
                has_error = True
            elif report.overall_status == "WARNING":
                has_warning = True

        if has_error:
            combined_report["overall_status"] = "ERROR"
        elif has_warning:
            combined_report["overall_status"] = "WARNING"

        report_path = meta_dir / "quality_report.json"
        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(combined_report, f, indent=2, ensure_ascii=False)

        self._log_info(f"Wrote quality report: {report_path}")

    def _build_features(self, episodes_data: List[EpisodeData]):
        """Build feature definitions from episode data."""
        # Get dimensions from first episode
        first_ep = episodes_data[0]

        state_dim = (
            len(first_ep.observation_state[0]) if first_ep.observation_state else 0
        )
        action_dim = len(first_ep.action[0]) if first_ep.action else 0

        # Default features (required by LeRobot)
        self._features = {
            "timestamp": {"dtype": "float32", "shape": (1,), "names": None},
            "frame_index": {"dtype": "int64", "shape": (1,), "names": None},
            "episode_index": {"dtype": "int64", "shape": (1,), "names": None},
            "index": {"dtype": "int64", "shape": (1,), "names": None},
            "task_index": {"dtype": "int64", "shape": (1,), "names": None},
        }

        # Add observation.state feature
        if state_dim > 0:
            self._features["observation.state"] = {
                "dtype": "float32",
                "shape": (state_dim,),
                "names": self._state_joint_names
                or [f"joint_{i}" for i in range(state_dim)],
            }

        # Add action feature
        if action_dim > 0:
            self._features["action"] = {
                "dtype": "float32",
                "shape": (action_dim,),
                "names": self._action_joint_names
                or [f"joint_{i}" for i in range(action_dim)],
            }

        # Add video features
        for ep in episodes_data:
            for camera_name, video_path in ep.video_files.items():
                feature_key = f"observation.images.{camera_name}"
                if feature_key not in self._features:
                    height, width = self._get_video_dimensions(video_path)

                    self._features[feature_key] = {
                        "dtype": "video",
                        "shape": (height, width, 3),
                        "names": ["height", "width", "channels"],
                    }

    def _write_dataset(self, episodes_data: List[EpisodeData]):
        """Write all dataset files to output directory."""
        output_dir = Path(self.config.output_dir)

        # Create directory structure
        (output_dir / "meta").mkdir(parents=True, exist_ok=True)
        (output_dir / "data").mkdir(parents=True, exist_ok=True)
        (output_dir / "videos").mkdir(parents=True, exist_ok=True)

        # Collect all tasks
        all_tasks = set()
        for ep in episodes_data:
            all_tasks.update(ep.tasks)

        for idx, task in enumerate(sorted(all_tasks)):
            self._tasks[idx] = task
            self._task_to_index[task] = idx

        # Write episodes
        for episode_data in episodes_data:
            self._write_episode(episode_data)

        # Write metadata files
        self._write_info_json()
        self._write_tasks_jsonl()

    def _write_episode(self, episode: EpisodeData):
        """Write a single episode's data files."""
        output_dir = Path(self.config.output_dir)
        ep_idx = episode.episode_index
        chunk_idx = ep_idx // self.config.chunks_size

        # Create chunk directories
        data_chunk_dir = output_dir / "data" / f"chunk-{chunk_idx:03d}"
        data_chunk_dir.mkdir(parents=True, exist_ok=True)

        video_chunk_dir = output_dir / "videos" / f"chunk-{chunk_idx:03d}"
        video_chunk_dir.mkdir(parents=True, exist_ok=True)

        # Write parquet file
        parquet_path = data_chunk_dir / f"episode_{ep_idx:06d}.parquet"
        self._write_parquet(episode, parquet_path)

        # Copy video files
        for camera_name, src_video in episode.video_files.items():
            video_dir = video_chunk_dir / f"observation.images.{camera_name}"
            video_dir.mkdir(parents=True, exist_ok=True)
            dst_video = video_dir / f"episode_{ep_idx:06d}.mp4"
            shutil.copy2(src_video, dst_video)
            self._log_info(f"Copied video: {src_video.name} -> {dst_video}")

        # Write episode metadata
        episode_dict = {
            "episode_index": ep_idx,
            "tasks": episode.tasks,
            "length": episode.length,
        }
        self._episodes[ep_idx] = episode_dict
        self._append_jsonl(episode_dict, output_dir / "meta" / "episodes.jsonl")

        # Compute and write episode stats
        ep_stats = self._compute_episode_stats(episode)
        self._episodes_stats[ep_idx] = ep_stats
        stats_entry = {
            "episode_index": ep_idx,
            "stats": self._serialize_stats(ep_stats),
        }
        self._append_jsonl(stats_entry, output_dir / "meta" / "episodes_stats.jsonl")

        # Update totals
        self._total_frames += episode.length
        self._total_episodes += 1

    def _write_parquet(self, episode: EpisodeData, parquet_path: Path):
        """Write episode data to parquet file with HuggingFace-compatible schema."""
        num_frames = episode.length

        # Determine dimensions
        state_dim = (
            len(episode.observation_state[0]) if episode.observation_state else 0
        )
        action_dim = len(episode.action[0]) if episode.action else 0

        # Build schema with fixed_size_list for HuggingFace compatibility
        schema_fields = [
            pa.field("timestamp", pa.float32()),
            pa.field("frame_index", pa.int64()),
            pa.field("episode_index", pa.int64()),
            pa.field("index", pa.int64()),
            pa.field("task_index", pa.int64()),
        ]

        if state_dim > 0:
            schema_fields.append(
                pa.field("observation.state", pa.list_(pa.float32(), state_dim))
            )
        if action_dim > 0:
            schema_fields.append(pa.field("action", pa.list_(pa.float32(), action_dim)))

        schema = pa.schema(schema_fields)

        # Build data arrays with explicit types
        arrays = [
            pa.array(
                [float(episode.timestamps[i]) for i in range(num_frames)],
                type=pa.float32(),
            ),
            pa.array(list(range(num_frames)), type=pa.int64()),
            pa.array([episode.episode_index] * num_frames, type=pa.int64()),
            pa.array(
                list(range(self._total_frames, self._total_frames + num_frames)),
                type=pa.int64(),
            ),
        ]

        # Task index
        default_task = episode.tasks[0] if episode.tasks else "default_task"
        task_idx = self._task_to_index.get(default_task, 0)
        arrays.append(pa.array([task_idx] * num_frames, type=pa.int64()))

        # Add observation.state as fixed_size_list
        if episode.observation_state:
            state_values = [
                [float(v) for v in state] for state in episode.observation_state
            ]
            arrays.append(
                pa.array(state_values, type=pa.list_(pa.float32(), state_dim))
            )

        # Add action as fixed_size_list
        if episode.action:
            action_values = [[float(v) for v in action] for action in episode.action]
            arrays.append(
                pa.array(action_values, type=pa.list_(pa.float32(), action_dim))
            )

        # Build HuggingFace metadata
        hf_features = {
            "timestamp": {"dtype": "float32", "_type": "Value"},
            "frame_index": {"dtype": "int64", "_type": "Value"},
            "episode_index": {"dtype": "int64", "_type": "Value"},
            "index": {"dtype": "int64", "_type": "Value"},
            "task_index": {"dtype": "int64", "_type": "Value"},
        }

        if state_dim > 0:
            hf_features["observation.state"] = {
                "feature": {"dtype": "float32", "_type": "Value"},
                "length": state_dim,
                "_type": "Sequence",
            }
        if action_dim > 0:
            hf_features["action"] = {
                "feature": {"dtype": "float32", "_type": "Value"},
                "length": action_dim,
                "_type": "Sequence",
            }

        hf_metadata = json.dumps({"info": {"features": hf_features}})

        # Add metadata to schema
        schema = schema.with_metadata({"huggingface": hf_metadata})

        # Create table with schema
        table = pa.table(
            dict(zip([f.name for f in schema_fields], arrays)), schema=schema
        )
        pq.write_table(table, parquet_path)
        self._log_info(f"Wrote parquet: {parquet_path}")

    def _compute_episode_stats(self, episode: EpisodeData) -> Dict[str, Dict]:
        """Compute statistics for an episode (LeRobot v2.1 format)."""
        stats = {}
        num_frames = episode.length

        if episode.observation_state:
            states = np.array(episode.observation_state)
            stats["observation.state"] = {
                "mean": np.mean(states, axis=0).tolist(),
                "std": np.std(states, axis=0).tolist(),
                "min": np.min(states, axis=0).tolist(),
                "max": np.max(states, axis=0).tolist(),
                "count": [num_frames],
            }

        if episode.action:
            actions = np.array(episode.action)
            stats["action"] = {
                "mean": np.mean(actions, axis=0).tolist(),
                "std": np.std(actions, axis=0).tolist(),
                "min": np.min(actions, axis=0).tolist(),
                "max": np.max(actions, axis=0).tolist(),
                "count": [num_frames],
            }

        for camera_name, video_path in episode.video_files.items():
            feature_key = f"observation.images.{camera_name}"
            video_stats = self._compute_video_stats(video_path)
            if video_stats:
                stats[feature_key] = video_stats

        return stats

    def _compute_video_stats(
        self, video_path: Path, max_samples: int = 100
    ) -> Optional[Dict]:
        """Compute video statistics (per-channel RGB, normalized to [0,1])."""
        try:
            import cv2

            cap = cv2.VideoCapture(str(video_path))
            if not cap.isOpened():
                return None

            total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            sample_indices = np.linspace(
                0, total_frames - 1, min(max_samples, total_frames), dtype=int
            )

            samples = []
            for idx in sample_indices:
                cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
                ret, frame = cap.read()
                if ret:
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    samples.append(frame_rgb)

            cap.release()

            if not samples:
                return None

            frames = np.array(samples, dtype=np.float32) / 255.0
            r_channel = frames[:, :, :, 0]
            g_channel = frames[:, :, :, 1]
            b_channel = frames[:, :, :, 2]

            def channel_stats(channel):
                return {
                    "mean": float(np.mean(channel)),
                    "std": float(np.std(channel)),
                    "min": float(np.min(channel)),
                    "max": float(np.max(channel)),
                }

            r_stats = channel_stats(r_channel)
            g_stats = channel_stats(g_channel)
            b_stats = channel_stats(b_channel)

            return {
                "min": [[[r_stats["min"]]], [[g_stats["min"]]], [[b_stats["min"]]]],
                "max": [[[r_stats["max"]]], [[g_stats["max"]]], [[b_stats["max"]]]],
                "mean": [[[r_stats["mean"]]], [[g_stats["mean"]]], [[b_stats["mean"]]]],
                "std": [[[r_stats["std"]]], [[g_stats["std"]]], [[b_stats["std"]]]],
                "count": [len(samples)],
            }
        except Exception as e:
            self._log_warning(f"Failed to compute video stats for {video_path}: {e}")
            return None

    def _serialize_stats(self, stats: Dict) -> Dict:
        """Serialize stats dictionary for JSON."""
        serialized = {}
        for key, value in stats.items():
            if isinstance(value, dict):
                serialized[key] = self._serialize_stats(value)
            elif isinstance(value, np.ndarray):
                serialized[key] = value.tolist()
            elif isinstance(value, (list, int, float)):
                serialized[key] = value
            else:
                serialized[key] = str(value)
        return serialized

    def _write_info_json(self):
        """Write info.json metadata file."""
        output_dir = Path(self.config.output_dir)

        num_video_keys = sum(
            1 for k in self._features if k.startswith("observation.images.")
        )

        info = {
            "codebase_version": CODEBASE_VERSION,
            "robot_type": self.config.robot_type,
            "total_episodes": self._total_episodes,
            "total_frames": self._total_frames,
            "total_tasks": len(self._tasks),
            "total_videos": self._total_episodes * num_video_keys,
            "total_chunks": (self._total_episodes // self.config.chunks_size) + 1,
            "chunks_size": self.config.chunks_size,
            "fps": self.config.fps,
            "splits": {"train": f"0:{self._total_episodes}"},
            "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"
            if self.config.use_videos
            else None,
            "features": self._features,
        }

        info_path = output_dir / "meta" / "info.json"
        with open(info_path, "w", encoding="utf-8") as f:
            json.dump(info, f, indent=4, ensure_ascii=False)

        self._log_info(f"Wrote info.json: {info_path}")

    def _write_tasks_jsonl(self):
        """Write tasks.jsonl metadata file."""
        output_dir = Path(self.config.output_dir)
        tasks_path = output_dir / "meta" / "tasks.jsonl"

        with open(tasks_path, "w", encoding="utf-8") as f:
            for task_idx, task in self._tasks.items():
                entry = {"task_index": task_idx, "task": task}
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")

        self._log_info(f"Wrote tasks.jsonl: {tasks_path}")

    def _append_jsonl(self, data: Dict, filepath: Path):
        """Append a single entry to a JSONL file."""
        filepath.parent.mkdir(parents=True, exist_ok=True)
        with open(filepath, "a", encoding="utf-8") as f:
            f.write(json.dumps(data, ensure_ascii=False) + "\n")


def convert_rosbags_to_lerobot(
    bag_paths: List[str],
    output_dir: str,
    repo_id: str,
    fps: int = DEFAULT_FPS,
    robot_type: str = "unknown",
    logger=None,
) -> bool:
    """
    Convenience function to convert multiple ROSbags to LeRobot dataset.

    Args:
        bag_paths: List of paths to ROSbag directories
        output_dir: Output directory for the dataset
        repo_id: Repository ID for the dataset (e.g., "user/dataset_name")
        fps: Target frames per second
        robot_type: Robot type identifier
        logger: Optional logger instance

    Returns:
        True if successful, False otherwise

    Example:
        >>> convert_rosbags_to_lerobot(
        ...     bag_paths=["/data/rosbag_001", "/data/rosbag_002"],
        ...     output_dir="/datasets/my_robot_dataset",
        ...     repo_id="robotis/ai_worker_pick_place",
        ...     fps=30,
        ...     robot_type="ai_worker",
        ... )
    """
    config = ConversionConfig(
        repo_id=repo_id,
        output_dir=Path(output_dir),
        fps=fps,
        robot_type=robot_type,
    )

    converter = RosbagToLerobotConverter(config, logger)
    return converter.convert_multiple_rosbags([Path(p) for p in bag_paths])
