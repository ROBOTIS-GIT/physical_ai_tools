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
from .video_metadata_extractor import VideoMetadataExtractor


CODEBASE_VERSION = "v2.1"
DEFAULT_CHUNK_SIZE = 1000
DEFAULT_FPS = 30


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

        # Dataset state
        self._features: Dict[str, Dict] = {}
        self._tasks: Dict[int, str] = {}
        self._task_to_index: Dict[str, int] = {}
        self._episodes: Dict[int, Dict] = {}
        self._episodes_stats: Dict[int, Dict] = {}
        self._total_frames = 0
        self._total_episodes = 0

        # Joint name mappings (populated from first rosbag)
        self._state_joint_names: List[str] = []
        self._action_joint_names: List[str] = []

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

    def convert_single_rosbag(
        self,
        bag_path: Path,
        episode_index: int,
    ) -> Optional[EpisodeData]:
        """
        Convert a single ROSbag recording to episode data.

        Args:
            bag_path: Path to the ROSbag directory
            episode_index: Index for this episode in the dataset

        Returns:
            EpisodeData if successful, None otherwise
        """
        bag_path = Path(bag_path)
        if not bag_path.exists():
            self._log_error(f"Bag path does not exist: {bag_path}")
            return None

        self._log_info(f"Converting rosbag: {bag_path} (episode {episode_index})")

        # Load robot_config.yaml for metadata
        robot_config = self._metadata_manager.load_robot_config(bag_path)
        if robot_config:
            self._update_config_from_robot_config(robot_config)

        # Get trim points and exclude regions
        trim_points = None
        exclude_regions = []
        if self.config.apply_trim:
            trim_points = self._metadata_manager.get_trim_points(bag_path)
        if self.config.apply_exclude_regions:
            exclude_regions = self._metadata_manager.get_exclude_regions(bag_path)

        # Extract joint data from rosbag
        episode_data = self._extract_joint_data(
            bag_path, episode_index, trim_points, exclude_regions
        )
        if episode_data is None:
            return None

        # Find and process video files
        video_files = self._find_video_files(bag_path)
        episode_data.video_files = video_files

        # Extract tasks from task markers
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
        action_messages: List[Tuple[float, np.ndarray]] = []

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
                    positions = np.array(msg.position, dtype=np.float32)
                    state_messages.append((timestamp, positions))

                    # Capture joint names on first message
                    if (
                        not self._state_joint_names
                        and hasattr(msg, "name")
                        and msg.name
                    ):
                        self._state_joint_names = list(msg.name)

            # Process action topics (JointTrajectory or JointState)
            elif self._is_action_topic(topic, topic_types):
                positions = self._extract_action_positions(msg)
                if positions is not None:
                    action_messages.append((timestamp, positions))

                    # Capture action joint names
                    if not self._action_joint_names:
                        names = self._extract_joint_names(msg)
                        if names:
                            self._action_joint_names = names

        if not state_messages:
            self._log_warning(f"No state messages found in {bag_path}")
            return None

        # Resample to target FPS
        episode = self._resample_to_fps(
            episode, state_messages, action_messages, trim_start
        )

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

    def _resample_to_fps(
        self,
        episode: EpisodeData,
        state_messages: List[Tuple[float, np.ndarray]],
        action_messages: List[Tuple[float, np.ndarray]],
        start_time: float,
    ) -> EpisodeData:
        """Resample messages to target FPS using nearest neighbor interpolation."""
        if not state_messages:
            return episode

        # Determine time range
        state_times = [t for t, _ in state_messages]
        min_time = min(state_times)
        max_time = max(state_times)

        # Generate target timestamps at target FPS
        frame_duration = 1.0 / self.config.fps
        num_frames = int((max_time - min_time) * self.config.fps) + 1

        for frame_idx in range(num_frames):
            target_time = min_time + frame_idx * frame_duration
            relative_time = target_time - min_time

            # Find nearest state
            state = self._find_nearest_value(state_messages, target_time)
            if state is None:
                continue

            # Find nearest action (or use zeros if not available)
            if action_messages:
                action = self._find_nearest_value(action_messages, target_time)
                if action is None:
                    action = np.zeros_like(state)
            else:
                action = np.zeros_like(state)

            episode.timestamps.append(relative_time)
            episode.observation_state.append(state)
            episode.action.append(action)

        episode.length = len(episode.timestamps)
        return episode

    def _find_nearest_value(
        self,
        messages: List[Tuple[float, np.ndarray]],
        target_time: float,
    ) -> Optional[np.ndarray]:
        """Find the message value nearest to target time."""
        if not messages:
            return None

        min_diff = float("inf")
        nearest_value = None

        for msg_time, value in messages:
            diff = abs(msg_time - target_time)
            if diff < min_diff:
                min_diff = diff
                nearest_value = value

        return nearest_value

    def _find_video_files(self, bag_path: Path) -> Dict[str, Path]:
        """Find MP4 video files in the rosbag directory."""
        video_files = {}

        # Look for compressed MP4 files
        for mp4_file in bag_path.glob("*_compressed.mp4"):
            camera_name = self._extract_camera_name(mp4_file.stem)
            video_files[camera_name] = mp4_file

        # Also check for non-compressed MP4s
        for mp4_file in bag_path.glob("*.mp4"):
            if "_compressed" not in mp4_file.stem:
                camera_name = self._extract_camera_name(mp4_file.stem)
                if camera_name not in video_files:
                    video_files[camera_name] = mp4_file

        return video_files

    def _extract_camera_name(self, filename: str) -> str:
        """Extract camera name from video filename."""
        # Remove common suffixes
        name = filename.replace("_compressed", "")

        # Try to extract meaningful camera name
        # Example: "camera_head_image_raw" -> "head"
        parts = name.split("_")
        if "camera" in parts:
            idx = parts.index("camera")
            if idx + 1 < len(parts):
                return parts[idx + 1]

        # Fallback: use sanitized filename
        return name.replace("/", "_").replace(".", "_")

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

        # Build features from collected data
        self._build_features(episodes_data)

        # Write dataset files
        self._write_dataset(episodes_data)

        self._log_info(f"Successfully converted {len(episodes_data)} episodes")
        return True

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
                    # Get video dimensions
                    video_info = self._video_extractor.get_video_info(video_path)
                    if video_info:
                        height = video_info.get("height", 480)
                        width = video_info.get("width", 640)
                    else:
                        height, width = 480, 640

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
        """Write episode data to parquet file."""
        num_frames = episode.length

        # Build data dictionary
        data = {
            "timestamp": [episode.timestamps[i] for i in range(num_frames)],
            "frame_index": list(range(num_frames)),
            "episode_index": [episode.episode_index] * num_frames,
            "index": list(range(self._total_frames, self._total_frames + num_frames)),
        }

        # Add task indices
        default_task = episode.tasks[0] if episode.tasks else "default_task"
        task_idx = self._task_to_index.get(default_task, 0)
        data["task_index"] = [task_idx] * num_frames

        # Add observation.state
        if episode.observation_state:
            data["observation.state"] = [
                state.tolist() for state in episode.observation_state
            ]

        # Add action
        if episode.action:
            data["action"] = [action.tolist() for action in episode.action]

        # Create PyArrow table
        table = pa.table(data)
        pq.write_table(table, parquet_path)
        self._log_info(f"Wrote parquet: {parquet_path}")

    def _compute_episode_stats(self, episode: EpisodeData) -> Dict[str, Dict]:
        """Compute statistics for an episode."""
        stats = {}

        # Compute stats for observation.state
        if episode.observation_state:
            states = np.array(episode.observation_state)
            stats["observation.state"] = {
                "mean": np.mean(states, axis=0).tolist(),
                "std": np.std(states, axis=0).tolist(),
                "min": np.min(states, axis=0).tolist(),
                "max": np.max(states, axis=0).tolist(),
                "count": len(states),
            }

        # Compute stats for action
        if episode.action:
            actions = np.array(episode.action)
            stats["action"] = {
                "mean": np.mean(actions, axis=0).tolist(),
                "std": np.std(actions, axis=0).tolist(),
                "min": np.min(actions, axis=0).tolist(),
                "max": np.max(actions, axis=0).tolist(),
                "count": len(actions),
            }

        return stats

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
