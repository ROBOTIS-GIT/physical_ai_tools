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

"""Video metadata extractor for ROSbag files."""

from pathlib import Path
from typing import Dict, List, Optional, Tuple


class VideoMetadataExtractor:
    """Extracts video metadata from ROSbag image metadata topics."""

    def __init__(self, logger=None):
        self.logger = logger

    def _log_info(self, msg: str):
        if self.logger:
            self.logger.info(msg)

    def _log_error(self, msg: str):
        if self.logger:
            self.logger.error(msg)

    def extract_camera_name_from_topic(self, topic: str) -> str:
        """
        Extract a meaningful camera name from a topic path.

        Examples:
            /zed/zed_node/left/image_rect_color/compressed -> cam_head
            /camera_left/camera_left/color/image_rect_raw/compressed -> cam_wrist_left
            /camera_right/camera_right/color/image_rect_raw/compressed -> cam_wrist_right
        """
        topic_lower = topic.lower()

        if "zed" in topic_lower:
            return "cam_head"
        elif "camera_left" in topic_lower or "cam_left" in topic_lower:
            return "cam_wrist_left"
        elif "camera_right" in topic_lower or "cam_right" in topic_lower:
            return "cam_wrist_right"
        elif "head" in topic_lower:
            return "cam_head"
        elif "wrist" in topic_lower:
            if "left" in topic_lower:
                return "cam_wrist_left"
            elif "right" in topic_lower:
                return "cam_wrist_right"
            return "cam_wrist"

        parts = [
            p
            for p in topic.split("/")
            if p
            and p
            not in ["compressed", "image_rect_raw", "image_rect_color", "color", "rgb"]
        ]
        if parts:
            return parts[0]

        return topic

    def calculate_fps_from_timestamps(
        self, timestamps: List[float], default_fps: float = 30.0
    ) -> float:
        """
        Calculate FPS from a list of timestamps.

        Args:
            timestamps: List of timestamps in seconds
            default_fps: Default FPS if calculation fails

        Returns:
            Calculated FPS value
        """
        if len(timestamps) < 2:
            return default_fps

        time_diffs = [
            timestamps[i + 1] - timestamps[i] for i in range(len(timestamps) - 1)
        ]
        avg_diff = sum(time_diffs) / len(time_diffs)

        if avg_diff > 0:
            return 1.0 / avg_diff
        return default_fps

    def get_video_files(self, bag_path: Path) -> List[str]:
        """
        Get list of video files in the bag's videos directory.

        Args:
            bag_path: Path to the ROSbag directory

        Returns:
            List of relative video file paths (e.g., 'videos/cam_head.mp4')
        """
        videos_dir = Path(bag_path) / "videos"
        if not videos_dir.exists():
            return []

        video_files = []
        for video_file in sorted(videos_dir.glob("*.mp4")):
            video_files.append(f"videos/{video_file.name}")

        return video_files

    def build_video_info(
        self,
        bag_path: Path,
        image_metadata_by_topic: Dict[str, List[Tuple[int, float]]],
        camera_name_map: Dict[str, str],
    ) -> Dict:
        """
        Build video information dictionary.

        Args:
            bag_path: Path to the ROSbag directory
            image_metadata_by_topic: Dict mapping topic -> [(frame_idx, timestamp), ...]
            camera_name_map: Dict mapping topic -> camera name from config

        Returns:
            Dictionary with video_files, video_topics, video_names, video_fps, frame_counts
        """
        result = {
            "video_files": [],
            "video_topics": [],
            "video_names": [],
            "video_fps": [],
            "frame_counts": {},
        }

        videos_dir = Path(bag_path) / "videos"
        if not videos_dir.exists():
            return result

        for video_file in sorted(videos_dir.glob("*.mp4")):
            result["video_files"].append(f"videos/{video_file.name}")

            video_name = video_file.stem
            matching_topic = self._find_matching_topic(
                video_name, image_metadata_by_topic
            )

            result["video_topics"].append(matching_topic or video_name)

            camera_name = self._get_camera_name(
                matching_topic, video_name, camera_name_map
            )
            result["video_names"].append(camera_name)

            fps = self._calculate_topic_fps(matching_topic, image_metadata_by_topic)
            result["video_fps"].append(fps)

        for i, video_name in enumerate(result["video_names"]):
            topic = (
                result["video_topics"][i] if i < len(result["video_topics"]) else None
            )
            if topic and topic in image_metadata_by_topic:
                result["frame_counts"][video_name] = len(image_metadata_by_topic[topic])
            else:
                result["frame_counts"][video_name] = 0

        return result

    def _find_matching_topic(
        self, video_name: str, image_metadata_by_topic: Dict[str, List]
    ) -> Optional[str]:
        """Find the matching topic for a video file."""
        for topic in image_metadata_by_topic.keys():
            sanitized = topic.replace("/", "_").lstrip("_")
            if sanitized == video_name:
                return topic
        return None

    def _get_camera_name(
        self, topic: Optional[str], video_name: str, camera_name_map: Dict[str, str]
    ) -> str:
        """Get camera name from config map or extract from topic/filename."""
        if topic:
            camera_name = camera_name_map.get(topic)
            if camera_name:
                return camera_name
            return self.extract_camera_name_from_topic(topic)
        return self.extract_camera_name_from_topic(video_name)

    def _calculate_topic_fps(
        self,
        topic: Optional[str],
        image_metadata_by_topic: Dict[str, List[Tuple[int, float]]],
    ) -> float:
        """Calculate FPS for a topic from its metadata."""
        if not topic or topic not in image_metadata_by_topic:
            return 30.0

        metadata = image_metadata_by_topic[topic]
        if len(metadata) <= 1:
            return 30.0

        sorted_metadata = sorted(metadata, key=lambda x: x[0])
        timestamps = [ts for _, ts in sorted_metadata]

        return self.calculate_fps_from_timestamps(timestamps)
