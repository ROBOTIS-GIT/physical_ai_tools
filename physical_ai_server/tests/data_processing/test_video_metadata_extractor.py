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

import tempfile
from pathlib import Path
import pytest

import sys

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "physical_ai_server"))

from data_processing.video_metadata_extractor import VideoMetadataExtractor


class TestVideoMetadataExtractor:
    @pytest.fixture
    def temp_bag_dir(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            yield Path(tmpdir)

    @pytest.fixture
    def extractor(self):
        return VideoMetadataExtractor()

    def test_extract_camera_name_zed(self, extractor):
        topic = "/zed/zed_node/left/image_rect_color/compressed"
        assert extractor.extract_camera_name_from_topic(topic) == "cam_head"

    def test_extract_camera_name_camera_left(self, extractor):
        topic = "/camera_left/camera_left/color/image_rect_raw/compressed"
        assert extractor.extract_camera_name_from_topic(topic) == "cam_wrist_left"

    def test_extract_camera_name_camera_right(self, extractor):
        topic = "/camera_right/camera_right/color/image_rect_raw/compressed"
        assert extractor.extract_camera_name_from_topic(topic) == "cam_wrist_right"

    def test_extract_camera_name_head(self, extractor):
        topic = "/head_camera/image"
        assert extractor.extract_camera_name_from_topic(topic) == "cam_head"

    def test_extract_camera_name_wrist_left(self, extractor):
        topic = "/wrist_left_camera/image"
        assert extractor.extract_camera_name_from_topic(topic) == "cam_wrist_left"

    def test_extract_camera_name_wrist_right(self, extractor):
        topic = "/wrist_right_camera/image"
        assert extractor.extract_camera_name_from_topic(topic) == "cam_wrist_right"

    def test_extract_camera_name_fallback(self, extractor):
        topic = "/custom_camera/image"
        assert extractor.extract_camera_name_from_topic(topic) == "custom_camera"

    def test_calculate_fps_from_timestamps_30fps(self, extractor):
        timestamps = [0.0, 0.0333, 0.0666, 0.1, 0.1333]
        fps = extractor.calculate_fps_from_timestamps(timestamps)
        assert 28 < fps < 32

    def test_calculate_fps_from_timestamps_60fps(self, extractor):
        timestamps = [0.0, 0.0166, 0.0333, 0.05, 0.0666]
        fps = extractor.calculate_fps_from_timestamps(timestamps)
        assert 55 < fps < 65

    def test_calculate_fps_from_timestamps_empty(self, extractor):
        fps = extractor.calculate_fps_from_timestamps([])
        assert fps == 30.0

    def test_calculate_fps_from_timestamps_single(self, extractor):
        fps = extractor.calculate_fps_from_timestamps([0.0])
        assert fps == 30.0

    def test_get_video_files_empty(self, extractor, temp_bag_dir):
        files = extractor.get_video_files(temp_bag_dir)
        assert files == []

    def test_get_video_files_success(self, extractor, temp_bag_dir):
        videos_dir = temp_bag_dir / "videos"
        videos_dir.mkdir()
        (videos_dir / "cam_head.mp4").touch()
        (videos_dir / "cam_wrist_left.mp4").touch()

        files = extractor.get_video_files(temp_bag_dir)
        assert len(files) == 2
        assert "videos/cam_head.mp4" in files
        assert "videos/cam_wrist_left.mp4" in files

    def test_build_video_info_empty(self, extractor, temp_bag_dir):
        result = extractor.build_video_info(temp_bag_dir, {}, {})
        assert result["video_files"] == []
        assert result["video_topics"] == []
        assert result["video_names"] == []
        assert result["video_fps"] == []
        assert result["frame_counts"] == {}

    def test_build_video_info_with_videos(self, extractor, temp_bag_dir):
        videos_dir = temp_bag_dir / "videos"
        videos_dir.mkdir()
        (videos_dir / "_zed_zed_node_left_image_rect_color_compressed.mp4").touch()

        image_metadata = {
            "/zed/zed_node/left/image_rect_color/compressed": [
                (0, 0.0),
                (1, 0.033),
                (2, 0.066),
                (3, 0.1),
            ]
        }
        camera_name_map = {"/zed/zed_node/left/image_rect_color/compressed": "cam_head"}

        result = extractor.build_video_info(
            temp_bag_dir, image_metadata, camera_name_map
        )
        assert len(result["video_files"]) == 1
        assert result["video_names"][0] == "cam_head"
        assert result["frame_counts"]["cam_head"] == 4


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
