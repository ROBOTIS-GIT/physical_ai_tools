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

import os
import tempfile
from pathlib import Path
import pytest
import yaml

import sys

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "physical_ai_server"))

from data_processing.metadata_manager import MetadataManager


class TestMetadataManager:
    @pytest.fixture
    def temp_bag_dir(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            yield Path(tmpdir)

    @pytest.fixture
    def manager(self):
        return MetadataManager()

    def test_load_robot_config_not_found(self, manager, temp_bag_dir):
        result = manager.load_robot_config(temp_bag_dir)
        assert result is None

    def test_load_robot_config_success(self, manager, temp_bag_dir):
        config_data = {
            "robot_type": "ai_worker",
            "camera_topics": {"cam_head": "/zed/image"},
        }
        config_path = temp_bag_dir / "robot_config.yaml"
        with open(config_path, "w") as f:
            yaml.dump(config_data, f)

        result = manager.load_robot_config(temp_bag_dir)
        assert result is not None
        assert result["robot_type"] == "ai_worker"
        assert "camera_topics" in result

    def test_save_robot_config(self, manager, temp_bag_dir):
        config_data = {"robot_type": "test_robot", "task_markers": []}

        success = manager.save_robot_config(temp_bag_dir, config_data)
        assert success is True

        config_path = temp_bag_dir / "robot_config.yaml"
        assert config_path.exists()

        with open(config_path, "r") as f:
            loaded = yaml.safe_load(f)
        assert loaded["robot_type"] == "test_robot"

    def test_get_directory_size(self, manager, temp_bag_dir):
        test_file = temp_bag_dir / "test.txt"
        with open(test_file, "w") as f:
            f.write("x" * 100)

        size = manager.get_directory_size(temp_bag_dir)
        assert size >= 100

    def test_get_task_markers_empty(self, manager, temp_bag_dir):
        markers = manager.get_task_markers(temp_bag_dir)
        assert markers == []

    def test_get_task_markers_success(self, manager, temp_bag_dir):
        config_data = {
            "task_markers": [
                {"frame": 0, "time": 0.0, "instruction": "Start"},
                {"frame": 100, "time": 3.33, "instruction": "Pick up"},
            ]
        }
        config_path = temp_bag_dir / "robot_config.yaml"
        with open(config_path, "w") as f:
            yaml.dump(config_data, f)

        markers = manager.get_task_markers(temp_bag_dir)
        assert len(markers) == 2
        assert markers[0]["instruction"] == "Start"

    def test_update_task_markers(self, manager, temp_bag_dir):
        markers = [
            {"frame": 50, "time": 1.5, "instruction": "Move"},
            {"frame": 10, "time": 0.3, "instruction": "Init"},
        ]

        result = manager.update_task_markers(temp_bag_dir, markers)
        assert result["success"] is True

        loaded_markers = manager.get_task_markers(temp_bag_dir)
        assert len(loaded_markers) == 2
        assert loaded_markers[0]["frame"] == 10
        assert loaded_markers[1]["frame"] == 50

    def test_update_task_markers_with_trim_points(self, manager, temp_bag_dir):
        markers = [{"frame": 0, "time": 0.0, "instruction": "Start"}]
        trim_points = {
            "start": {"time": 1.0, "frame": 30},
            "end": {"time": 10.0, "frame": 300},
        }

        result = manager.update_task_markers(
            temp_bag_dir, markers, trim_points=trim_points
        )
        assert result["success"] is True

        loaded_trim = manager.get_trim_points(temp_bag_dir)
        assert loaded_trim is not None
        assert loaded_trim["start"]["time"] == 1.0

    def test_get_camera_name_map(self, manager, temp_bag_dir):
        config_data = {
            "camera_topics": {
                "cam_head": "/zed/image",
                "cam_wrist_left": "/camera_left/image",
            }
        }
        config_path = temp_bag_dir / "robot_config.yaml"
        with open(config_path, "w") as f:
            yaml.dump(config_data, f)

        camera_map = manager.get_camera_name_map(temp_bag_dir)
        assert "/zed/image" in camera_map
        assert camera_map["/zed/image"] == "cam_head"

    def test_is_action_topic_from_config(self, manager, temp_bag_dir):
        config_data = {
            "action_topics": {
                "left_arm": "/leader/left/trajectory",
                "right_arm": "/leader/right/trajectory",
            }
        }
        config_path = temp_bag_dir / "robot_config.yaml"
        with open(config_path, "w") as f:
            yaml.dump(config_data, f)

        assert manager.is_action_topic("/leader/left/trajectory", temp_bag_dir) is True
        assert manager.is_action_topic("/follower/left/state", temp_bag_dir) is False

    def test_is_action_topic_fallback(self, manager, temp_bag_dir):
        assert manager.is_action_topic("/leader/left/trajectory", temp_bag_dir) is True
        assert manager.is_action_topic("/some_action_topic", temp_bag_dir) is True
        assert manager.is_action_topic("/some_state_topic", temp_bag_dir) is False


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
