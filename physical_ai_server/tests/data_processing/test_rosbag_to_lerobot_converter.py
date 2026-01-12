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

"""Unit tests for RosbagToLerobotConverter."""

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

from physical_ai_server.data_processing.rosbag_to_lerobot_converter import (
    ConversionConfig,
    EpisodeData,
    RosbagToLerobotConverter,
)


class TestConversionConfig(unittest.TestCase):
    """Tests for ConversionConfig dataclass."""

    def test_default_values(self):
        config = ConversionConfig(
            repo_id="test/dataset",
            output_dir=Path("/tmp/test"),
        )
        self.assertEqual(config.fps, 30)
        self.assertEqual(config.robot_type, "unknown")
        self.assertTrue(config.use_videos)
        self.assertEqual(config.chunks_size, 1000)
        self.assertTrue(config.apply_trim)
        self.assertTrue(config.apply_exclude_regions)

    def test_custom_values(self):
        config = ConversionConfig(
            repo_id="user/my_dataset",
            output_dir=Path("/datasets/output"),
            fps=60,
            robot_type="ai_worker",
            chunks_size=500,
            apply_trim=False,
        )
        self.assertEqual(config.fps, 60)
        self.assertEqual(config.robot_type, "ai_worker")
        self.assertEqual(config.chunks_size, 500)
        self.assertFalse(config.apply_trim)


class TestEpisodeData(unittest.TestCase):
    """Tests for EpisodeData dataclass."""

    def test_default_initialization(self):
        episode = EpisodeData(episode_index=0)
        self.assertEqual(episode.episode_index, 0)
        self.assertEqual(episode.timestamps, [])
        self.assertEqual(episode.observation_state, [])
        self.assertEqual(episode.action, [])
        self.assertEqual(episode.video_files, {})
        self.assertEqual(episode.tasks, [])
        self.assertEqual(episode.length, 0)

    def test_with_data(self):
        episode = EpisodeData(
            episode_index=5,
            timestamps=[0.0, 0.033, 0.066],
            observation_state=[
                np.array([1.0, 2.0]),
                np.array([1.1, 2.1]),
                np.array([1.2, 2.2]),
            ],
            action=[
                np.array([0.1, 0.2]),
                np.array([0.11, 0.21]),
                np.array([0.12, 0.22]),
            ],
            tasks=["pick object"],
            length=3,
        )
        self.assertEqual(episode.episode_index, 5)
        self.assertEqual(len(episode.timestamps), 3)
        self.assertEqual(episode.length, 3)


class TestRosbagToLerobotConverter(unittest.TestCase):
    """Tests for RosbagToLerobotConverter class."""

    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.config = ConversionConfig(
            repo_id="test/dataset",
            output_dir=Path(self.temp_dir),
        )
        self.converter = RosbagToLerobotConverter(self.config)

    def test_initialization(self):
        self.assertEqual(self.converter.config.repo_id, "test/dataset")
        self.assertEqual(self.converter._total_episodes, 0)
        self.assertEqual(self.converter._total_frames, 0)

    def test_extract_camera_name(self):
        test_cases = [
            ("camera_head_image_raw_compressed", "head"),
            ("camera_wrist_left_compressed", "wrist"),
            ("zed_left_image", "zed"),
            ("simple_video", "simple"),
        ]
        for filename, expected_contains in test_cases:
            result = self.converter._extract_camera_name(filename)
            self.assertIsInstance(result, str)
            self.assertGreater(len(result), 0)

    def test_is_in_exclude_region(self):
        exclude_regions = [
            {"start": {"time": 1.0}, "end": {"time": 2.0}},
            {"start": {"time": 5.0}, "end": {"time": 6.0}},
        ]

        self.assertFalse(self.converter._is_in_exclude_region(0.5, exclude_regions))
        self.assertTrue(self.converter._is_in_exclude_region(1.5, exclude_regions))
        self.assertFalse(self.converter._is_in_exclude_region(3.0, exclude_regions))
        self.assertTrue(self.converter._is_in_exclude_region(5.5, exclude_regions))
        self.assertFalse(self.converter._is_in_exclude_region(7.0, exclude_regions))

    def test_find_nearest_value(self):
        messages = [
            (0.0, np.array([1.0])),
            (1.0, np.array([2.0])),
            (2.0, np.array([3.0])),
        ]

        result = self.converter._find_nearest_value(messages, 0.4)
        np.testing.assert_array_equal(result, np.array([1.0]))

        result = self.converter._find_nearest_value(messages, 0.6)
        np.testing.assert_array_equal(result, np.array([2.0]))

        result = self.converter._find_nearest_value(messages, 1.9)
        np.testing.assert_array_equal(result, np.array([3.0]))

    def test_find_nearest_value_empty(self):
        result = self.converter._find_nearest_value([], 1.0)
        self.assertIsNone(result)

    def test_build_features(self):
        episodes = [
            EpisodeData(
                episode_index=0,
                observation_state=[np.array([1.0, 2.0, 3.0])],
                action=[np.array([0.1, 0.2, 0.3])],
                length=1,
            )
        ]
        self.converter._state_joint_names = ["j1", "j2", "j3"]
        self.converter._action_joint_names = ["a1", "a2", "a3"]

        self.converter._build_features(episodes)

        self.assertIn("observation.state", self.converter._features)
        self.assertIn("action", self.converter._features)
        self.assertEqual(self.converter._features["observation.state"]["shape"], (3,))
        self.assertEqual(self.converter._features["action"]["shape"], (3,))

    def test_compute_episode_stats(self):
        episode = EpisodeData(
            episode_index=0,
            observation_state=[
                np.array([1.0, 2.0]),
                np.array([3.0, 4.0]),
                np.array([5.0, 6.0]),
            ],
            action=[
                np.array([0.1, 0.2]),
                np.array([0.3, 0.4]),
                np.array([0.5, 0.6]),
            ],
            length=3,
        )

        stats = self.converter._compute_episode_stats(episode)

        self.assertIn("observation.state", stats)
        self.assertIn("action", stats)
        self.assertIn("mean", stats["observation.state"])
        self.assertIn("std", stats["observation.state"])
        self.assertIn("min", stats["observation.state"])
        self.assertIn("max", stats["observation.state"])

        np.testing.assert_array_almost_equal(
            stats["observation.state"]["mean"], [3.0, 4.0]
        )

    def test_serialize_stats(self):
        stats = {
            "observation.state": {
                "mean": np.array([1.0, 2.0]),
                "std": np.array([0.1, 0.2]),
            }
        }

        serialized = self.converter._serialize_stats(stats)

        self.assertIsInstance(serialized["observation.state"]["mean"], list)
        self.assertEqual(serialized["observation.state"]["mean"], [1.0, 2.0])

    def test_resample_to_fps(self):
        episode = EpisodeData(episode_index=0)
        state_messages = [
            (0.0, np.array([1.0])),
            (0.5, np.array([2.0])),
            (1.0, np.array([3.0])),
        ]
        action_messages = [
            (0.0, np.array([0.1])),
            (0.5, np.array([0.2])),
            (1.0, np.array([0.3])),
        ]

        self.converter.config.fps = 10
        result = self.converter._resample_to_fps(
            episode, state_messages, action_messages, 0.0
        )

        self.assertGreater(result.length, 0)
        self.assertEqual(len(result.timestamps), result.length)
        self.assertEqual(len(result.observation_state), result.length)
        self.assertEqual(len(result.action), result.length)


class TestInfoJsonGeneration(unittest.TestCase):
    """Tests for info.json generation."""

    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.config = ConversionConfig(
            repo_id="test/dataset",
            output_dir=Path(self.temp_dir),
            fps=30,
            robot_type="test_robot",
        )
        self.converter = RosbagToLerobotConverter(self.config)

    def test_write_info_json(self):
        self.converter._features = {
            "timestamp": {"dtype": "float32", "shape": (1,), "names": None},
            "observation.state": {"dtype": "float32", "shape": (6,), "names": None},
            "action": {"dtype": "float32", "shape": (6,), "names": None},
        }
        self.converter._tasks = {0: "test_task"}
        self.converter._total_episodes = 5
        self.converter._total_frames = 500

        (Path(self.temp_dir) / "meta").mkdir(parents=True, exist_ok=True)

        self.converter._write_info_json()

        info_path = Path(self.temp_dir) / "meta" / "info.json"
        self.assertTrue(info_path.exists())

        with open(info_path) as f:
            info = json.load(f)

        self.assertEqual(info["codebase_version"], "v2.1")
        self.assertEqual(info["robot_type"], "test_robot")
        self.assertEqual(info["fps"], 30)
        self.assertEqual(info["total_episodes"], 5)
        self.assertEqual(info["total_frames"], 500)
        self.assertIn("features", info)


if __name__ == "__main__":
    unittest.main()
