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

import os
import time
import requests
from pathlib import Path

from lerobot.common.datasets.utils import DEFAULT_FEATURES
from lerobot.common.robot_devices.control_configs import RecordControlConfig
from physical_ai_manager.data_processing.lerobot_dataset_wrapper import LeRobotDatasetWrapper

class DataSaver:

    def __init__(
            self,
            repo_id,
            save_path,
            task_instruction,
            save_fps=30,
            warmup_time_s=10,
            episode_time_s=60,
            reset_time_s=10,
            num_episodes=50,
            video=True,
            push_to_hub=False,
            private=False,
            resume=False,
            num_image_writer_processes=1,
            num_image_writer_threads_per_camera=3):

        self._record_config = RecordControlConfig(
            repo_id=repo_id,
            single_task=task_instruction,
            root=save_path,
            fps=save_fps,
            warmup_time_s=warmup_time_s,
            episode_time_s=episode_time_s,
            reset_time_s=reset_time_s,
            num_episodes=num_episodes,
            video=video,
            push_to_hub=push_to_hub,
            private=private,
            num_image_writer_processes=num_image_writer_processes,
            num_image_writer_threads_per_camera=num_image_writer_threads_per_camera,
        )

        self._lerobot_dataset = None
        self._task_instruction = task_instruction
        self._record_episode_count = 0
        self._start_time_s = 0
        self._warmup = False

    def record(
            self,
            images,
            state,
            action,
            joint_list):

        if self._start_time_s == 0:
            self._start_time_s = time.perf_counter()

        if self._check_warmup_time():
            return

        if not self._check_lerobot_dataset(images, joint_list):
            return

        frame = {}
        for camera_name, image in images.items():
            frame[f'observation.images.{camera_name}'] = image
            

        frame['observation.state'] = state
        frame['action'] = action
        frame['task'] = self._task_instruction
        self._lerobot_dataset.add_frame_without_save_image(frame)

        timestamp = time.perf_counter() - self._start_time_s
        if timestamp > self._record_config.episode_time_s:
            self.save()

    def save(self):
        self._lerobot_dataset.save_episode()
        self._record_episode_count += 1
        self._start_time_s = 0
        self._warmup = True

    def clear(self):
        self._lerobot_dataset.clear_episode_buffer()
        
    def _create_dataset(
            self,
            repo_id,
            images,
            joint_list) -> LeRobotDatasetWrapper:

        features = DEFAULT_FEATURES.copy()
        for camera_name, image in images.items():
            features[f'observation.images.{camera_name}'] = {
                'dtype': 'video',
                'names': ['channels', 'height', 'width'],
                'shape': image.shape
            }

        features['observation.state'] = {
            'dtype': 'float32',
            'names': joint_list,
            'shape': (len(joint_list),)
        }

        features['action'] = {
            'dtype': 'float32',
            'names': joint_list,
            'shape': (len(joint_list),)
        }

        self._lerobot_dataset = LeRobotDatasetWrapper.create(
            repo_id=repo_id,
            fps=self._record_config.fps,
            features=features,
            use_videos=True
        )

    def _check_warmup_time(self):
        if self._warmup:
            timestamp = time.perf_counter() - self._start_time_s
            if timestamp > self._record_config.warmup_time_s:
                self._warmup = False
                self._start_time_s = time.perf_counter()
                return False
            else:
                return True
        return False

    def _check_dataset_exists(self, repo_id, root):
        # Local dataset check
        if os.path.exists(root):
            return True

        # Huggingface dataset check
        url = f"https://huggingface.co/api/datasets/{repo_id}"
        response = requests.get(url)
        url_exist_code = 200

        if response.status_code == url_exist_code:
            return True

        return False

    def _check_lerobot_dataset(self, images, joint_list):
        try:
            if self._lerobot_dataset is None:
                if self._check_dataset_exists(
                        self._record_config.repo_id,
                        self._record_config.root):

                    self._lerobot_dataset = LeRobotDatasetWrapper(
                        self._record_config.repo_id,
                        self._record_config.root
                    )
                else:
                    self._create_dataset(
                        self._record_config.repo_id,
                        images, joint_list)
            return True
        except Exception as e:
            self.get_logger().error(f'Error checking lerobot dataset: {e}')
            return False
