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

from huggingface_hub import snapshot_download
from lerobot.common.datasets.utils import DEFAULT_FEATURES
from lerobot.common.robot_devices.control_configs import RecordControlConfig
import numpy as np
from physical_ai_manager.data_processing.lerobot_dataset_wrapper import LeRobotDatasetWrapper


class DataSaver:

    def __init__(
            self,
            repo_id,
            save_path,
            task_instruction,
            use_image_buffer=True,
            save_fps=30,
            reset_time_s=10,
            episode_time_s=10,
            num_episodes=3,
            video=True,
            push_to_hub=False,
            private=False,
            resume=False,
            num_image_writer_processes=12,
            num_image_writer_threads_per_camera=4):

        self._record_config = RecordControlConfig(
            repo_id=repo_id,
            single_task=task_instruction,
            root=save_path,
            fps=save_fps,
            reset_time_s=reset_time_s,
            episode_time_s=episode_time_s,
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
        self.status = 'start'
        self.use_image_buffer = use_image_buffer

    def record(
            self,
            images,
            state,
            action,
            joint_list):
        if self._start_time_s == 0:
            self._start_time_s = time.perf_counter()

        if not self._check_lerobot_dataset(images, joint_list):
            return True

        if self._record_episode_count >= self._record_config.num_episodes:
            if self._lerobot_dataset.check_video_encoding_completed():
                self.upload_dataset('ai_worker', private=False)
                return True
            return False

        if self._check_reset_time():
            return False

        frame = {}
        for camera_name, image in images.items():
            frame[f'observation.images.{camera_name}'] = image
        frame['observation.state'] = np.array(state)
        frame['action'] = np.array(action)
        frame['task'] = self._task_instruction

        if self.use_image_buffer:
            self._lerobot_dataset.add_frame_without_write_image(frame)
        else:
            self._lerobot_dataset.add_frame(frame)

        timestamp = time.perf_counter() - self._start_time_s
        if timestamp > self._record_config.episode_time_s:
            self.save()

        return False

    def save(self):
        if self.use_image_buffer:
            self._lerobot_dataset.save_episode_without_write_image()
        else:
            self._lerobot_dataset.save_episode()

        self._lerobot_dataset.episode_buffer = None
        self._record_episode_count += 1
        self._start_time_s = 0
        self.status = 'reset'

    def record_stop(self):
        self.status = 'stop'
        self._lerobot_dataset.episode_buffer = None
        self._start_time_s = 0

    def _check_reset_time(self):
        if self.status == 'reset':
            timestamp = time.perf_counter() - self._start_time_s
            if timestamp > self._record_config.reset_time_s:
                self.status = 'start'
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
            print(f'Dataset {repo_id} exists on Huggingface, downloading...')
            self.download_dataset(repo_id)
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
            print(f'Error checking lerobot dataset: {e}')
            return False

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

        if not self.use_image_buffer:
            self._lerobot_dataset.start_image_writer(
                    num_processes=self._record_config.num_image_writer_processes,
                    num_threads=self._record_config.num_image_writer_threads_per_camera *
                    len(images),
                )

    def upload_dataset(self, robot_type, private=False):
        tags = [robot_type]
        self._lerobot_dataset.push_to_hub(tags=tags, private=private)

    def download_dataset(self, repo_id):
        snapshot_download(
            repo_id,
            repo_type="dataset",
            local_dir=self._record_config.root,
        )
