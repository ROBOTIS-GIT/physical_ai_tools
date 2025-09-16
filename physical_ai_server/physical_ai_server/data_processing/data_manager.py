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
# Author: Dongyun Kim, Seongwoo Kim

import gc
import os
import shutil
import subprocess
import time
from pathlib import Path

import cv2
from geometry_msgs.msg import Twist
from huggingface_hub import (
    HfApi,
    snapshot_download,
    upload_folder
)
from lerobot.datasets.utils import DEFAULT_FEATURES


from nav_msgs.msg import Odometry
import numpy as np
from physical_ai_interfaces.msg import TaskStatus
from physical_ai_server.data_processing.data_converter import DataConverter
from physical_ai_server.data_processing.lerobot_dataset_wrapper import LeRobotDatasetWrapper
from physical_ai_server.device_manager.cpu_checker import CPUChecker
from physical_ai_server.device_manager.ram_checker import RAMChecker
from physical_ai_server.device_manager.storage_checker import StorageChecker
import requests
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from physical_ai_server.data_processing.progress_tracker import (
    HuggingFaceProgressTqdm
)


class DataManager:
    RECORDING = False
    RECORD_COMPLETED = True
    RAM_LIMIT_GB = 2  # GB
    SKIP_TIME = 0.1  # Seconds

    # Progress queue for multiprocessing communication
    _progress_queue = None

    def __init__(
            self,
            save_root_path,
            robot_type,
            task_info):
        self._robot_type = robot_type
        self._save_repo_name = f'{task_info.user_id}/{robot_type}_{task_info.task_name}'
        self._save_path = save_root_path / self._save_repo_name
        self._on_saving = False
        self._single_task = len(task_info.task_instruction) == 1
        self._task_info = task_info

        self._lerobot_dataset = None
        self._record_episode_count = 0
        self._start_time_s = 0
        self._proceed_time = 0
        self._status = 'warmup'
        self._cpu_checker = CPUChecker()
        self.data_converter = DataConverter()
        self.force_save_for_safety = False
        self._stop_save_completed = False
        self.current_instruction = ''
        self._current_task = 0
        self._init_task_limits()
        self._current_scenario_number = 0

    def record(
            self,
            images,
            state,
            action):

        if self._start_time_s == 0:
            self._start_time_s = time.perf_counter()

        if self._status == 'warmup':
            self._current_task = 0
            self._current_scenario_number = 0
            if not self._check_time(self._task_info.warmup_time_s, 'run'):
                return self.RECORDING

        elif self._status == 'run':
            if not self._check_time(self._task_info.episode_time_s, 'save'):
                if RAMChecker.get_free_ram_gb() < self.RAM_LIMIT_GB:
                    if not self._single_task:
                        self._status = 'finish'
                    else:
                        self.record_early_save()
                    return self.RECORDING
                frame = self.create_frame(images, state, action)
                if self._task_info.use_optimized_save_mode:
                    self._lerobot_dataset.add_frame_without_write_image(
                        frame,
                        self.current_instruction)
                else:
                    self._lerobot_dataset.add_frame(
                        frame,
                        self.current_instruction)

        elif self._status == 'save':
            if self._on_saving:
                if (
                    self._lerobot_dataset.check_video_encoding_completed()
                    or (
                        not self._single_task
                        and self._lerobot_dataset.check_append_buffer_completed()
                    )
                ):
                    self._episode_reset()
                    self._record_episode_count += 1
                    self._get_current_scenario_number()
                    self._current_task += 1
                    self._status = 'reset'
                    self._start_time_s = 0
                    self._on_saving = False
            else:
                self.save()
                self._on_saving = True

        elif self._status == 'reset':
            if not self._single_task:
                if not self._check_time(self.SKIP_TIME, 'run'):
                    return self.RECORDING
            else:
                if not self._check_time(self._task_info.reset_time_s, 'run'):
                    return self.RECORDING

        elif self._status == 'skip_task':
            if not self._check_time(self.SKIP_TIME, 'run'):
                return self.RECORDING

        elif self._status == 'stop':
            if not self._stop_save_completed:
                if self._on_saving:
                    if self._lerobot_dataset.check_video_encoding_completed():
                        self._on_saving = False
                        self._episode_reset()
                        self._record_episode_count += 1
                        self._get_current_scenario_number()
                        self._current_task += 1
                        self._stop_save_completed = True
                else:
                    self.save()
                    self._proceed_time = 0
                    self._on_saving = True
            return self.RECORDING

        elif self._status == 'finish':
            if self._on_saving:
                if self._lerobot_dataset.check_video_encoding_completed():
                    self._on_saving = False
                    self._episode_reset()
                    if (self._task_info.push_to_hub and
                            self._record_episode_count > 0):
                        self._upload_dataset(
                            self._task_info.tags,
                            self._task_info.private_mode)
                    return self.RECORD_COMPLETED
            else:
                self.save()
                if not self._single_task:
                    self._lerobot_dataset.video_encoding()
                self._proceed_time = 0
                self._on_saving = True

        if self._record_episode_count >= self._task_info.num_episodes:
            if self._lerobot_dataset.check_video_encoding_completed():
                if (self._task_info.push_to_hub and
                        self._record_episode_count > 0):
                    self._upload_dataset(
                        self._task_info.tags,
                        self._task_info.private_mode)
                return self.RECORD_COMPLETED

        return self.RECORDING

    def save(self):
        if self._lerobot_dataset.episode_buffer is None:
            return
        if self._task_info.use_optimized_save_mode:
            if not self._single_task:
                self._lerobot_dataset.save_episode_without_video_encoding()
            else:
                self._lerobot_dataset.save_episode_without_write_image()
        else:
            if self._lerobot_dataset.episode_buffer['size'] > 0:
                self._lerobot_dataset.save_episode()

    def create_frame(
            self,
            images: dict,
            state: list,
            action: list) -> dict:

        frame = {}
        for camera_name, image in images.items():
            frame[f'observation.images.{camera_name}'] = image
        frame['observation.state'] = np.array(state)
        frame['action'] = np.array(action)
        self.current_instruction = self._task_info.task_instruction[
            self._current_task % len(self._task_info.task_instruction)
        ]
        return frame

    def record_early_save(self):
        if self._lerobot_dataset.episode_buffer is not None:
            self._status = 'save'

    def record_stop(self):
        self._status = 'stop'

    def record_finish(self):
        self._status = 'finish'

    def re_record(self):
        self._stop_save_completed = False
        self._episode_reset()
        self._status = 'reset'

    def record_skip_task(self):
        self._stop_save_completed = False
        self._episode_reset()
        self._status = 'skip_task'
        self._get_current_scenario_number()
        self._current_task += 1

    def record_next_episode(self):
        self._status = 'save'

    def get_current_record_status(self):
        current_status = TaskStatus()
        current_status.robot_type = self._robot_type
        current_status.task_info = self._task_info

        if self._status == 'warmup':
            current_status.phase = TaskStatus.WARMING_UP
            current_status.total_time = int(self._task_info.warmup_time_s)
        elif self._status == 'run':
            current_status.phase = TaskStatus.RECORDING
            current_status.total_time = int(self._task_info.episode_time_s)
        elif self._status == 'reset':
            current_status.phase = TaskStatus.RESETTING
            current_status.total_time = int(self._task_info.reset_time_s)
        elif self._status == 'save' or self._status == 'finish':
            is_saving, encoding_progress = self._get_encoding_progress()
            if is_saving:
                current_status.phase = TaskStatus.SAVING
                current_status.total_time = int(0)
                self._proceed_time = int(0)
                current_status.encoding_progress = encoding_progress
        elif self._status == 'stop':
            is_saving, encoding_progress = self._get_encoding_progress()
            current_status.total_time = int(0)
            self._proceed_time = int(0)
            if is_saving:
                current_status.phase = TaskStatus.SAVING
                current_status.encoding_progress = encoding_progress
            else:
                current_status.phase = TaskStatus.STOPPED

        current_status.current_task_instruction = self.current_instruction
        current_status.proceed_time = int(getattr(self, '_proceed_time', 0))
        current_status.current_episode_number = int(self._record_episode_count)

        total_storage, used_storage = StorageChecker.get_storage_gb('/')
        current_status.used_storage_size = float(used_storage)
        current_status.total_storage_size = float(total_storage)

        current_status.used_cpu = float(self._cpu_checker.get_cpu_usage())

        ram_total, ram_used = RAMChecker.get_ram_gb()
        current_status.used_ram_size = float(ram_used)
        current_status.total_ram_size = float(ram_total)
        if not self._single_task:
            current_status.current_scenario_number = self._current_scenario_number

        return current_status

    def _get_current_scenario_number(self):
        task_count = len(self._task_info.task_instruction)
        if task_count == 0:
            return
        next_task_index = (self._current_task + 1) % task_count
        if next_task_index == 0:
            self._current_scenario_number += 1

    def _get_encoding_progress(self):
        min_encoding_percentage = 100
        is_saving = False
        if self._lerobot_dataset is not None:
            if hasattr(self._lerobot_dataset, 'encoders') and \
                    self._lerobot_dataset.encoders is not None:
                if self._lerobot_dataset.encoders:
                    is_saving = True
                    for key, values in self._lerobot_dataset.encoders.items():
                        min_encoding_percentage = min(
                            min_encoding_percentage,
                            values.get_encoding_status()['progress_percentage'])

        return is_saving, float(min_encoding_percentage)

    def convert_msgs_to_raw_datas(
            self,
            image_msgs,
            follower_msgs,
            total_joint_order,
            leader_msgs=None,
            leader_joint_order=None) -> tuple:

        camera_data = {}
        follower_data = []
        leader_data = []

        if image_msgs is not None:
            for key, value in image_msgs.items():
                camera_data[key] = cv2.cvtColor(
                    self.data_converter.compressed_image2cvmat(value),
                    cv2.COLOR_BGR2RGB)
        if follower_msgs is not None:
            for key, value in follower_msgs.items():
                if value is not None:
                    follower_data.extend(self.joint_msgs2tensor_array(
                        value, total_joint_order))
        if leader_msgs is not None:
            for key, value in leader_joint_order.items():
                # remove joint_order. from key
                prefix_key = key.replace('joint_order.', '')
                if prefix_key not in leader_msgs:
                    return camera_data, follower_data, None
                elif leader_msgs[prefix_key] is not None:
                    leader_data.extend(self.joint_msgs2tensor_array(
                        leader_msgs[prefix_key], value))
                else:
                    return camera_data, follower_data, None

        return camera_data, follower_data, leader_data

    def joint_msgs2tensor_array(self, msg_data, joint_order=None):
        if isinstance(msg_data, JointTrajectory):
            return self.data_converter.joint_trajectory2tensor_array(
                msg_data, joint_order)
        elif isinstance(msg_data, JointState):
            return self.data_converter.joint_state2tensor_array(
                msg_data, joint_order)
        elif isinstance(msg_data, Odometry):
            return self.data_converter.odometry2tensor_array(msg_data)
        elif isinstance(msg_data, Twist):
            return self.data_converter.twist2tensor_array(msg_data)
        else:
            raise ValueError(f'Unsupported message type: {type(msg_data)}')

    def _episode_reset(self):
        if (
            self._lerobot_dataset
            and hasattr(self._lerobot_dataset, 'episode_buffer')
            or self._current_task == 0
        ):
            if self._lerobot_dataset.episode_buffer is not None:
                for key, value in self._lerobot_dataset.episode_buffer.items():
                    if isinstance(value, list):
                        value.clear()
                    del value
                self._lerobot_dataset.episode_buffer.clear()
            self._lerobot_dataset.episode_buffer = None
        self._start_time_s = 0
        gc.collect()

    def _check_time(self, limit_time, next_status):
        self._proceed_time = time.perf_counter() - self._start_time_s
        if self._proceed_time >= limit_time:
            self._status = next_status
            self._start_time_s = 0
            self._proceed_time = 0
            return True
        else:
            return False

    def _check_dataset_exists(self, repo_id, root):
        # Local dataset check
        if os.path.exists(root):
            dataset_necessary_folders = ['meta', 'videos', 'data']
            invalid_foler = False
            for folder in dataset_necessary_folders:
                if not os.path.exists(os.path.join(root, folder)):
                    print(f'Dataset {repo_id} is incomplete, missing {folder} folder.')
                    invalid_foler = True
            if not invalid_foler:
                return True
            else:
                print(f'Dataset {repo_id} is incomplete, re-creating dataset.')
                shutil.rmtree(root)

        if self._task_info.push_to_hub:
            # Huggingface dataset check
            url = f'https://huggingface.co/api/datasets/{repo_id}'
            response = requests.get(url)
            url_exist_code = 200

            if response.status_code == url_exist_code:
                print(f'Dataset {repo_id} exists on Huggingface, downloading...')
                self._download_dataset(repo_id)
                return True

        return False

    def check_lerobot_dataset(self, images, joint_list):
        try:
            if self._lerobot_dataset is None:
                if self._check_dataset_exists(
                        self._save_repo_name,
                        self._save_path):
                    self._lerobot_dataset = LeRobotDatasetWrapper(
                        self._save_repo_name,
                        self._save_path
                    )
                else:
                    self._lerobot_dataset = self._create_dataset(
                        self._save_repo_name,
                        images, joint_list)

                if not self._task_info.use_optimized_save_mode:
                    self._lerobot_dataset.start_image_writer(
                            num_processes=1,
                            num_threads=1
                        )
            self._lerobot_dataset.set_robot_type(self._robot_type)

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
                'names': ['height', 'width', 'channels'],
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
        return LeRobotDatasetWrapper.create(
                repo_id=repo_id,
                fps=self._task_info.fps,
                features=features,
                use_videos=True
            )

    def _upload_dataset(self, tags, private=False):
        try:
            self._lerobot_dataset.push_to_hub(tags=tags, private=private)
        except Exception as e:
            print(f'Error uploading dataset: {e}')

    def _download_dataset(self, repo_id):
        snapshot_download(
            repo_id,
            repo_type='dataset',
            local_dir=self._save_path,
        )

    def convert_action_to_joint_trajectory_msg(self, action):
        joint_trajectory_msgs = self.data_converter.tensor_array2joint_trajectory(
            action,
            self.total_joint_order)
        return joint_trajectory_msgs

    def get_task_info(self):
        return self._task_info

    def _init_task_limits(self):
        if not self._single_task:
            self._task_info.num_episodes = 1_000_000
            self._task_info.episode_time_s = 1_000_000

    @staticmethod
    def get_huggingface_user_id():
        api = HfApi()
        try:
            user_info = api.whoami()
            user_ids = [user_info['name']]
            for org_info in user_info['orgs']:
                user_ids.append(org_info['name'])
            print(user_ids)
            return user_ids

        except Exception as e:
            print(f'Token validation failed: {e}')
            return None

    @staticmethod
    def register_huggingface_token(hf_token):
        api = HfApi(token=hf_token)
        try:
            user_info = api.whoami()
            user_name = user_info['name']
            print(f'Successfully validated HuggingFace token for user: {user_name}')

        except Exception as e:
            print(f'Token is invalid, please check hf token: {e}')
            return False

        try:
            result = subprocess.run([
                'huggingface-cli', 'login', '--token', hf_token
            ], capture_output=True, text=True, check=True)

            print('Successfully logged in to HuggingFace Hub')
            return result

        except subprocess.CalledProcessError as e:
            print(f'Failed to login with huggingface-cli: {e}')
            print(f'Error output: {e.stderr}')
            return False
        except FileNotFoundError:
            print('huggingface-cli not found. Please install package.')
            return False

    @staticmethod
    def download_huggingface_repo(
        repo_id,
        repo_type='dataset'
    ):
        download_path = {
            'dataset': Path.home() / '.cache/huggingface/lerobot',
            'model': Path.home() /'ros2_ws/src/physical_ai_tools/lerobot/outputs/train/'
        }

        save_path = download_path.get(repo_type)

        if save_path is None:
            raise ValueError(f'Invalid repo type: {repo_type}')

        save_dir = save_path / repo_id

        try:
            print(f"Starting download of {repo_id} ({repo_type})...")

            # Create a wrapper class that includes the progress_queue
            class ProgressTqdmWrapper(HuggingFaceProgressTqdm):
                def __init__(self, *args, **kwargs):
                    kwargs['progress_queue'] = DataManager._progress_queue
                    super().__init__(*args, **kwargs)

            result = snapshot_download(
                repo_id=repo_id,
                repo_type=repo_type,
                local_dir=save_dir,
                tqdm_class=ProgressTqdmWrapper
            )

            print(f"Download completed: {repo_id}")
            return result
        except Exception as e:
            print(f'Error downloading HuggingFace repo: {e}')
            # Print more detailed error information
            import traceback
            print(f'Detailed error traceback:\n{traceback.format_exc()}')
            return False

    @classmethod
    def set_progress_queue(cls, progress_queue):
        """Set progress queue for multiprocessing communication"""
        cls._progress_queue = progress_queue

    @staticmethod
    def upload_huggingface_repo(
        repo_id,
        repo_type,
        local_dir
    ):
        try:
            # Verify authentication first
            api = HfApi()

            try:
                user_info = api.whoami()
                print(f'Authenticated as: {user_info["name"]}')
            except Exception as auth_e:
                print(f'Authentication failed: {auth_e}')
                print('Please make sure you are authenticated with HuggingFace')
                return False
            # Create repository first
            print(f'Creating HuggingFace repository: {repo_id}')
            url = api.create_repo(
                repo_id,
                repo_type=repo_type,
                private=False,
                exist_ok=True,
            )
            print(f'Repository created/verified: {url}')

            # Upload folder contents
            print(f'Uploading folder {local_dir} to repository {repo_id}')
            upload_folder(
                repo_id=repo_id,
                folder_path=local_dir,
                repo_type=repo_type
            )
            print(f'Upload completed successfully for {repo_id}')

            # Create tag after successful upload
            try:
                print(f'Creating tag for {repo_id} ({repo_type})')
                api.create_tag(repo_id=repo_id, tag='v2.1', repo_type=repo_type)
                print(f'Tag "v2.1" created successfully for {repo_id}')
            except Exception as e:
                print(f'Warning: Failed to create tag for {repo_id} ({repo_type}): {e}')
                # Don't fail the entire upload just because tag creation failed

            return True
        except Exception as e:
            print(f'Error Uploading HuggingFace repo: {e}')
            # Print more detailed error information
            import traceback
            print(f'Detailed error traceback:\n{traceback.format_exc()}')
            return False

    @staticmethod
    def delete_huggingface_repo(
        repo_id,
        repo_type='dataset',
    ):
        try:
            result = HfApi().delete_repo(repo_id, repo_type=repo_type)
            return result
        except Exception as e:
            print(f'Error deleting HuggingFace repo: {e}')
            return False

    @staticmethod
    def get_huggingface_repo_list(
        author,
        data_type='dataset'
    ):
        repo_id_list = []
        if data_type == 'dataset':
            dataset_list = HfApi().list_datasets(author=author)
            for dataset in dataset_list:
                repo_id_list.append(dataset.id)

        elif data_type == 'model':
            model_list = HfApi().list_models(author=author)
            for model in model_list:
                repo_id_list.append(model.id)
        reverse = repo_id_list[::-1]
        return reverse

    @staticmethod
    def get_collections_repo_list(
        collection_id
    ):
        repo_id_list = []
        collection_list = HfApi().get_collection(collection_id)
        repo_list_in_collection = []
        for item in collection_list.items:
            repo_list_in_collection.append(item.item_id)
        return repo_list_in_collection

# repo_list = DataManager.get_collections_repo_list('ROBOTIS/pickcoffee-env1-task3-68b4dbb34096a56b85b8b613')
# for repo_id in repo_list:
#     DataManager.download_huggingface_repo(repo_id, repo_type='dataset')
