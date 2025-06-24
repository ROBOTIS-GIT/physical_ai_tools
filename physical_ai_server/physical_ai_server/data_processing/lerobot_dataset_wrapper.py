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

import threading

from lerobot.common.datasets.compute_stats import (
    get_feature_stats
)
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.datasets.utils import (
    validate_episode_buffer,
    validate_frame,
    write_episode,
    write_episode_stats,
    write_info
)
import numpy as np
from physical_ai_server.video_encoder.ffmpeg_encoder import FFmpegEncoder


class LeRobotDatasetWrapper(LeRobotDataset):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.encoders = {}

    def add_frame_without_write_image(self, frame: dict) -> None:
        validate_frame(frame, self.features)

        if self.episode_buffer is None:
            self.episode_buffer = self.create_episode_buffer()

        # if 'task' in self.episode_buffer and self.episode_buffer['task']:
        #     last_task = self.episode_buffer['task'][-1]
        #     if frame.get('task') != last_task:
        #         print(f"[DEBUG][MISMATCH] Task changed from {last_task} to {frame.get('task')} — possible skip/reset not cleared properly")
        if 'task' in self.episode_buffer and self.episode_buffer['task']:
            last_task = self.episode_buffer['task'][-1]
            if frame.get('task') != last_task:
                print(f"[DEBUG][MISMATCH] Task changed from {last_task} to {frame.get('task')} — forcing buffer reset")
                self.episode_buffer = self.create_episode_buffer()

        # Automatically add frame_index and timestamp to episode buffer
        frame_index = self.episode_buffer['size']
        timestamp = frame.pop('timestamp') if 'timestamp' in frame else frame_index / self.fps
        self.episode_buffer['frame_index'].append(frame_index)
        self.episode_buffer['timestamp'].append(timestamp)

        # Add frame features to episode_buffer
        for key in frame:
            if key not in self.episode_buffer:
                self.episode_buffer[key] = [frame[key]]
            else:
                self.episode_buffer[key].append(frame[key])

        self.episode_buffer['size'] += 1

    def save_episode_without_write_image(self):
        episode_buffer = self.episode_buffer
        validate_episode_buffer(
            episode_buffer,
            self.meta.total_episodes,
            self.features)

        episode_length = episode_buffer.pop('size')
        tasks = episode_buffer.pop('task')
        episode_tasks = list(set(tasks))
        episode_index = episode_buffer['episode_index']

        episode_buffer['index'] = np.arange(
            self.meta.total_frames,
            self.meta.total_frames + episode_length)
        episode_buffer['episode_index'] = np.full((episode_length,), episode_index)

        # Add new tasks to the tasks dictionary
        for task in episode_tasks:
            task_index = self.meta.get_task_index(task)
            if task_index is None:
                self.meta.add_task(task)

        # Given tasks in natural language, find their corresponding task indices
        episode_buffer['task_index'] = np.array([self.meta.get_task_index(task) for task in tasks])
        for key, ft in self.features.items():
            if (key in ['index', 'episode_index', 'task_index'] or
                    ft['dtype'] in ['image', 'video']):
                continue
            episode_buffer[key] = np.stack(episode_buffer[key])

        self._save_episode_table(episode_buffer, episode_index)
        ep_stats = self.compute_episode_stats_buffer(episode_buffer, self.features)

        video_paths = {}
        video_count = 0
        for key, ep in self.episode_buffer.items():
            if 'observation.images' in key:
                video_path = self.root / self.meta.get_video_file_path(episode_index, key)
                video_paths[key] = str(video_path)
                self._create_video(ep, video_path)
                video_count += 1
                video_info = {
                    'video.height': self.features[key]['shape'][0],
                    'video.width': self.features[key]['shape'][1],
                    'video.channels': self.features[key]['shape'][2],
                    'video.codec': 'libx264',
                    'video.pix_fmt': 'yuv420p',
                }
                self.meta.info['features'][key]['info'] = video_info

        self.save_meta_info(
            video_count,
            episode_index,
            episode_length,
            episode_tasks,
            ep_stats
        )

    def save_meta_info(
            self,
            video_count,
            episode_index,
            episode_length,
            episode_tasks,
            episode_stats):
        chunk = self.meta.get_episode_chunk(episode_index)
        if chunk >= self.meta.total_chunks:
            self.meta.info['total_chunks'] += 1
        self.meta.info['total_episodes'] += 1
        self.meta.info['total_frames'] += episode_length
        self.meta.info['total_videos'] += video_count
        self.meta.info['splits'] = {'train': f"0:{self.meta.info['total_episodes']}"}
        self.meta.info['robot_type'] = 'aiworker'

        episode_dict = {
            'episode_index': episode_index,
            'tasks': episode_tasks,
            'length': episode_length,
        }

        write_info(self.meta.info, self.meta.root)
        write_episode(episode_dict, self.meta.root)
        write_episode_stats(episode_index, episode_stats, self.meta.root)

    def _create_video(
            self,
            image_buffer: list[np.ndarray],
            save_path: str):
        if not hasattr(self, 'encoders') or self.encoders is None:
            self.encoders = {}

        self.encoders[save_path] = FFmpegEncoder(
                fps=self.fps,
                chunk_size=50,
                preset='ultrafast',
                crf=28,
                pix_fmt='yuv420p',
                vcodec='libx264'
            )
        self.encoders[save_path].set_buffer(image_buffer)
        encoding_thread = threading.Thread(
            target=self.encoders[save_path].encode_video,
            args=(save_path,)
        )
        encoding_thread.start()

    def check_video_encoding_completed(self) -> bool:
        if not hasattr(self, 'encoders') or self.encoders is None:
            self.encoders = {}
            return True

        if self.encoders:
            all_completed = True
            completed_encoders = []

            for key, encoder in self.encoders.items():
                if not encoder.encoding_completed:
                    all_completed = False
                else:
                    completed_encoders.append(key)

            for key in completed_encoders:
                encoder = self.encoders[key]
                encoder.clear_buffer()
                del self.encoders[key]
                del encoder

            if all_completed:
                self.encoders = {}
                return True
            else:
                return False

        return True

    def compute_episode_stats_buffer(self, episode_buffer, features):
        ep_stats = {}
        for key, data in episode_buffer.items():
            if features[key]['dtype'] == 'string':
                continue
            elif features[key]['dtype'] in ['image', 'video']:
                ep_ft_array = self._sample_images(data)
                axes_to_reduce = (0, 2, 3)
                keepdims = True
            else:
                ep_ft_array = data
                axes_to_reduce = 0
                keepdims = ep_ft_array.ndim == 1
            ep_stats[key] = get_feature_stats(
                ep_ft_array,
                axis=axes_to_reduce,
                keepdims=keepdims)
            if features[key]['dtype'] in ['image', 'video']:
                ep_stats[key] = {
                    k: v if k == 'count' else np.squeeze(
                        v / 255.0, axis=0) for k, v in ep_stats[key].items()
                }
        return ep_stats

    def _estimate_num_samples(
        self,
        dataset_len: int,
        min_num_samples: int = 100,
        max_num_samples: int = 10_000,
        power: float = 0.75
    ) -> int:
        if dataset_len < min_num_samples:
            min_num_samples = dataset_len
        return max(min_num_samples, min(int(dataset_len**power), max_num_samples))

    def _sample_indices(self, data_len: int) -> list[int]:
        num_samples = self._estimate_num_samples(data_len)
        return np.round(np.linspace(0, data_len - 1, num_samples)).astype(int).tolist()

    def _auto_downsample_height_width(
            self,
            img: np.ndarray,
            target_size: int = 150,
            max_size_threshold: int = 300):
        _, h, w = img.shape

        if max(w, h) < max_size_threshold:
            return img

        downsample_factor = int(w / target_size) if w > h else int(h / target_size)
        return img[:, ::downsample_factor, ::downsample_factor]

    def _sample_images(self, image_array) -> np.ndarray:
        sampled_indices = self._sample_indices(len(image_array))
        images = None
        for i, idx in enumerate(sampled_indices):
            img = image_array[idx]
            img = np.transpose(img, (2, 0, 1))
            img = self._auto_downsample_height_width(img)
            if images is None:
                images = np.empty((len(sampled_indices), *img.shape), dtype=np.uint8)
            images[i] = img

        return images
