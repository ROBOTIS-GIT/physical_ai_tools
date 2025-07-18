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
# Author: Seongwoo Kim

from pathlib import Path
import threading

import draccus
from lerobot.configs.train import TrainPipelineConfig
from physical_ai_interfaces.msg import TrainingInfo, TrainingStatus
from physical_ai_server.training.trainers.lerobot.lerobot_trainer import LerobotTrainer
# TODO: Uncomment when training metrics is implemented
# from physical_ai_server.training.trainers.gr00tn1.gr00tn1_trainer import Gr00tN1Trainer
# from physical_ai_server.training.trainers.openvla.openvla_trainer import OpenVLATrainer


class TrainingManager:

    DEFAULT_TRAINING_DIR = 'src/physical_ai_tools/lerobot/outputs/train/'
    TRAINER_MAPPING = {
        'pi0fast': LerobotTrainer,
        'pi0': LerobotTrainer,
        'diffusion': LerobotTrainer,
        'act': LerobotTrainer,
        'tdmpc': LerobotTrainer,
        'vqbet': LerobotTrainer,
        # TODO: Uncomment when Gr00t and OpenVLA are implemented
        # 'gr00tn1': Gr00tN1Trainer,
        # 'openvla': OpenVLATrainer,
    }

    def __init__(self):
        self.training_info = TrainingInfo()
        self.trainer = None
        self.cfg = None
        self.stop_event = threading.Event()
        self.parser = None

    def _get_training_config(self):
        if isinstance(self.trainer, LerobotTrainer):
            args = [
                f'--policy.type={self.training_info.policy_type}',
                f'--policy.device={self.training_info.policy_device}',
                f'--dataset.repo_id={self.training_info.dataset}',
                f'--output_dir={
                    self.DEFAULT_TRAINING_DIR
                    + self.training_info.output_folder_name
                }',
                f'--seed={self.training_info.seed or 1000}',
                f'--num_workers={self.training_info.num_workers or 4}',
                f'--batch_size={self.training_info.batch_size or 8}',
                f'--steps={self.training_info.steps or 100000}',
                f'--eval_freq={self.training_info.eval_freq or 20000}',
                f'--log_freq={self.training_info.log_freq or 200}',
                f'--save_freq={self.training_info.save_freq or 1000}',
            ]
            self.cfg = draccus.parse(TrainPipelineConfig, None, args=args)

    def _get_trainer(self):
        policy_type = self.training_info.policy_type.lower()
        trainer_class = self.TRAINER_MAPPING.get(policy_type)
        if not trainer_class:
            raise ValueError(
                f"Unknown policy type: '{policy_type}'."
            )
        self.trainer = trainer_class()

    # TODO: Uncomment when training metrics is implemented
    # def get_training_metrics(self):
    #     metrics = self.trainer.send_training_metrics()
    #     return metrics

    def get_available_list() -> tuple[list[str], list[str]]:
        policy_list = [
            'tdmpc',
            'diffusion',
            'act',
            'vqbet',
            'pi0',
            'pi0fast',
        ]

        device_list = [
            'cuda',
            'cpu',
        ]
        return policy_list, device_list

    def get_weight_save_root_path():
        weight_save_root_path = (
            Path(__file__).resolve().parent.parent.parent / 'lerobot' / 'outputs' / 'train'
        )
        return weight_save_root_path

    def get_current_training_status(self):
        current_training_status = TrainingStatus()
        current_training_status.training_info = self.training_info
        if self.trainer:
            current_training_status.current_step = self.trainer.get_current_step()
        else:
            current_training_status.current_step = 0
        return current_training_status

    def train(self):
        self._get_trainer()
        self._get_training_config()
        self.trainer.train(self.cfg, stop_event=self.stop_event)
