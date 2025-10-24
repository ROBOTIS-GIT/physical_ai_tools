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
# Author: Seongwoo Kim, Woojin Wie

import json
from pathlib import Path
import threading

import draccus
import lerobot
from lerobot.configs.train import TrainPipelineConfig
from physical_ai_interfaces.msg import TrainingInfo, TrainingStatus
from physical_ai_server.training.trainers.lerobot.lerobot_trainer import LerobotTrainer
# TODO: Uncomment when training metrics is implemented
# from physical_ai_server.training.trainers.gr00tn1.gr00tn1_trainer import Gr00tN1Trainer
# from physical_ai_server.training.trainers.openvla.openvla_trainer import OpenVLATrainer


class TrainingManager:
    """
    Manages training pipeline configuration and execution for various AI models.

    Supports both new training and resume functionality through configuration management.
    """

    # Path constants
    DEFAULT_TRAINING_DIR = 'src/physical_ai_tools/lerobot/outputs/train/'

    # Supported trainer mapping
    TRAINER_MAPPING = {
        'pi0fast': LerobotTrainer,
        'pi0': LerobotTrainer,
        'diffusion': LerobotTrainer,
        'act': LerobotTrainer,
        'tdmpc': LerobotTrainer,
        'vqbet': LerobotTrainer,
        'smolvla': LerobotTrainer
        # TODO: Add support for additional trainers when implemented
        # 'gr00tn1': Gr00tN1Trainer,
        # 'openvla': OpenVLATrainer,
    }

    def __init__(self):
        """Initialize training manager with default configuration."""
        self.training_info = TrainingInfo()
        self.trainer = None
        self.cfg = None
        self.stop_event = threading.Event()

        # Resume-specific configuration
        self.resume = False
        self.resume_model_path = None

    def _update_config_with_training_info(self, config_path):
        """
        Update saved train_config.json with current training_info values.

        Only updates non-zero/non-empty values to preserve existing configuration.

        Parameters
        ----------
        config_path : Path
            Path to the train_config.json file

        Returns
        -------
        bool
            True if update successful, False otherwise

        """
        try:
            # Read existing configuration
            with open(config_path, 'r', encoding='utf-8') as f:
                config_data = json.load(f)

            # Update configuration with provided values (skip zero/empty values)
            if self.training_info.dataset.strip():
                # Update dataset repo_id if provided
                if 'dataset' not in config_data:
                    config_data['dataset'] = {}
                config_data['dataset']['repo_id'] = self.training_info.dataset
            if self.training_info.seed != 0:
                config_data['seed'] = self.training_info.seed
            if self.training_info.num_workers != 0:
                config_data['num_workers'] = self.training_info.num_workers
            if self.training_info.batch_size != 0:
                config_data['batch_size'] = self.training_info.batch_size
            if self.training_info.steps != 0:
                config_data['steps'] = self.training_info.steps
            if self.training_info.eval_freq != 0:
                config_data['eval_freq'] = self.training_info.eval_freq
            if self.training_info.log_freq != 0:
                config_data['log_freq'] = self.training_info.log_freq
            if self.training_info.save_freq != 0:
                config_data['save_freq'] = self.training_info.save_freq

            # Write back updated configuration
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=2)

            return True
        except Exception as e:
            print(f'Error updating config file: {e}')
            return False

    def _get_training_config(self):
        """
        Configure training pipeline based on resume mode or new training setup.

        For resume mode: Loads existing config and applies user overrides
        For new training: Creates config from scratch with user parameters
        """
        if self.resume and self.resume_model_path:
            # Resume mode: Load existing configuration and apply overrides
            weight_save_root_path = TrainingManager.get_weight_save_root_path()
            full_config_path = weight_save_root_path / self.resume_model_path / 'train_config.json'

            # Update existing config with current training_info values
            if not self._update_config_with_training_info(full_config_path):
                raise RuntimeError(f'Failed to update config file: {full_config_path}')

            # Create minimal args for resume (config file will override most settings)
            args = [
                f'--config_path={full_config_path}',
                '--resume=true'
            ]

            self.cfg = draccus.parse(TrainPipelineConfig, None, args=args)

            # Ensure config_path is properly set for resume operations
            if not hasattr(self.cfg, 'config_path') or not self.cfg.config_path:
                self.cfg.config_path = str(full_config_path)
        else:
            # New training mode: Build configuration from training_info
            args = [
                f'--policy.type={self.training_info.policy_type}',
                f'--policy.device={self.training_info.policy_device}',
                f'--dataset.repo_id={self.training_info.dataset}',
                f"--output_dir={
                    str(TrainingManager.get_weight_save_root_path()) + '/'
                    + self.training_info.output_folder_name
                }",
                f'--seed={self.training_info.seed or 1000}',
                f'--num_workers={self.training_info.num_workers or 4}',
                f'--batch_size={self.training_info.batch_size or 8}',
                f'--steps={self.training_info.steps or 100000}',
                f'--eval_freq={self.training_info.eval_freq or 20000}',
                f'--log_freq={self.training_info.log_freq or 200}',
                f'--save_freq={self.training_info.save_freq or 1000}',
                f'--policy.push_to_hub={False}'
            ]

            self.cfg = draccus.parse(TrainPipelineConfig, None, args=args)

    def _get_trainer(self):
        """
        Initialize appropriate trainer based on policy type.

        Raises
        ------
        ValueError
            If policy type is not supported

        """
        policy_type = self.training_info.policy_type.lower()
        trainer_class = self.TRAINER_MAPPING.get(policy_type)
        if not trainer_class:
            raise ValueError(
                f'Supported types: {list(self.TRAINER_MAPPING.keys())}'
            )
        self.trainer = trainer_class()

    @staticmethod
    def get_available_list() -> tuple[list[str], list[str]]:
        """
        Get lists of available policy types and devices.

        Returns
        -------
        tuple
            (policy_list, device_list)

        """
        policy_list = [
            'tdmpc',
            'diffusion',
            'act',
            'vqbet',
            'pi0',
            'pi0fast',
            'smolvla',
        ]

        device_list = [
            'cuda',
            'cpu',
        ]
        return policy_list, device_list

    @staticmethod
    def get_weight_save_root_path():
        """
        Get the root path for saving training weights and checkpoints.

        Returns
        -------
        Path
            Absolute path to the training outputs directory

        """
        # Extract the base lerobot directory from lerobot.__file__
        lerobot_file_path = Path(lerobot.__file__).resolve()

        # Find the outermost 'lerobot' directory in the path
        lerobot_dirs = [parent for parent in lerobot_file_path.parents if parent.name == 'lerobot']
        if lerobot_dirs:
            current_path = lerobot_dirs[-1]  # outermost 'lerobot' directory
        else:
            # Fallback: use the parent of the file
            current_path = lerobot_file_path.parent.parent  # up to 'lerobot'

        weight_save_root_path = current_path / 'outputs' / 'train'
        return weight_save_root_path.resolve()

    def get_current_training_status(self):
        """
        Get current training status including step and loss information.

        Returns
        -------
        TrainingStatus
            Current training status message

        """
        current_training_status = TrainingStatus()
        current_training_status.training_info = self.training_info

        if self.trainer:
            current_training_status.current_step = self.trainer.get_current_step()
            current_training_status.current_loss = self.trainer.get_current_loss()
        else:
            current_training_status.current_step = 0
            current_training_status.current_loss = float('nan')

        return current_training_status

    def train(self):
        """
        Execute the training pipeline.

        Sets up configuration, initializes trainer, and starts training process.
        """
        self._get_training_config()
        self._get_trainer()
        self.trainer.train(self.cfg, stop_event=self.stop_event)
