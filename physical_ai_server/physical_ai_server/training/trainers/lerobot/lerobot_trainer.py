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

from contextlib import nullcontext
from pprint import pformat
import time
from typing import Any

from lerobot.configs.train import TrainPipelineConfig
from lerobot.datasets.factory import make_dataset
from lerobot.datasets.sampler import EpisodeAwareSampler
from lerobot.datasets.utils import cycle
from lerobot.envs.factory import make_env
from lerobot.optim.factory import make_optimizer_and_scheduler
from lerobot.policies.factory import make_policy
from lerobot.policies.pretrained import PreTrainedPolicy
from lerobot.policies.utils import get_device_from_parameters
from lerobot.scripts.eval import eval_policy
from lerobot.utils.logging_utils import AverageMeter, MetricsTracker
from lerobot.utils.random_utils import set_seed
from lerobot.utils.train_utils import (
    get_step_checkpoint_dir,
    get_step_identifier,
    load_training_state,
    save_checkpoint,
    update_last_checkpoint,
)
from lerobot.utils.utils import (
    format_big_number,
    get_safe_torch_device,
    has_method,
)
from lerobot.utils.wandb_utils import WandBLogger
from physical_ai_server.training.trainers.trainer import Trainer
from rclpy.logging import get_logger
from termcolor import colored
import torch
from torch.amp import GradScaler
from torch.optim import Optimizer


class LerobotTrainer(Trainer):
    """
    LeRobot-based trainer implementation for imitation learning policies.

    Supports various policy types (ACT, Diffusion, VQ-BeT, etc.) with resume functionality.
    """

    def __init__(self):
        """Initialize trainer with logging and training state tracking."""
        super().__init__()
        self.logger = get_logger('LerobotTrainer')
        self.current_step = 0
        self.current_loss = float('nan')

    def train(self, cfg: TrainPipelineConfig, stop_event=None):
        """
        Execute training pipeline with support for resume functionality.

        Parameters
        ----------
        cfg : TrainPipelineConfig
            Training pipeline configuration
        stop_event : threading.Event, optional
            Threading event to stop training gracefully

        """
        # Setup working directory for LeRobot relative paths
        self._setup_working_directory()

        # Handle config path conversion and resume setup
        self._prepare_config_for_training(cfg)

        # Validate configuration (skip for resume mode)
        self._validate_config(cfg)

        # Log training configuration
        self.logger.info('Training Configuration:')
        self.logger.info(pformat(cfg.to_dict()))

        # Initialize training components
        wandb_logger = self._setup_logging(cfg)
        device = self._setup_device_and_optimization(cfg)
        dataset = self._create_dataset(cfg)
        eval_env = self._create_evaluation_environment(cfg)
        policy = self._create_policy(cfg, dataset)
        optimizer, lr_scheduler, grad_scaler = self._create_optimization_components(
            cfg,
            policy,
            device
        )

        # Handle training state resume
        step = self._load_training_state_if_resume(cfg, optimizer, lr_scheduler)

        # Log training information
        self._log_training_info(cfg, dataset, policy, step)

        # Execute training loop
        self._run_training_loop(
            cfg,
            dataset,
            policy,
            optimizer,
            lr_scheduler,
            grad_scaler,
            device,
            wandb_logger,
            eval_env,
            step,
            stop_event
        )

        # Cleanup
        if eval_env:
            eval_env.close()
        self.logger.info('Training completed successfully')

    def _setup_working_directory(self):
        """Set up working directory for LeRobot relative path resolution."""
        import os
        lerobot_dir = '/root/ros2_ws/src/physical_ai_tools/lerobot'
        if os.path.exists(lerobot_dir):
            os.chdir(lerobot_dir)

    def _prepare_config_for_training(self, cfg):
        """Prepare configuration for training, handling path conversions."""
        from pathlib import Path

        # Convert absolute config_path to relative if needed for LeRobot compatibility
        if hasattr(cfg, 'config_path') and cfg.config_path:
            config_path = Path(cfg.config_path)
            if config_path.is_absolute() and 'outputs' in config_path.parts:
                outputs_index = config_path.parts.index('outputs')
                relative_parts = config_path.parts[outputs_index:]
                cfg.config_path = str(Path(*relative_parts))

    def _validate_config(self, cfg):
        """Validate configuration, with special handling for resume mode."""
        if getattr(cfg, 'resume', False) and hasattr(cfg, 'config_path') and cfg.config_path:
            # Resume mode: Setup required paths and skip validation
            from pathlib import Path
            config_path = Path(cfg.config_path)

            if config_path.exists():
                # Setup policy and checkpoint paths for resume
                policy_path = config_path.parent
                cfg.policy.pretrained_path = policy_path
                cfg.checkpoint_path = policy_path.parent
                self.logger.info('Resume mode: Skipping validation, using existing configuration')
            else:
                self.logger.error(f'Resume config file not found: {config_path}')
                cfg.validate()
        else:
            # New training mode: Use standard validation
            cfg.validate()

    def _setup_logging(self, cfg):
        """Set up logging configuration (WandB or local)."""
        if cfg.wandb.enable and cfg.wandb.project:
            wandb_logger = WandBLogger(cfg)
        else:
            wandb_logger = None
            self.logger.info(colored('Logs will be saved locally.', 'yellow', attrs=['bold']))
        return wandb_logger

    def _setup_device_and_optimization(self, cfg):
        """Set up compute device and optimization settings."""
        if cfg.seed is not None:
            set_seed(cfg.seed)

        device = get_safe_torch_device(cfg.policy.device, log=True)
        torch.backends.cudnn.benchmark = True
        torch.backends.cuda.matmul.allow_tf32 = True
        return device

    def _create_dataset(self, cfg):
        """Create training dataset."""
        self.logger.info('Creating dataset')
        return make_dataset(cfg)

    def _create_evaluation_environment(self, cfg):
        """Create evaluation environment if needed."""
        eval_env = None
        if cfg.eval_freq > 0 and cfg.env is not None:
            self.logger.info('Creating evaluation environment')
            eval_env = make_env(
                cfg.env,
                n_envs=cfg.eval.batch_size,
                use_async_envs=cfg.eval.use_async_envs
            )
        return eval_env

    def _create_policy(self, cfg, dataset):
        """Create policy model."""
        self.logger.info('Creating policy')
        return make_policy(cfg=cfg.policy, ds_meta=dataset.meta)

    def _create_optimization_components(self, cfg, policy, device):
        """Create optimizer, scheduler, and gradient scaler."""
        self.logger.info('Creating optimizer and scheduler')
        optimizer, lr_scheduler = make_optimizer_and_scheduler(cfg, policy)
        grad_scaler = GradScaler(device.type, enabled=cfg.policy.use_amp)
        return optimizer, lr_scheduler, grad_scaler

    def _load_training_state_if_resume(self, cfg, optimizer, lr_scheduler):
        """Load training state for resume mode."""
        step = 0  # Default starting step

        if cfg.resume:
            self.logger.info('Loading training state for resume')
            step, optimizer, lr_scheduler = load_training_state(
                cfg.checkpoint_path,
                optimizer,
                lr_scheduler
            )
            self.logger.info(f'Resumed from step {step}')

        return step

    def _log_training_info(self, cfg, dataset, policy, step):
        """Log training information and statistics."""
        num_learnable_params = sum(p.numel() for p in policy.parameters() if p.requires_grad)
        num_total_params = sum(p.numel() for p in policy.parameters())

        self.logger.info(colored('Output dir:', 'yellow', attrs=['bold']) + f' {cfg.output_dir}')
        if cfg.env is not None:
            self.logger.info(f'{cfg.env.task=}')
        self.logger.info(
            f'Training steps: {cfg.steps} ({format_big_number(cfg.steps)})'
        )
        self.logger.info(
            f'Dataset frames: {dataset.num_frames} ({format_big_number(dataset.num_frames)})'
        )
        self.logger.info(f'Dataset episodes: {dataset.num_episodes}')
        self.logger.info(
            f'Learnable parameters: {num_learnable_params} '
            f'({format_big_number(num_learnable_params)})'
        )
        self.logger.info(
            f'Total parameters: {num_total_params} '
            f'({format_big_number(num_total_params)})'
        )
        if step > 0:
            self.logger.info(f'Starting from step: {step} (resumed)')

    def _run_training_loop(
        self,
        cfg,
        dataset,
        policy,
        optimizer,
        lr_scheduler,
        grad_scaler,
        device,
        wandb_logger,
        eval_env,
        step,
        stop_event,
    ):
        """Execute the main training loop."""
        # Create dataloader for offline training
        if hasattr(cfg.policy, 'drop_n_last_frames'):
            shuffle = False
            sampler = EpisodeAwareSampler(
                dataset.episode_data_index,
                drop_n_last_frames=cfg.policy.drop_n_last_frames,
                shuffle=True,
            )
        else:
            shuffle = True
            sampler = None

        dataloader = torch.utils.data.DataLoader(
            dataset,
            num_workers=cfg.num_workers,
            batch_size=cfg.batch_size,
            shuffle=shuffle,
            sampler=sampler,
            pin_memory=device.type != 'cpu',
            drop_last=False,
        )
        dl_iter = cycle(dataloader)

        # Initialize training metrics
        train_metrics = {
            'loss': AverageMeter('loss', ':.3f'),
            'grad_norm': AverageMeter('grdn', ':.3f'),
            'lr': AverageMeter('lr', ':0.1e'),
            'update_s': AverageMeter('updt_s', ':.3f'),
            'dataloading_s': AverageMeter('data_s', ':.3f'),
        }

        train_tracker = MetricsTracker(
            cfg.batch_size,
            dataset.num_frames,
            dataset.num_episodes,
            train_metrics,
            initial_step=step
        )

        policy.train()
        self.logger.info('Starting offline training on fixed dataset')

        # Main training loop
        for _ in range(step, cfg.steps):
            if stop_event and stop_event.is_set():
                self.logger.info('Training stopped by stop event')
                break

            self.current_step = step + 1

            # Load batch and move to device
            start_time = time.perf_counter()
            batch = next(dl_iter)
            train_tracker.dataloading_s = time.perf_counter() - start_time

            for key in batch:
                if isinstance(batch[key], torch.Tensor):
                    batch[key] = batch[key].to(device, non_blocking=True)

            # Update policy
            train_tracker, output_dict = self._update_policy(
                train_tracker, policy, batch, optimizer,
                cfg.optimizer.grad_clip_norm, grad_scaler=grad_scaler,
                lr_scheduler=lr_scheduler, use_amp=cfg.policy.use_amp,
            )

            step += 1
            train_tracker.step()

            # Determine logging/saving/eval steps
            is_log_step = cfg.log_freq > 0 and step % cfg.log_freq == 0
            is_saving_step = step % cfg.save_freq == 0 or step == cfg.steps
            is_eval_step = cfg.eval_freq > 0 and step % cfg.eval_freq == 0

            # Logging
            if is_log_step:
                self.logger.info(str(train_tracker))
                if wandb_logger:
                    wandb_log_dict = train_tracker.to_dict()
                    if output_dict:
                        wandb_log_dict.update(output_dict)
                    wandb_logger.log_dict(wandb_log_dict, step)
                train_tracker.reset_averages()

            # Checkpointing
            if cfg.save_checkpoint and is_saving_step:
                self.logger.info(f'Saving checkpoint at step {step}')
                checkpoint_dir = get_step_checkpoint_dir(cfg.output_dir, cfg.steps, step)
                save_checkpoint(checkpoint_dir, step, cfg, policy, optimizer, lr_scheduler)
                update_last_checkpoint(checkpoint_dir)
                if wandb_logger:
                    wandb_logger.log_policy(checkpoint_dir)

            # Evaluation
            if cfg.env and is_eval_step:
                self._run_evaluation(cfg, eval_env, policy, device, step, wandb_logger, dataset)

    def _run_evaluation(self, cfg, eval_env, policy, device, step, wandb_logger, dataset):
        """Run policy evaluation."""
        step_id = get_step_identifier(step, cfg.steps)
        self.logger.info(f'Evaluating policy at step {step}')

        with (
            torch.no_grad(),
            torch.autocast(device_type=device.type) if cfg.policy.use_amp else nullcontext(),
        ):
            eval_info = eval_policy(
                eval_env, policy, cfg.eval.n_episodes,
                videos_dir=cfg.output_dir / 'eval' / f'videos_step_{step_id}',
                max_episodes_rendered=4, start_seed=cfg.seed,
            )

        # Log evaluation metrics
        eval_metrics = {
            'avg_sum_reward': AverageMeter('∑rwrd', ':.3f'),
            'pc_success': AverageMeter('success', ':.1f'),
            'eval_s': AverageMeter('eval_s', ':.3f'),
        }

        eval_tracker = MetricsTracker(
            cfg.batch_size, dataset.num_frames, dataset.num_episodes,
            eval_metrics, initial_step=step
        )

        eval_tracker.eval_s = eval_info['aggregated'].pop('eval_s')
        eval_tracker.avg_sum_reward = eval_info['aggregated'].pop('avg_sum_reward')
        eval_tracker.pc_success = eval_info['aggregated'].pop('pc_success')

        self.logger.info(str(eval_tracker))

        if wandb_logger:
            wandb_log_dict = {**eval_tracker.to_dict(), **eval_info}
            wandb_logger.log_dict(wandb_log_dict, step, mode='eval')
            wandb_logger.log_video(eval_info['video_paths'][0], step, mode='eval')

    def _update_policy(
        self,
        train_metrics: MetricsTracker,
        policy: PreTrainedPolicy,
        batch: Any,
        optimizer: Optimizer,
        grad_clip_norm: float,
        grad_scaler: GradScaler,
        lr_scheduler=None,
        use_amp: bool = False,
        lock=None,
    ) -> tuple[MetricsTracker, dict]:
        """
        Update policy with a single training step.

        Parameters
        ----------
        train_metrics : MetricsTracker
            Training metrics tracker
        policy : PreTrainedPolicy
            Policy model to update
        batch : Any
            Training batch
        optimizer : Optimizer
            Optimizer for parameter updates
        grad_clip_norm : float
            Gradient clipping norm
        grad_scaler : GradScaler
            Gradient scaler for mixed precision
        lr_scheduler : optional
            Learning rate scheduler (optional)
        use_amp : bool, default=False
            Whether to use automatic mixed precision
        lock : optional
            Threading lock (optional)

        Returns
        -------
        tuple
            Updated metrics tracker and output dictionary

        """
        start_time = time.perf_counter()
        device = get_device_from_parameters(policy)
        policy.train()

        # Forward pass with optional mixed precision
        with torch.autocast(device_type=device.type) if use_amp else nullcontext():
            loss, output_dict = policy.forward(batch)

        # Backward pass
        grad_scaler.scale(loss).backward()
        grad_scaler.unscale_(optimizer)

        # Gradient clipping
        grad_norm = torch.nn.utils.clip_grad_norm_(
            policy.parameters(),
            grad_clip_norm,
            error_if_nonfinite=False,
        )

        # Optimizer step with optional lock for thread safety
        with lock if lock is not None else nullcontext():
            grad_scaler.step(optimizer)
        grad_scaler.update()
        optimizer.zero_grad()

        # Learning rate scheduler step
        if lr_scheduler is not None:
            lr_scheduler.step()

        # Policy-specific updates
        if has_method(policy, 'update'):
            policy.update()

        # Update metrics and internal state
        train_metrics.loss = loss.item()
        train_metrics.grad_norm = grad_norm.item()
        train_metrics.lr = optimizer.param_groups[0]['lr']
        train_metrics.update_s = time.perf_counter() - start_time
        self.current_loss = loss.item()

        return train_metrics, output_dict

    def get_current_step(self):
        """Get current training step."""
        return self.current_step

    def get_current_loss(self):
        """Get current training loss."""
        return self.current_loss
