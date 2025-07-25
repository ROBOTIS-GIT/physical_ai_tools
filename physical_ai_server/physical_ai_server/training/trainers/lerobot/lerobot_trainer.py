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

    def __init__(self):
        super().__init__()
        self.logger = get_logger('LerobotTrainer')
        self.current_step = 0

    # TODO: Uncomment when training metrics is implemented
    # def send_training_metrics(self):
    #     pass

    def train(self, cfg: TrainPipelineConfig, stop_event=None):
        cfg.validate()
        self.logger.info(pformat(cfg.to_dict()))

        if cfg.wandb.enable and cfg.wandb.project:
            wandb_logger = WandBLogger(cfg)
        else:
            wandb_logger = None
            self.logger.info(colored('Logs will be saved locally.', 'yellow', attrs=['bold']))

        if cfg.seed is not None:
            set_seed(cfg.seed)

        device = get_safe_torch_device(cfg.policy.device, log=True)
        torch.backends.cudnn.benchmark = True
        torch.backends.cuda.matmul.allow_tf32 = True

        self.logger.info('Creating dataset')
        dataset = make_dataset(cfg)

        eval_env = None
        if cfg.eval_freq > 0 and cfg.env is not None:
            self.logger.info('Creating env')
            eval_env = make_env(
                cfg.env,
                n_envs=cfg.eval.batch_size,
                use_async_envs=cfg.eval.use_async_envs
            )

        self.logger.info('Creating policy')
        policy = make_policy(
            cfg=cfg.policy,
            ds_meta=dataset.meta,
        )

        self.logger.info('Creating optimizer and scheduler')
        optimizer, lr_scheduler = make_optimizer_and_scheduler(cfg, policy)
        grad_scaler = GradScaler(device.type, enabled=cfg.policy.use_amp)

        step = 0  # number of policy updates (forward + backward + optim)

        if cfg.resume:
            step, optimizer, lr_scheduler = load_training_state(
                cfg.checkpoint_path,
                optimizer,
                lr_scheduler
            )

        num_learnable_params = sum(p.numel() for p in policy.parameters() if p.requires_grad)
        num_total_params = sum(p.numel() for p in policy.parameters())

        self.logger.info(colored('Output dir:', 'yellow', attrs=['bold']) + f' {cfg.output_dir}')
        if cfg.env is not None:
            self.logger.info(f'{cfg.env.task=}')
        self.logger.info(f'{cfg.steps=} ({format_big_number(cfg.steps)})')
        self.logger.info(f'{dataset.num_frames=} ({format_big_number(dataset.num_frames)})')
        self.logger.info(f'{dataset.num_episodes=}')
        self.logger.info(f'{num_learnable_params=} ({format_big_number(num_learnable_params)})')
        self.logger.info(f'{num_total_params=} ({format_big_number(num_total_params)})')

        # create dataloader for offline training
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

        policy.train()

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

        self.logger.info('Start offline training on a fixed dataset')
        for _ in range(step, cfg.steps):
            if stop_event and stop_event.is_set():
                self.logger.info('Training stopped by stop event')
                break
            self.current_step = step + 1
            start_time = time.perf_counter()
            batch = next(dl_iter)
            train_tracker.dataloading_s = time.perf_counter() - start_time

            for key in batch:
                if isinstance(batch[key], torch.Tensor):
                    batch[key] = batch[key].to(device, non_blocking=True)

            train_tracker, output_dict = self._update_policy(
                train_tracker,
                policy,
                batch,
                optimizer,
                cfg.optimizer.grad_clip_norm,
                grad_scaler=grad_scaler,
                lr_scheduler=lr_scheduler,
                use_amp=cfg.policy.use_amp,
            )
            step += 1
            train_tracker.step()
            is_log_step = cfg.log_freq > 0 and step % cfg.log_freq == 0
            is_saving_step = step % cfg.save_freq == 0 or step == cfg.steps
            is_eval_step = cfg.eval_freq > 0 and step % cfg.eval_freq == 0

            if is_log_step:
                self.logger.info(str(train_tracker))
                if wandb_logger:
                    wandb_log_dict = train_tracker.to_dict()
                    if output_dict:
                        wandb_log_dict.update(output_dict)
                    wandb_logger.log_dict(wandb_log_dict, step)
                train_tracker.reset_averages()

            if cfg.save_checkpoint and is_saving_step:
                self.logger.info(f'Checkpoint policy after step {step}')
                checkpoint_dir = get_step_checkpoint_dir(cfg.output_dir, cfg.steps, step)
                save_checkpoint(checkpoint_dir, step, cfg, policy, optimizer, lr_scheduler)
                update_last_checkpoint(checkpoint_dir)
                if wandb_logger:
                    wandb_logger.log_policy(checkpoint_dir)

            if cfg.env and is_eval_step:
                step_id = get_step_identifier(step, cfg.steps)
                self.logger.info(f'Eval policy at step {step}')
                with (
                    torch.no_grad(),
                    torch.autocast(device_type=device.type)
                    if cfg.policy.use_amp else nullcontext(),
                ):
                    eval_info = eval_policy(
                        eval_env,
                        policy,
                        cfg.eval.n_episodes,
                        videos_dir=cfg.output_dir / 'eval' / f'videos_step_{step_id}',
                        max_episodes_rendered=4,
                        start_seed=cfg.seed,
                    )

                eval_metrics = {
                    'avg_sum_reward': AverageMeter('∑rwrd', ':.3f'),
                    'pc_success': AverageMeter('success', ':.1f'),
                    'eval_s': AverageMeter('eval_s', ':.3f'),
                }
                eval_tracker = MetricsTracker(
                    cfg.batch_size,
                    dataset.num_frames,
                    dataset.num_episodes,
                    eval_metrics,
                    initial_step=step
                )
                eval_tracker.eval_s = eval_info['aggregated'].pop('eval_s')
                eval_tracker.avg_sum_reward = eval_info['aggregated'].pop('avg_sum_reward')
                eval_tracker.pc_success = eval_info['aggregated'].pop('pc_success')
                self.logger.info(str(eval_tracker))
                if wandb_logger:
                    wandb_log_dict = {**eval_tracker.to_dict(), **eval_info}
                    wandb_logger.log_dict(wandb_log_dict, step, mode='eval')
                    wandb_logger.log_video(eval_info['video_paths'][0], step, mode='eval')

        if eval_env:
            eval_env.close()
        self.logger.info('End of training')

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
        start_time = time.perf_counter()
        device = get_device_from_parameters(policy)
        policy.train()
        with torch.autocast(device_type=device.type) if use_amp else nullcontext():
            loss, output_dict = policy.forward(batch)
        grad_scaler.scale(loss).backward()

        grad_scaler.unscale_(optimizer)

        grad_norm = torch.nn.utils.clip_grad_norm_(
            policy.parameters(),
            grad_clip_norm,
            error_if_nonfinite=False,
        )

        with lock if lock is not None else nullcontext():
            grad_scaler.step(optimizer)
        grad_scaler.update()

        optimizer.zero_grad()

        if lr_scheduler is not None:
            lr_scheduler.step()

        if has_method(policy, 'update'):
            policy.update()

        train_metrics.loss = loss.item()
        train_metrics.grad_norm = grad_norm.item()
        train_metrics.lr = optimizer.param_groups[0]['lr']
        train_metrics.update_s = time.perf_counter() - start_time
        return train_metrics, output_dict

    def get_current_step(self):
        return self.current_step
