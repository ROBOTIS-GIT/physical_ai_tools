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

import argparse
import os
from typing import Dict, List, Tuple

from lerobot.configs.default import DatasetConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
import numpy as np
from physical_ai_server.evaluation.visualization_manager import VisualizationManager
from physical_ai_server.inference.inference_manager import InferenceManager


class EvaluationManager:

    def __init__(self):
        self.visualization_manager = VisualizationManager()

    def load_dataset(
        self,
        repo_id: str,
        root: str = None,
        episodes: List[int] = None
    ) -> LeRobotDataset:
        dataset_config = DatasetConfig(
            repo_id=repo_id,
            root=root,
            episodes=episodes,
        )

        dataset = LeRobotDataset(
            repo_id=dataset_config.repo_id,
            root=dataset_config.root,
            episodes=dataset_config.episodes,
            delta_timestamps=None,
            image_transforms=None,
            revision=dataset_config.revision,
            video_backend=dataset_config.video_backend,
        )
        return dataset

    def get_episode_boundaries(
        self,
        dataset: LeRobotDataset,
        episode_idx: int
    ) -> Tuple[int, int, int]:
        if episode_idx >= dataset.num_episodes:
            raise ValueError(
                f'Episode index {episode_idx} exceeds available episodes {dataset.num_episodes}')

        episode_start = int(dataset.episode_data_index['from'][episode_idx])
        episode_end = int(dataset.episode_data_index['to'][episode_idx])
        episode_length = episode_end - episode_start
        return episode_start, episode_end, episode_length

    def get_episode_data(
        self,
        dataset: LeRobotDataset,
        episode_idx: int
    ) -> List[Dict]:
        episode_start, episode_end, _ = self.get_episode_boundaries(dataset, episode_idx)

        episode_frames = []
        for frame_idx in range(episode_start, episode_end):
            frame_data = dataset[frame_idx]
            episode_frames.append(frame_data)

        return episode_frames

    def extract_frame_components(
        self,
        frame_data: Dict
    ) -> Tuple[np.ndarray, np.ndarray, Dict[str, np.ndarray], str]:
        # Extract ground truth state and action
        gt_state = frame_data['observation.state'].numpy()
        gt_action = frame_data['action'].numpy()

        # Prepare observation data for inference
        images = {}
        for key, value in frame_data.items():
            if key.startswith('observation.images.'):
                # Convert tensor back to numpy array in expected format [H, W, C]
                img_tensor = value  # Shape: [C, H, W]
                img_np = img_tensor.permute(1, 2, 0).numpy()
                img_np = img_np.astype(np.float32)
                img_np = img_np * 255.0
                camera_name = key.replace('observation.images.', '')
                images[camera_name] = img_np

        task_instruction = frame_data.get('task')

        return gt_state, gt_action, images, task_instruction

    def calculate_mse(
        self,
        gt_actions: np.ndarray,
        pred_actions: np.ndarray
    ) -> float:
        return np.mean((gt_actions - pred_actions) ** 2)

    def evaluate_policy_on_episode(
        self,
        inference_manager: InferenceManager,
        dataset: LeRobotDataset,
        episode_idx: int,
        plot: bool = False,
        save_plot_path: str = None
    ) -> float:
        episode_start, _, episode_length = self.get_episode_boundaries(dataset, episode_idx)

        # Storage for trajectory data
        state_across_time = []
        gt_action_across_time = []
        pred_action_across_time = []

        # Process each frame in the episode
        for step_count in range(episode_length):
            frame_idx = episode_start + step_count
            frame_data = dataset[frame_idx]

            # Extract frame components
            gt_state, gt_action, images, task_instruction = self.extract_frame_components(
                frame_data)

            # Store ground truth data
            state_across_time.append(gt_state)
            gt_action_across_time.append(gt_action)

            # Get policy prediction
            predicted_action = inference_manager.predict(
                images=images,
                state=gt_state.tolist(),
                task_instruction=task_instruction
            )
            pred_action_across_time.append(predicted_action)

        # Convert to numpy arrays
        state_across_time = np.array(state_across_time)
        gt_action_across_time = np.array(gt_action_across_time)
        pred_action_across_time = np.array(pred_action_across_time)

        # Calculate evaluation metrics
        mse = self.calculate_mse(gt_action_across_time, pred_action_across_time)

        # Create visualization if requested
        if plot:
            self.visualization_manager.plot_action_comparison(
                episode_idx=episode_idx,
                state_across_time=state_across_time,
                gt_action_across_time=gt_action_across_time,
                pred_action_across_time=pred_action_across_time,
                save_path=save_plot_path,
                mse=mse
            )

        return mse

    def evaluate_policy_on_dataset(
        self,
        inference_manager: InferenceManager,
        dataset: LeRobotDataset,
        sample_episodes: List[int] = None,
        plot_episodes: bool = False,
        plot_summary: bool = True,
        save_plot_dir: str = None
    ) -> Dict[str, float]:

        episode_mses = []
        total_episodes = dataset.num_episodes
        if sample_episodes is None:
            sample_episodes = list(range(total_episodes))

        for episode_idx in sample_episodes:
            try:
                # Determine save path for this episode
                episode_save_path = None
                if plot_episodes and save_plot_dir:
                    episode_save_path = os.path.join(
                        save_plot_dir, f'episode_{episode_idx}_evaluation.png')

                # Evaluate episode
                mse = self.evaluate_policy_on_episode(
                    inference_manager=inference_manager,
                    dataset=dataset,
                    episode_idx=episode_idx,
                    plot=plot_episodes,
                    save_plot_path=episode_save_path
                )

                episode_mses.append(mse)
                print(f'Episode {episode_idx}: MSE = {mse:.6f}')

            except Exception as e:
                print(f'Failed to evaluate episode {episode_idx}: {e}')
                continue

        # Calculate aggregate metrics
        if episode_mses:
            results = {
                'mean_mse': np.mean(episode_mses),
                'std_mse': np.std(episode_mses),
                'min_mse': np.min(episode_mses),
                'max_mse': np.max(episode_mses),
                'best_episode_idx': np.argmin(episode_mses),
                'worst_episode_idx': np.argmax(episode_mses),
                'total_episodes': len(episode_mses),
                'episode_mses': episode_mses
            }
        else:
            results = {
                'mean_mse': float('inf'),
                'std_mse': 0.0,
                'min_mse': float('inf'),
                'max_mse': float('inf'),
                'best_episode_idx': -1,
                'worst_episode_idx': -1,
                'total_episodes': 0,
                'episode_mses': []
            }

        # Plot MSE comparison across episodes
        if plot_summary and episode_mses:
            mse_comparison_path = None
            mse_distribution_path = None

            if save_plot_dir:
                mse_comparison_path = os.path.join(save_plot_dir, 'overall_mse_comparison.png')
                mse_distribution_path = os.path.join(save_plot_dir, 'overall_mse_distribution.png')

            self.visualization_manager.plot_episode_mse_comparison(
                episode_mses=episode_mses,
                save_path=mse_comparison_path,
                title='Overall Episode MSE Comparison'
            )

            # Plot MSE distribution
            self.visualization_manager.plot_episode_mse_distribution(
                episode_mses=episode_mses,
                save_path=mse_distribution_path
            )

        return results


def main():
    parser = argparse.ArgumentParser(
        description='Evaluate a policy on a dataset.'
    )
    parser.add_argument(
        '--repo_id',
        type=str,
        required=True,
        help='The repository ID of the dataset on the Hugging Face Hub.'
    )
    parser.add_argument(
        '--policy_path',
        type=str,
        required=True,
        help='The path to the policy file to be evaluated.'
    )
    parser.add_argument(
        '--sample_episodes',
        type=int,
        nargs='+',
        default=None,
        help='A list of episode indices to evaluate. If not provided, all episodes are evaluated.'
    )
    parser.add_argument(
        '--save_plot_dir',
        type=str,
        default='./plots',
        help='The directory to save evaluation plots.'
    )
    args = parser.parse_args()

    # Initialize managers
    evaluation_manager = EvaluationManager()

    # Configuration
    repo_id = args.repo_id
    policy_path = args.policy_path

    # Load dataset
    dataset = evaluation_manager.load_dataset(repo_id)

    # Initialize inference manager
    inference_manager = InferenceManager()
    success, message = inference_manager.validate_policy(policy_path)

    if success:
        policy_loaded = inference_manager.load_policy()
        if policy_loaded:
            # Evaluate policy on entire dataset
            results = evaluation_manager.evaluate_policy_on_dataset(
                inference_manager=inference_manager,
                dataset=dataset,
                sample_episodes=args.sample_episodes,
                plot_episodes=True,
                plot_summary=True,
                save_plot_dir=args.save_plot_dir
            )
            print(f'Evaluation results: {results}')

        else:
            print('Failed to load policy')
    else:
        print(f'Policy validation failed: {message}')

    print('\nDataset evaluation completed!')


if __name__ == '__main__':
    main()
