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
from typing import List

import matplotlib.pyplot as plt
import numpy as np


class VisualizationManager:

    def __init__(self, default_save_dir: str = os.path.expanduser('~/.cache')):
        self.default_save_dir = default_save_dir
        os.makedirs(self.default_save_dir, exist_ok=True)

    def plot_action_comparison(
        self,
        episode_idx: int,
        state_across_time: np.ndarray,
        gt_action_across_time: np.ndarray,
        pred_action_across_time: np.ndarray,
        save_path: str = None,
        mse: float = None
    ) -> None:

        action_dim = gt_action_across_time.shape[1]

        fig, axes = plt.subplots(
            nrows=action_dim, ncols=1, figsize=(12, 3 * action_dim))
        if action_dim == 1:
            axes = [axes]

        # Add global title with MSE information
        title = f'Episode {episode_idx} - Action Prediction Evaluation'
        if mse is not None:
            title += f' (MSE: {mse:.6f})'
        fig.suptitle(title, fontsize=14, color='blue')

        # Plot each action dimension
        for i, ax in enumerate(axes):
            self._plot_single_action_dimension(
                ax,
                i,
                state_across_time,
                gt_action_across_time,
                pred_action_across_time
            )

        plt.tight_layout()

        # Save plot if path is provided
        if save_path:
            self._save_plot(save_path)
        else:
            # Auto-generate path if not provided
            auto_path = os.path.join(
                self.default_save_dir, f'episode_{episode_idx}_action_comparison.png')
            self._save_plot(auto_path)

    def _plot_single_action_dimension(
        self,
        ax: plt.Axes,
        dim_idx: int,
        state_across_time: np.ndarray,
        gt_action_across_time: np.ndarray,
        pred_action_across_time: np.ndarray
    ) -> None:

        if state_across_time.shape[1] == gt_action_across_time.shape[1]:
            ax.plot(
                state_across_time[:, dim_idx], label='Current State', alpha=0.7, linestyle='--')

        # Plot ground truth and predicted actions
        ax.plot(
            gt_action_across_time[:, dim_idx],
            label='Ground Truth Action',
            color='green',
            linewidth=2)
        ax.plot(
            pred_action_across_time[:, dim_idx],
            label='Predicted Action',
            color='red',
            linewidth=2)

        # Configure plot appearance
        ax.set_title(f'Action Dimension {dim_idx}')
        ax.set_xlabel('Time Step')
        ax.set_ylabel('Action Value')
        ax.legend()
        ax.grid(True, alpha=0.3)

    def _save_plot(self, save_path: str) -> None:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f'Plot saved to: {save_path}')

    def plot_episode_mse_comparison(
        self,
        episode_mses: List[float],
        save_path: str = None,
        title: str = 'Episode MSE Comparison'
    ) -> None:

        if not episode_mses:
            print('No episode MSE data to plot')
            return

        # Create figure and axis
        _, ax = plt.subplots(figsize=(max(12, len(episode_mses) * 0.8), 6))

        # Episode indices
        episode_indices = list(range(len(episode_mses)))

        # Create bar plot
        bars = ax.bar(
            episode_indices,
            episode_mses,
            alpha=0.7,
            color='skyblue',
            edgecolor='navy')

        # Highlight the episode with highest MSE
        if episode_mses:
            max_mse_idx = np.argmax(episode_mses)
            bars[max_mse_idx].set_color('red')
            bars[max_mse_idx].set_alpha(0.8)

        # Highlight the episode with lowest MSE
        if episode_mses:
            min_mse_idx = np.argmin(episode_mses)
            bars[min_mse_idx].set_color('green')
            bars[min_mse_idx].set_alpha(0.8)

        # Configure plot appearance
        ax.set_xlabel('Episode Index')
        ax.set_ylabel('MSE Value')
        ax.set_title(title)
        ax.grid(True, alpha=0.3, axis='y')

        # Add value labels on bars
        for i, mse in enumerate(episode_mses):
            ax.text(
                i,
                mse + max(episode_mses) * 0.01,
                f'{mse:.4f}',
                ha='center',
                va='bottom',
                fontsize=8,
                rotation=45
            )

        # Add statistics text
        if episode_mses:
            stats_text = f'Mean: {np.mean(episode_mses):.4f}\n'
            stats_text += f'Std: {np.std(episode_mses):.4f}\n'
            stats_text += f'Min: {np.min(episode_mses):.4f} (Episode {np.argmin(episode_mses)})\n'
            stats_text += f'Max: {np.max(episode_mses):.4f} (Episode {np.argmax(episode_mses)})'

            ax.text(
                0.02,
                0.98,
                stats_text,
                transform=ax.transAxes,
                verticalalignment='top',
                bbox={'boxstyle': 'round', 'facecolor': 'white', 'alpha': 0.8}
            )

        # Create legend
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='skyblue', alpha=0.7, label='Episode MSE'),
            Patch(facecolor='green', alpha=0.8, label='Best Episode (Lowest MSE)'),
            Patch(facecolor='red', alpha=0.8, label='Worst Episode (Highest MSE)')
        ]
        ax.legend(handles=legend_elements, loc='upper right')

        plt.tight_layout()

        # Save plot if path is provided
        if save_path:
            self._save_plot(save_path)
        else:
            # Auto-generate path if not provided
            auto_path = os.path.join(self.default_save_dir, 'episode_mse_comparison.png')
            self._save_plot(auto_path)

    def plot_episode_mse_distribution(
            self,
            episode_mses: List[float],
            save_path: str = None) -> None:

        if not episode_mses:
            print('No episode MSE data to plot')
            return

        # Create figure and axis
        _, ax = plt.subplots(figsize=(10, 6))

        # Plot histogram of MSE values
        ax.hist(
            episode_mses,
            bins=20,
            color='blue',
            alpha=0.7,
            edgecolor='black',
            label='MSE Distribution')

        # Add vertical lines for mean and median
        mean_mse = np.mean(episode_mses)
        median_mse = np.median(episode_mses)

        ax.axvline(
            mean_mse,
            color='red',
            linestyle='--',
            linewidth=2,
            label=f'Mean: {mean_mse:.4f}'
        )
        ax.axvline(
            median_mse,
            color='green',
            linestyle='--',
            linewidth=2,
            label=f'Median: {median_mse:.4f}'
        )

        # Configure plot appearance
        ax.set_xlabel('MSE Value')
        ax.set_ylabel('Frequency')
        ax.set_title('Distribution of MSE Values Across Episodes')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Add statistics text
        stats_text = f'Episodes: {len(episode_mses)}\n'
        stats_text += f'Mean: {mean_mse:.4f}\n'
        stats_text += f'Std: {np.std(episode_mses):.4f}\n'
        stats_text += f'Min: {np.min(episode_mses):.4f}\n'
        stats_text += f'Max: {np.max(episode_mses):.4f}'

        bbox = {'boxstyle': 'round', 'facecolor': 'white', 'alpha': 0.8}
        ax.text(
            0.98,
            0.98,
            stats_text,
            transform=ax.transAxes,
            verticalalignment='top',
            horizontalalignment='right',
            bbox=bbox
        )

        plt.tight_layout()

        # Save plot if path is provided
        if save_path:
            self._save_plot(save_path)
        else:
            # Auto-generate path if not provided
            auto_path = os.path.join(self.default_save_dir, 'episode_mse_distribution.png')
            self._save_plot(auto_path)
