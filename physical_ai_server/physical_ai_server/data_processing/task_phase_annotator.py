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

"""Tool for annotating task phase labels on existing LeRobot datasets.

Task phase labels:
    0 = Before task (robot idle or approaching)
    1 = During task (robot executing the task)
    2 = Task complete (robot finished the task)

Usage:
    python task_phase_annotator.py \\
        --dataset-path /path/to/dataset \\
        --episode 0 \\
        --ranges "0-50:0,51-180:1,181-200:2"

    python task_phase_annotator.py \\
        --dataset-path /path/to/dataset \\
        --all-episodes \\
        --auto-label \\
        --before-ratio 0.1 \\
        --after-ratio 0.1
"""

import argparse
import json
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq


TASK_PHASE_BEFORE = 0
TASK_PHASE_DURING = 1
TASK_PHASE_COMPLETE = 2


def load_episode_parquet(dataset_path, episode_index):
    """Load a single episode parquet file."""
    data_dir = Path(dataset_path) / 'data'
    # Find the parquet file for this episode
    pattern = f'episode_{episode_index:06d}.parquet'
    for chunk_dir in sorted(data_dir.iterdir()):
        parquet_path = chunk_dir / pattern
        if parquet_path.exists():
            return parquet_path, pq.read_table(str(parquet_path))
    raise FileNotFoundError(
        f'Episode {episode_index} parquet file not found in {data_dir}')


def save_episode_parquet(parquet_path, table):
    """Save the modified parquet table back to disk."""
    pq.write_table(table, str(parquet_path))


def parse_ranges(range_str, num_frames):
    """Parse range string like '0-50:0,51-180:1,181-200:2' into per-frame labels.

    Args:
        range_str: Comma-separated ranges in format 'start-end:label'
        num_frames: Total number of frames in the episode

    Returns:
        numpy array of int64 labels for each frame
    """
    labels = np.full(num_frames, TASK_PHASE_DURING, dtype=np.int64)

    for segment in range_str.split(','):
        segment = segment.strip()
        range_part, label_str = segment.rsplit(':', 1)
        label = int(label_str)
        if label not in (TASK_PHASE_BEFORE, TASK_PHASE_DURING, TASK_PHASE_COMPLETE):
            raise ValueError(
                f'Invalid label {label}. Must be 0, 1, or 2.')

        start_str, end_str = range_part.split('-')
        start = int(start_str)
        end = min(int(end_str), num_frames - 1)
        labels[start:end + 1] = label

    return labels


def auto_label_episode(num_frames, before_ratio=0.1, after_ratio=0.1):
    """Automatically label an episode based on position ratios.

    Args:
        num_frames: Total number of frames
        before_ratio: Fraction of frames at start labeled as 'before task'
        after_ratio: Fraction of frames at end labeled as 'task complete'

    Returns:
        numpy array of int64 labels
    """
    labels = np.full(num_frames, TASK_PHASE_DURING, dtype=np.int64)
    before_count = max(1, int(num_frames * before_ratio))
    after_count = max(1, int(num_frames * after_ratio))

    labels[:before_count] = TASK_PHASE_BEFORE
    labels[num_frames - after_count:] = TASK_PHASE_COMPLETE

    return labels


def extend_action_with_phase(table, phase_labels):
    """Append task_phase values to the action column in the parquet table.

    The action column is expected to contain lists/arrays of float32 values.
    This function appends one extra float32 value (the task_phase) to each row.

    Args:
        table: PyArrow table with 'action' column
        phase_labels: numpy array of task phase labels (0, 1, or 2)

    Returns:
        Modified PyArrow table with extended action column
    """
    action_col = table.column('action')
    new_actions = []

    for i in range(len(action_col)):
        action_array = action_col[i].as_py()
        if isinstance(action_array, list):
            extended = action_array + [float(phase_labels[i])]
        else:
            extended = list(action_array) + [float(phase_labels[i])]
        new_actions.append(extended)

    # Replace the action column
    col_index = table.schema.get_field_index('action')
    new_action_array = pa.array(new_actions, type=pa.list_(pa.float32()))
    table = table.set_column(col_index, 'action', new_action_array)

    return table


def update_info_json(dataset_path, extra_dim_name='task_phase'):
    """Update meta/info.json to reflect the extended action dimension.

    Args:
        dataset_path: Root path of the dataset
        extra_dim_name: Name of the extra dimension added
    """
    info_path = Path(dataset_path) / 'meta' / 'info.json'
    with open(info_path, 'r', encoding='utf-8') as f:
        info = json.load(f)

    # Update action feature shape and names
    features = info.get('features', {})
    if 'action' in features:
        action_feature = features['action']
        current_shape = action_feature.get('shape', [])
        current_names = action_feature.get('names', [])

        # Check if task_phase is already added
        if extra_dim_name not in current_names:
            if isinstance(current_shape, list) and len(current_shape) == 1:
                action_feature['shape'] = [current_shape[0] + 1]
            elif isinstance(current_shape, (list, tuple)):
                new_shape = list(current_shape)
                new_shape[-1] = new_shape[-1] + 1
                action_feature['shape'] = new_shape

            if isinstance(current_names, list):
                action_feature['names'] = current_names + [extra_dim_name]

            features['action'] = action_feature
            info['features'] = features

    with open(info_path, 'w', encoding='utf-8') as f:
        json.dump(info, f, indent=4)

    print(f'Updated {info_path}')


def recompute_action_stats(dataset_path):
    """Recompute action statistics in meta/stats.json after extending actions.

    Args:
        dataset_path: Root path of the dataset
    """
    stats_path = Path(dataset_path) / 'meta' / 'stats.json'
    if not stats_path.exists():
        print(f'stats.json not found at {stats_path}, skipping stats update')
        return

    # Collect all action data
    data_dir = Path(dataset_path) / 'data'
    all_actions = []
    for chunk_dir in sorted(data_dir.iterdir()):
        if not chunk_dir.is_dir():
            continue
        for parquet_file in sorted(chunk_dir.glob('*.parquet')):
            table = pq.read_table(str(parquet_file))
            action_col = table.column('action')
            for i in range(len(action_col)):
                all_actions.append(action_col[i].as_py())

    if not all_actions:
        print('No action data found, skipping stats update')
        return

    actions_array = np.array(all_actions, dtype=np.float32)
    action_mean = actions_array.mean(axis=0).tolist()
    action_std = actions_array.std(axis=0).tolist()
    action_min = actions_array.min(axis=0).tolist()
    action_max = actions_array.max(axis=0).tolist()

    with open(stats_path, 'r', encoding='utf-8') as f:
        stats = json.load(f)

    stats['action'] = {
        'mean': action_mean,
        'std': action_std,
        'min': action_min,
        'max': action_max,
    }

    with open(stats_path, 'w', encoding='utf-8') as f:
        json.dump(stats, f, indent=4)

    print(f'Updated action statistics in {stats_path}')


def annotate_episode(dataset_path, episode_index, phase_labels):
    """Annotate a single episode with task phase labels.

    Args:
        dataset_path: Root path of the dataset
        episode_index: Episode number to annotate
        phase_labels: numpy array of phase labels for each frame
    """
    parquet_path, table = load_episode_parquet(dataset_path, episode_index)
    num_frames = len(table)

    if len(phase_labels) != num_frames:
        raise ValueError(
            f'Phase labels length ({len(phase_labels)}) does not match '
            f'episode frame count ({num_frames})')

    # Check if action already has task_phase appended
    action_col = table.column('action')
    first_action = action_col[0].as_py()
    info_path = Path(dataset_path) / 'meta' / 'info.json'
    with open(info_path, 'r', encoding='utf-8') as f:
        info = json.load(f)
    expected_original_dim = None
    features = info.get('features', {})
    if 'action' in features:
        names = features['action'].get('names', [])
        if 'task_phase' in names:
            expected_original_dim = len(names) - 1
        else:
            expected_original_dim = len(names)

    if expected_original_dim and len(first_action) > expected_original_dim:
        # Action already extended; update the last dimension
        new_actions = []
        for i in range(len(action_col)):
            action_array = list(action_col[i].as_py())
            action_array[-1] = float(phase_labels[i])
            new_actions.append(action_array)
        col_index = table.schema.get_field_index('action')
        new_action_array = pa.array(new_actions, type=pa.list_(pa.float32()))
        table = table.set_column(col_index, 'action', new_action_array)
        print(f'Updated existing task_phase labels for episode {episode_index}')
    else:
        table = extend_action_with_phase(table, phase_labels)
        print(f'Extended action with task_phase for episode {episode_index}')

    save_episode_parquet(parquet_path, table)
    print(f'Saved episode {episode_index} to {parquet_path}')


def get_episode_count(dataset_path):
    """Get the total number of episodes from the dataset metadata."""
    info_path = Path(dataset_path) / 'meta' / 'info.json'
    with open(info_path, 'r', encoding='utf-8') as f:
        info = json.load(f)
    return info.get('total_episodes', 0)


def main():
    parser = argparse.ArgumentParser(
        description='Annotate task phase labels on LeRobot datasets')
    parser.add_argument(
        '--dataset-path', type=str, required=True,
        help='Path to the LeRobot dataset root directory')
    parser.add_argument(
        '--episode', type=int, default=None,
        help='Episode index to annotate (single episode mode)')
    parser.add_argument(
        '--all-episodes', action='store_true',
        help='Annotate all episodes in the dataset')
    parser.add_argument(
        '--ranges', type=str, default=None,
        help='Frame ranges with labels, e.g. "0-50:0,51-180:1,181-200:2"')
    parser.add_argument(
        '--auto-label', action='store_true',
        help='Automatically label based on position ratios')
    parser.add_argument(
        '--before-ratio', type=float, default=0.1,
        help='Ratio of frames at start labeled as before_task (default: 0.1)')
    parser.add_argument(
        '--after-ratio', type=float, default=0.1,
        help='Ratio of frames at end labeled as task_complete (default: 0.1)')

    args = parser.parse_args()
    dataset_path = args.dataset_path

    if not Path(dataset_path).exists():
        print(f'Dataset path does not exist: {dataset_path}')
        return

    if args.all_episodes:
        total_episodes = get_episode_count(dataset_path)
        print(f'Annotating all {total_episodes} episodes...')
        for ep_idx in range(total_episodes):
            try:
                _, table = load_episode_parquet(dataset_path, ep_idx)
                num_frames = len(table)

                if args.auto_label:
                    labels = auto_label_episode(
                        num_frames, args.before_ratio, args.after_ratio)
                elif args.ranges:
                    labels = parse_ranges(args.ranges, num_frames)
                else:
                    print('Must specify --auto-label or --ranges')
                    return

                annotate_episode(dataset_path, ep_idx, labels)
            except FileNotFoundError:
                print(f'Skipping episode {ep_idx} (not found)')
                continue

    elif args.episode is not None:
        _, table = load_episode_parquet(dataset_path, args.episode)
        num_frames = len(table)

        if args.auto_label:
            labels = auto_label_episode(
                num_frames, args.before_ratio, args.after_ratio)
        elif args.ranges:
            labels = parse_ranges(args.ranges, num_frames)
        else:
            print('Must specify --auto-label or --ranges for labeling')
            return

        annotate_episode(dataset_path, args.episode, labels)
    else:
        print('Must specify --episode or --all-episodes')
        return

    # Update dataset metadata
    update_info_json(dataset_path)
    recompute_action_stats(dataset_path)
    print('Annotation complete.')


if __name__ == '__main__':
    main()
