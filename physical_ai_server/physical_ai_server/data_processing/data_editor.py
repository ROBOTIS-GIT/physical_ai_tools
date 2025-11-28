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

from contextlib import suppress
from dataclasses import dataclass
import json
import logging
from pathlib import Path
import re
import shutil
from typing import Any, Callable, Iterable, List, Optional, Set

import pandas as pd
from physical_ai_server.utils.file_utils import FileIO


@dataclass
class MergeResult:
    output_dir: Path
    total_parquet_processed: int
    total_episodes: int | str
    dataset_episode_counts: List[int]


@dataclass
class DeleteResult:
    dataset_dir: Path
    deleted_episode: int
    frames_removed: int
    videos_removed: int
    success: bool


@dataclass
class BatchDeleteResult:
    dataset_dir: Path
    deleted_episodes: List[int]
    total_frames_removed: int
    total_videos_removed: int
    success: bool


def _default_logger(verbose: bool) -> logging.Logger:
    logger = logging.getLogger('DataEditor')
    if not logger.handlers:
        handler = logging.StreamHandler()
        formatter = logging.Formatter('[%(levelname)s] %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    logger.setLevel(logging.INFO if verbose else logging.INFO)
    return logger


class DataEditor:
    DEFAULT_CHUNK_NAME: str = 'chunk-000'
    EPISODE_INDEX_WIDTH: int = 6
    DELETE_STEM_RE = re.compile(
        fr'^episode_(\d{{{EPISODE_INDEX_WIDTH}}})$')
    DELETE_PATCH_KEYS = {'episode_index', 'index'}
    MERGE_NUM_KEYS: List[str] = [
        'total_episodes',
        'total_frames',
        'total_videos'
    ]

    def __init__(
        self, *, verbose: bool = False, logger: Optional[logging.Logger] = None
    ):
        self.verbose = verbose
        self.logger = logger or _default_logger(verbose)
        self.total_task_num = 0

    def _log(self, msg: str, level: int = logging.INFO):
        if level == logging.INFO and not self.verbose:
            return
        self.logger.log(level, msg)

    @staticmethod
    def _extract_idx_from_name(
        name_with_index: str, default_pad: int = 6
    ) -> int:
        num_re = re.compile(r'(\d+)(?=\.parquet$|\.mp4$|$)')
        match = num_re.search(name_with_index)
        if not match:
            raise ValueError(
                f'Could not find a numeric index in {name_with_index}')
        return int(match.group(1))

    @staticmethod
    def _natural_sort_paths(paths: Iterable[Path]) -> List[Path]:
        return sorted(
            paths, key=lambda p: DataEditor._extract_idx_from_name(p.name)
        )

    def merge_datasets(
        self,
        dataset_paths: List,
        output_dir: Path,
        chunk_name: str = DEFAULT_CHUNK_NAME,
        verbose: bool | None = None
    ) -> MergeResult | None:

        output_dir = Path(output_dir)
        dataset_paths = [Path(p) for p in dataset_paths]
        if verbose is not None:
            self.verbose = verbose
        if not dataset_paths:
            self._log(
                'No dataset paths provided for merging.', logging.WARNING)
            return None

        self._log(
            f'Starting merge. Output directory: {output_dir}')
        data_dst_dir = output_dir / 'data' / chunk_name
        meta_dst_dir = output_dir / 'meta'
        video_dst_chunk_root = output_dir / 'videos' / chunk_name

        for p in (data_dst_dir, meta_dst_dir, video_dst_chunk_root.parent, video_dst_chunk_root):
            FileIO.safe_mkdir(p)

        cumulative_episode_offset_parquets = 0
        cumulative_frame_offset_parquets = 0
        total_parquets_processed_overall = 0
        actual_episode_counts_per_dataset: List[int] = []

        merged_task_mapping, task_name_to_index = self._build_merged_task_mapping(dataset_paths)

        self._log(
            '--- Processing Parquet Files and Determining Episode Counts ---',
            logging.INFO
        )

        for i, dataset_path in enumerate(dataset_paths):
            self._log(
                f'Processing dataset {i + 1}/{len(dataset_paths)}: {dataset_path}', logging.INFO
            )
            current_dataset_frames = 0
            info_path = dataset_path / 'meta' / 'info.json'
            info_data = FileIO.read_json(info_path, default={}) or {}
            current_dataset_frames = info_data.get(
                'total_frames', 0) if isinstance(info_data.get('total_frames'), int) else 0

            task_index_map = merged_task_mapping.get(i, {})

            processed_eps = self._copy_parquet_and_update_indices_for_merge(
                dataset_path,
                data_dst_dir,
                chunk_name,
                cumulative_episode_offset_parquets,
                cumulative_frame_offset_parquets,
                task_index_map,
            )
            self._log(
                f'Processed {processed_eps} Parquet '
                f'episode files from {dataset_path}.',
                logging.INFO
            )

            total_parquets_processed_overall += processed_eps
            current_dataset_episodes = info_data.get('total_episodes', 0)
            if not isinstance(current_dataset_episodes, int):
                current_dataset_episodes = 0

            # We prioritize total_episodes from info.json over actual processed Parquet count
            # to maintain consistency with metadata and
            # avoid index collisions when files are missing.
            # Log a warning if the difference is significant,
            # as it may indicate data inconsistency.
            if (
                current_dataset_episodes > 0
                and abs(current_dataset_episodes - processed_eps) > 1
            ):
                self._log(
                    f'Warning: total_episodes from info.json ({current_dataset_episodes}) '
                    f'differs significantly from number '
                    f'of processed Parquet files ({processed_eps}) '
                    f'in dataset {dataset_path}. This may indicate missing or extra files.',
                    logging.WARNING
                )

            episodes_offset_to_add = (
                current_dataset_episodes if current_dataset_episodes > 0 else processed_eps
            )

            actual_episode_counts_per_dataset.append(episodes_offset_to_add)
            cumulative_episode_offset_parquets += episodes_offset_to_add
            cumulative_frame_offset_parquets += current_dataset_frames

        self._log('--- Processing Metadata Files ---', logging.INFO)
        self._merge_all_meta_files(
            dataset_paths, meta_dst_dir, actual_episode_counts_per_dataset, task_name_to_index
        )

        self._log('--- Processing Video Files ---', logging.INFO)
        self._copy_all_videos_for_merge(
            dataset_paths, video_dst_chunk_root, chunk_name, actual_episode_counts_per_dataset
        )

        final_info_path = meta_dst_dir / 'info.json'
        final_total_episodes: int | str = 'N/A'
        if final_info_path.exists():
            final_total_episodes = FileIO.read_json(
                final_info_path, default={}).get('total_episodes', 'N/A')

        self._log(
            'Merge finished: '
            f'parquets={total_parquets_processed_overall}, '
            f'episodes={final_total_episodes}, output={output_dir}'
        )
        return MergeResult(
            output_dir=output_dir,
            total_parquet_processed=total_parquets_processed_overall,
            total_episodes=final_total_episodes,
            dataset_episode_counts=actual_episode_counts_per_dataset,
        )

    def _build_merged_task_mapping(
        self,
        dataset_paths: List[Path]
    ) -> tuple[dict[int, dict[int, int]], dict[str, int]]:
        """
        Build task_index mapping for each dataset.

        Uses episodes.jsonl to build episode_index -> task_instruction mapping,
        which is more reliable than tasks.jsonl alone.

        Returns
        -------
        tuple containing:
            - dataset_mappings: dict[int, dict[int, int]] where key is dataset index,
              value is {episode_index: new_task_index}
            - task_name_to_index: dict[str, int] where key is task_name, value is new_task_index

        """
        task_name_to_index: dict[str, int] = {}
        next_task_idx = 0
        dataset_mappings: dict[int, dict[int, int]] = {}

        for ds_idx, dataset_path in enumerate(dataset_paths):
            src_episodes_path = dataset_path / 'meta' / 'episodes.jsonl'
            if not src_episodes_path.exists():
                dataset_mappings[ds_idx] = {}
                continue

            src_episodes = FileIO.read_jsonl(src_episodes_path)
            episode_to_task_map = {}

            for episode_record in src_episodes:
                episode_idx = episode_record.get('episode_index')
                tasks_list = episode_record.get('tasks', [])

                if episode_idx is None or not tasks_list:
                    continue

                task_name = tasks_list[0]

                # Check if this task already exists in the global mapping
                if task_name in task_name_to_index:
                    new_task_idx = task_name_to_index[task_name]
                else:
                    # New task, assign new index
                    new_task_idx = next_task_idx
                    task_name_to_index[task_name] = new_task_idx
                    next_task_idx += 1

                # Map episode_index to new_task_index (normalize to int for consistency)
                try:
                    normalized_episode_idx = int(episode_idx)
                    episode_to_task_map[normalized_episode_idx] = new_task_idx
                except (ValueError, TypeError):
                    # If episode_idx cannot be converted to int, skip this episode
                    self._log(
                        f'Warning: Cannot convert episode_index'
                        f' {episode_idx} to int in {dataset_path}',
                        logging.WARNING
                    )

            dataset_mappings[ds_idx] = episode_to_task_map

        return dataset_mappings, task_name_to_index

    def _copy_parquet_and_update_indices_for_merge(
        self,
        src_root: Path,
        dst_data_dir: Path,
        chunk_name: str,
        episode_idx_offset: int,
        frame_idx_offset: int,
        episode_to_task_map: dict[int, int],
        verbose: bool | None = None,
    ) -> int:
        # Use provided verbose or fall back to instance verbose
        use_verbose = verbose if verbose is not None else self.verbose

        src_chunk_dir = src_root / 'data' / chunk_name
        if not src_chunk_dir.exists():
            if use_verbose:
                self._log(
                    f'Source chunk directory not found: {src_chunk_dir}', logging.INFO)
            return 0
        src_files = self._natural_sort_paths(src_chunk_dir.glob('episode_*.parquet'))
        if not src_files:
            if use_verbose:
                self._log(
                    f'No Parquet files found in {src_chunk_dir}', logging.INFO)
            return 0

        count_processed = 0
        for src_file_path in src_files:
            try:
                original_episode_idx = self._extract_idx_from_name(src_file_path.name)
                new_episode_idx = original_episode_idx + episode_idx_offset
                parquet_name = f'episode_{new_episode_idx:0{self.EPISODE_INDEX_WIDTH}d}.parquet'
                dst_file_path = dst_data_dir / parquet_name
                df = pd.read_parquet(src_file_path)
                if 'episode_index' in df.columns:
                    df['episode_index'] = new_episode_idx
                if 'index' in df.columns:
                    df['index'] = df['index'] + frame_idx_offset
                if 'frame_index' in df.columns:
                    df['frame_index'] = df['frame_index'] + frame_idx_offset

                # Update task_index using the episode-based mapping
                if 'task_index' in df.columns and episode_to_task_map:
                    # Look up the new task_index (original_episode_idx
                    # is already an int from _extract_idx_from_name)
                    if original_episode_idx in episode_to_task_map:
                        df['task_index'] = episode_to_task_map[original_episode_idx]
                    elif use_verbose:
                        self._log(
                            f'Warning: No task mapping found for episode {original_episode_idx} '
                            f'in {src_file_path.name}. task_index will remain unchanged.',
                            logging.WARNING
                        )

                df.to_parquet(dst_file_path)
                count_processed += 1
            except Exception as e:
                self._log(f'Error processing Parquet {src_file_path}: {e}', logging.WARNING)
                with suppress(Exception):
                    dst_file_path.unlink(missing_ok=True)  # type: ignore
        return count_processed

    def _merge_all_meta_files(
        self,
        dataset_paths: List[Path],
        meta_dst_dir: Path,
        actual_episode_counts: List[int],
        task_name_to_index: dict[str, int]
    ):
        current_meta_episode_offset = 0
        ep_stats_out = meta_dst_dir / 'episodes_stats.jsonl'
        ep_out = meta_dst_dir / 'episodes.jsonl'
        tasks_out = meta_dst_dir / 'tasks.jsonl'
        info_out = meta_dst_dir / 'info.json'

        for p in [ep_stats_out, ep_out, tasks_out, info_out]:
            p.unlink(missing_ok=True)

        for i, dataset_path in enumerate(dataset_paths):
            src_meta_dir = dataset_path / 'meta'
            eps_in_this_ds_for_meta = actual_episode_counts[i]
            if not src_meta_dir.exists():
                self._log(f'Meta dir not found: {src_meta_dir}', logging.INFO)
                current_meta_episode_offset += eps_in_this_ds_for_meta
                continue

            # episodes_stats.jsonl
            self._merge_episode_stats(
                src_meta_dir / 'episodes_stats.jsonl',
                ep_stats_out, current_meta_episode_offset)
            # episodes.jsonl
            self._merge_episodes(
                src_meta_dir / 'episodes.jsonl',
                ep_out, current_meta_episode_offset)
            # info.json
            self._merge_info(
                src_meta_dir / 'info.json',
                info_out)

            current_meta_episode_offset += eps_in_this_ds_for_meta

        # Generate tasks.jsonl from global task mapping (ensures consistency with parquet files)
        self._write_merged_tasks(tasks_out, task_name_to_index)

        # Ensure train split correctness
        info_data = FileIO.read_json(info_out, default={}) or {}
        total_eps = info_data.get('total_episodes', 0)
        if isinstance(total_eps, int):
            info_data.setdefault(
                'splits', {})['train'] = f'0:{total_eps - 1 if total_eps > 0 else 0}'
            FileIO.write_json(info_out, info_data)

    def _merge_episode_stats(self, src: Path, dst: Path, offset: int):
        if not src.exists():
            return
        base_data = FileIO.read_jsonl(dst) if dst.exists() else []
        new_data = FileIO.read_jsonl(src)
        for r_new in new_data:
            try:
                r_new['episode_index'] += offset
                if 'index' in r_new:
                    idx_val = r_new['index']
                    r_new['index'] = (
                        [x + offset for x in idx_val] if (
                            isinstance(idx_val, list)) else idx_val + offset
                    )
                if 'stats' in r_new:
                    r_new['stats'] = DataEditor._shift_positive_ints_recursive(
                        r_new['stats'], offset
                    )
            except Exception as e:
                self._log(f'Failed patching episode_stats record: {e}', logging.INFO)
        FileIO.write_jsonl(base_data + new_data, dst)

    def _merge_episodes(self, src: Path, dst: Path, offset: int):
        if not src.exists():
            return
        base_data = FileIO.read_jsonl(dst) if dst.exists() else []
        new_data = FileIO.read_jsonl(src)
        for r_new in new_data:
            try:
                r_new['episode_index'] += offset
                if 'index' in r_new:
                    idx_val = r_new['index']
                    r_new['index'] = (
                        [x + offset for x in idx_val] if (
                            isinstance(idx_val, list)) else idx_val + offset
                    )
            except Exception as e:
                self._log(f'Failed patching episode record: {e}', logging.INFO)
        FileIO.write_jsonl(base_data + new_data, dst)

    def _write_merged_tasks(self, dst: Path, task_name_to_index: dict[str, int]):
        """Write tasks.jsonl using the globally computed task_name_to_index mapping."""
        tasks_list = [
            {'task': task_name, 'task_index': task_idx}
            for task_name, task_idx in task_name_to_index.items()
        ]
        FileIO.write_jsonl(sorted(tasks_list, key=lambda x: x['task_index']), dst)
        self.total_task_num = len(tasks_list)

    def _merge_info(self, src: Path, dst: Path):
        if not src.exists():
            return
        d_base = FileIO.read_json(dst, default={}) or {}
        d_new = FileIO.read_json(src, default={}) or {}
        merged_info = d_base.copy()
        for k in self.MERGE_NUM_KEYS:
            merged_info[k] = merged_info.get(k, 0) + d_new.get(k, 0)
        for k, v in d_new.items():
            if (k not in self.MERGE_NUM_KEYS and k != 'splits') or (k == 'splits' and not d_base):
                merged_info[k] = v
        if hasattr(self, 'total_task_num') and self.total_task_num > 0:
            merged_info['total_tasks'] = self.total_task_num
        FileIO.write_json(dst, merged_info)

    @staticmethod
    def _shift_positive_ints_recursive(obj: Any, offset: int) -> Any:
        if isinstance(obj, int):
            return obj + offset
        if isinstance(obj, list):
            return [
                DataEditor._shift_positive_ints_recursive(x, offset) for x in obj
            ]
        if isinstance(obj, dict):
            return {
                k: DataEditor._shift_positive_ints_recursive(v, offset) for k, v in obj.items()
            }
        return obj

    @staticmethod
    def _parse_episode_index_from_stem(name: str) -> Optional[int]:
        match = DataEditor.DELETE_STEM_RE.match(name)
        return int(match.group(1)) if match else None

    @staticmethod
    def _add_offset_value(val: Any, off: int):
        if isinstance(val, int):
            return val + off
        if isinstance(val, list):
            return [DataEditor._add_offset_value(x, off) for x in val]
        return val

    @staticmethod
    def _shift_delete_patch_indices_recursive(obj: Any, off: int):
        if isinstance(obj, dict):
            return {
                k: (
                    DataEditor._add_offset_value(v, off)
                    if k in DataEditor.DELETE_PATCH_KEYS
                    else DataEditor._shift_delete_patch_indices_recursive(v, off)
                )
                for k, v in obj.items()
            }
        if isinstance(obj, list):
            return [DataEditor._shift_delete_patch_indices_recursive(x, off) for x in obj]
        return obj

    def _copy_all_videos_for_merge(
        self,
        dataset_paths: List[Path],
        video_dst_chunk_root: Path,
        chunk_name: str,
        actual_episode_counts_per_dataset: List[int],
    ) -> None:
        cumulative_offset = 0
        for ds_idx, ds_path in enumerate(dataset_paths):
            src_chunk_dir = ds_path / 'videos' / chunk_name
            if not src_chunk_dir.exists():
                self._log(
                    f'Video chunk dir missing: {src_chunk_dir}', logging.INFO)
                cumulative_offset += actual_episode_counts_per_dataset[ds_idx]
                continue
            max_valid_ep = actual_episode_counts_per_dataset[ds_idx]

            cam_dirs = [d for d in src_chunk_dir.iterdir() if d.is_dir()]

            def _copy_set(video_files: List[Path], dst_dir: Path):
                FileIO.safe_mkdir(dst_dir)
                for vf in video_files:
                    try:
                        ep_idx = self._extract_idx_from_name(vf.name)
                        if ep_idx >= max_valid_ep:
                            continue  # skip videos beyond parquet count
                        new_idx = ep_idx + cumulative_offset
                        dst_file = dst_dir / f'episode_{new_idx:0{self.EPISODE_INDEX_WIDTH}d}.mp4'
                        if not dst_file.exists():
                            shutil.copy2(vf, dst_file)
                        else:
                            # Silently skip duplicates (INFO only)
                            if self.verbose:
                                self._log(f'Skipped existing video {dst_file}', logging.INFO)
                    except Exception as e:
                        self._log(f'Failed copying video {vf}: {e}', logging.WARNING)

            if cam_dirs:
                for cam_dir in cam_dirs:
                    dst_cam_dir = video_dst_chunk_root / cam_dir.name
                    video_files = self._natural_sort_paths(
                        cam_dir.glob('episode_*.mp4'))
                    _copy_set(video_files, dst_cam_dir)
            else:
                video_files = self._natural_sort_paths(
                    src_chunk_dir.glob('episode_*.mp4'))
                _copy_set(video_files, video_dst_chunk_root)

            cumulative_offset += actual_episode_counts_per_dataset[ds_idx]

    def _shift_episode_paths(
        self,
        paths: List[Path],
        removed_idx: int,
        pad: int,
        after_move: Optional[Callable[[Path, Path, int], None]] = None,
    ) -> None:
        sorted_paths = self._natural_sort_paths(paths)
        for p in sorted_paths:
            name = p.name
            try:
                ep_idx = self._extract_idx_from_name(name)
            except Exception:
                continue
            if ep_idx <= removed_idx:
                continue
            new_idx = ep_idx - 1
            # Preserve extension (if any)
            if p.is_dir():
                new_name = f'episode_{new_idx:0{pad}d}'
            else:
                suffix = ''.join(p.suffixes)  # handles .mp4/.parquet
                new_name = f'episode_{new_idx:0{pad}d}{suffix}'
            new_path = p.with_name(new_name)
            try:
                p.rename(new_path)
                if after_move:
                    after_move(p, new_path, new_idx)
            except Exception as e:
                self._log(
                    f'Failed renaming {p} -> {new_path}: {e}', logging.WARNING)

    def delete_episodes_batch(
        self,
        dataset_dir: str,
        episode_indices_to_delete: List[int],
        chunk_name: str = DEFAULT_CHUNK_NAME,
        verbose: bool | None = None
    ) -> BatchDeleteResult:
        """
        Delete multiple episodes in a single batch operation.

        Much faster than deleting one by one.
        """
        if verbose is not None:
            self.verbose = verbose

        dataset_dir = Path(dataset_dir).resolve()
        if not dataset_dir.is_dir():
            self._log(
                f'Dataset directory not found: {dataset_dir}', logging.ERROR
            )
            raise FileNotFoundError(f'Dataset directory not found: {dataset_dir}')

        # Sort and convert to set for fast lookup
        episodes_to_delete = set(episode_indices_to_delete)
        self._log(
            f'Batch deleting {len(episodes_to_delete)} episodes in {dataset_dir}'
        )

        total_frames_removed = 0
        total_videos_removed = 0

        data_chunk_dir = dataset_dir / 'data' / chunk_name
        video_chunk_dir = dataset_dir / 'videos' / chunk_name
        images_root_dir = dataset_dir / 'images'
        meta_dir = dataset_dir / 'meta'

        # Build episode index mapping (old_idx -> new_idx)
        # Count how many episodes before each index are being deleted
        all_parquet_files = sorted(
            data_chunk_dir.glob('episode_*.parquet'),
            key=lambda p: self._extract_idx_from_name(p.name)
        )

        episode_mapping = {}  # old_idx -> new_idx
        deleted_count = 0

        for parquet_file in all_parquet_files:
            old_idx = self._extract_idx_from_name(parquet_file.name)
            if old_idx in episodes_to_delete:
                deleted_count += 1
                episode_mapping[old_idx] = None  # Mark for deletion
            else:
                episode_mapping[old_idx] = old_idx - deleted_count

        # 1. Delete parquet files and count frames
        self._log('Deleting parquet files...')
        for old_idx in episodes_to_delete:
            parquet_file = data_chunk_dir / (
                f'episode_{old_idx:0{self.EPISODE_INDEX_WIDTH}d}.parquet'
            )
            if parquet_file.exists():
                with suppress(Exception):
                    df = pd.read_parquet(parquet_file)
                    total_frames_removed += len(df)
                parquet_file.unlink()

        # 2. Delete video files
        self._log('Deleting video files...')
        camera_subdirs = [
            d for d in video_chunk_dir.iterdir() if d.is_dir()
        ] if video_chunk_dir.exists() else []

        for old_idx in episodes_to_delete:
            video_name = f'episode_{old_idx:0{self.EPISODE_INDEX_WIDTH}d}.mp4'
            if camera_subdirs:
                for cam_subdir in camera_subdirs:
                    video_file = cam_subdir / video_name
                    if video_file.exists():
                        video_file.unlink()
                        total_videos_removed += 1
            else:
                video_file = video_chunk_dir / video_name
                if video_file.exists():
                    video_file.unlink()
                    total_videos_removed += 1

        # 3. Delete image folders
        self._log('Deleting image folders...')
        if images_root_dir.exists():
            for old_idx in episodes_to_delete:
                folder_name = f'episode_{old_idx:0{self.EPISODE_INDEX_WIDTH}d}'
                for cam_obs_dir in images_root_dir.iterdir():
                    if cam_obs_dir.is_dir():
                        image_folder = cam_obs_dir / folder_name
                        if image_folder.exists():
                            shutil.rmtree(image_folder)

        # 4. Rename remaining files in batch
        self._log('Renaming remaining files...')
        self._batch_rename_and_update_parquets(
            data_chunk_dir, episode_mapping
        )

        # 5. Rename video files
        if camera_subdirs:
            for cam_subdir in camera_subdirs:
                self._batch_rename_files(
                    cam_subdir, episode_mapping, '.mp4'
                )
        else:
            self._batch_rename_files(
                video_chunk_dir, episode_mapping, '.mp4'
            )

        # 6. Rename image folders
        if images_root_dir.exists():
            for cam_obs_dir in images_root_dir.iterdir():
                if cam_obs_dir.is_dir():
                    self._batch_rename_folders(
                        cam_obs_dir, episode_mapping
                    )

        # 7. Update metadata files
        self._log('Updating metadata files...')
        self._batch_update_meta_files(
            meta_dir, episodes_to_delete, episode_mapping
        )

        # 8. Update info.json
        info_path = meta_dir / 'info.json'
        if info_path.exists():
            meta_info = FileIO.read_json(info_path, default={}) or {}
            if isinstance(meta_info.get('total_episodes'), int):
                meta_info['total_episodes'] = max(
                    0, meta_info['total_episodes'] - len(episodes_to_delete)
                )
            if isinstance(meta_info.get('total_frames'), int):
                meta_info['total_frames'] = max(
                    0, meta_info['total_frames'] - total_frames_removed
                )
            if total_videos_removed > 0 and isinstance(
                    meta_info.get('total_videos'), int):
                meta_info['total_videos'] = max(
                    0, meta_info['total_videos'] - total_videos_removed
                )

            # Update splits
            if (
                isinstance(meta_info.get('splits'), dict) and
                isinstance(meta_info['splits'].get('train'), str)
            ):
                new_total = meta_info['total_episodes']
                if new_total == 0:
                    meta_info['splits']['train'] = '0:0'
                else:
                    meta_info['splits']['train'] = f'0:{new_total - 1}'

            FileIO.write_json(info_path, meta_info)

        self._log(
            f'Batch deletion complete: {len(episodes_to_delete)} '
            f'episodes, {total_frames_removed} frames, '
            f'{total_videos_removed} videos removed'
        )

        return BatchDeleteResult(
            dataset_dir=dataset_dir,
            deleted_episodes=sorted(episodes_to_delete),
            total_frames_removed=total_frames_removed,
            total_videos_removed=total_videos_removed,
            success=True
        )

    def _batch_rename_and_update_parquets(
        self,
        data_dir: Path,
        episode_mapping: dict
    ):
        """Rename and update episode indices in parquet files."""
        # Sort by old_idx ascending (small to large)
        # This works because deleted episodes are already gone
        sorted_items = sorted(
            [(k, v) for k, v in episode_mapping.items() if v is not None],
            key=lambda x: x[0]
        )

        for old_idx, new_idx in sorted_items:
            if old_idx == new_idx:  # No change
                continue

            old_file = data_dir / (
                f'episode_{old_idx:0{self.EPISODE_INDEX_WIDTH}d}.parquet'
            )
            new_file = data_dir / (
                f'episode_{new_idx:0{self.EPISODE_INDEX_WIDTH}d}.parquet'
            )

            if old_file.exists():
                # Read, update episode_index, and save with new name
                df = pd.read_parquet(old_file)
                if 'episode_index' in df.columns:
                    df['episode_index'] = new_idx
                df.to_parquet(new_file, index=False)
                old_file.unlink()

    def _batch_rename_files(
        self,
        directory: Path,
        episode_mapping: dict,
        extension: str
    ):
        """Rename files based on episode mapping."""
        # Sort by old_idx ascending (small to large)
        # This works because deleted episodes are already gone
        sorted_items = sorted(
            [(k, v) for k, v in episode_mapping.items() if v is not None],
            key=lambda x: x[0]
        )

        for old_idx, new_idx in sorted_items:
            if old_idx == new_idx:
                continue

            old_file = directory / (
                f'episode_{old_idx:0{self.EPISODE_INDEX_WIDTH}d}{extension}'
            )
            new_file = directory / (
                f'episode_{new_idx:0{self.EPISODE_INDEX_WIDTH}d}{extension}'
            )

            if old_file.exists():
                old_file.rename(new_file)

    def _batch_rename_folders(
        self,
        directory: Path,
        episode_mapping: dict
    ):
        """Rename folders based on episode mapping."""
        # Sort by old_idx ascending (small to large)
        # This works because deleted episodes are already gone
        sorted_items = sorted(
            [(k, v) for k, v in episode_mapping.items() if v is not None],
            key=lambda x: x[0]
        )

        for old_idx, new_idx in sorted_items:
            if old_idx == new_idx:
                continue

            old_folder = directory / (
                f'episode_{old_idx:0{self.EPISODE_INDEX_WIDTH}d}'
            )
            new_folder = directory / (
                f'episode_{new_idx:0{self.EPISODE_INDEX_WIDTH}d}'
            )

            if old_folder.exists():
                old_folder.rename(new_folder)

    def _batch_update_meta_files(
        self,
        meta_dir: Path,
        episodes_to_delete: Set[int],
        episode_mapping: dict
    ):
        """Update metadata files by removing deleted episodes and remapping."""
        for meta_file_name in [
            'episodes_stats.jsonl', 'episodes.jsonl', 'episodes.json'
        ]:
            meta_file = meta_dir / meta_file_name
            if not meta_file.exists():
                continue

            try:
                content = meta_file.read_text().strip()
                if not content:
                    continue

                is_json_list = content.startswith('[') and content.endswith(']')

                if is_json_list:
                    data = json.loads(content)
                    new_data = []
                    for item in data:
                        ep_idx = item.get('episode_index')
                        if ep_idx in episodes_to_delete:
                            continue
                        if ep_idx in episode_mapping:
                            new_idx = episode_mapping[ep_idx]
                            if new_idx is not None:
                                item = self._remap_episode_indices(
                                    item, new_idx
                                )
                                new_data.append(item)
                    meta_file.write_text(
                        json.dumps(new_data, indent=2) + '\n'
                    )
                else:
                    # JSONL format
                    lines = content.splitlines()
                    new_lines = []
                    for line in lines:
                        if not line.strip():
                            continue
                        try:
                            item = json.loads(line)
                            ep_idx = item.get('episode_index')
                            if ep_idx in episodes_to_delete:
                                continue
                            if ep_idx in episode_mapping:
                                new_idx = episode_mapping[ep_idx]
                                if new_idx is not None:
                                    item = self._remap_episode_indices(
                                        item, new_idx
                                    )
                                    new_lines.append(
                                        json.dumps(item, separators=(',', ':'))
                                    )
                        except json.JSONDecodeError:
                            new_lines.append(line)

                    meta_file.write_text('\n'.join(new_lines) + '\n')
            except Exception as e:
                self._log(
                    f'Error updating {meta_file_name}: {e}', logging.WARNING
                )

    def _remap_episode_indices(self, item: dict, new_idx: int) -> dict:
        """Remap episode_index and related index fields."""
        item['episode_index'] = new_idx

        # Update 'index' field if it exists
        if 'index' in item:
            # For now, we don't remap frame indices as it's complex
            # and may not be necessary for all use cases
            pass

        return item

    def delete_episode(
        self,
        dataset_dir: str,
        episode_index_to_delete: int,
        chunk_name: str = DEFAULT_CHUNK_NAME,
        verbose: bool | None = None
    ) -> DeleteResult:
        """
        Delete a single episode.

        For deleting multiple episodes, use delete_episodes_batch for better performance.
        """
        if verbose is not None:
            self.verbose = verbose
        dataset_dir = Path(dataset_dir).resolve()
        if not dataset_dir.is_dir():
            self._log(
                f'Dataset directory not found: {dataset_dir}', logging.ERROR
            )
            raise FileNotFoundError(f'Dataset directory not found: {dataset_dir}')

        self._log(
            f'Deleting episode {episode_index_to_delete} in {dataset_dir} (chunk={chunk_name})'
        )

        target_episode_stem = f'episode_{episode_index_to_delete:0{self.EPISODE_INDEX_WIDTH}d}'
        removed_frame_count = 0
        removed_video_count = 0

        # 1. Delete parquet & count frames
        data_chunk_dir = dataset_dir / 'data' / chunk_name
        tgt_parquet = data_chunk_dir / f'{target_episode_stem}.parquet'
        if tgt_parquet.exists():
            with suppress(Exception):
                df = pd.read_parquet(tgt_parquet)
                removed_frame_count = len(df)
            with suppress(Exception):
                tgt_parquet.unlink()
        else:
            raise FileNotFoundError(
                f'Target parquet file does not exist : {tgt_parquet}')

        # 2. Delete associated videos
        video_chunk_dir = dataset_dir / 'videos' / chunk_name
        if video_chunk_dir.exists():
            camera_subdirs = [d for d in video_chunk_dir.iterdir() if d.is_dir()]
            if camera_subdirs:
                for cam_subdir in camera_subdirs:
                    tgt_video_file = cam_subdir / f'{target_episode_stem}.mp4'
                    if tgt_video_file.exists():
                        with suppress(Exception):
                            tgt_video_file.unlink()
                        removed_video_count += 1
                    else:
                        raise FileNotFoundError(
                            f'Target video file does not exist : {tgt_video_file}')
            else:
                tgt_video_file = video_chunk_dir / f'{target_episode_stem}.mp4'
                if tgt_video_file.exists():
                    with suppress(Exception):
                        tgt_video_file.unlink()
                    removed_video_count += 1
                else:
                    raise FileNotFoundError(
                        f'Target video file does not exist : {tgt_video_file}')

        # 3. Delete images folder(s)
        images_root_dir = dataset_dir / 'images'
        if images_root_dir.exists():
            for cam_obs_dir in images_root_dir.iterdir():
                if cam_obs_dir.is_dir():
                    tgt_image_dir = cam_obs_dir / target_episode_stem
                    if tgt_image_dir.exists() and tgt_image_dir.is_dir():
                        with suppress(Exception):
                            shutil.rmtree(tgt_image_dir)
                    else:
                        raise FileNotFoundError(
                            f'Target image directory does not exist : {tgt_image_dir}')

        # 4. Shift remaining parquet files & patch content
        if data_chunk_dir.exists():
            parquet_paths = list(data_chunk_dir.glob('episode_*.parquet'))
            self._shift_episode_paths(
                parquet_paths,
                episode_index_to_delete,
                self.EPISODE_INDEX_WIDTH,
                after_move=lambda old,
                new,
                new_idx: self._adjust_parquet_episode_index(new, -1, self.verbose),
            )
        else:
            raise FileNotFoundError(
                f'Target parquet file does not exist : {tgt_parquet}'
            )

        # 5. Shift remaining video filenames
        if video_chunk_dir.exists():
            camera_subdirs = [d for d in video_chunk_dir.iterdir() if d.is_dir()]
            if camera_subdirs:
                video_paths = [p for cam in camera_subdirs for p in cam.glob('episode_*.mp4')]
            else:
                video_paths = list(video_chunk_dir.glob('episode_*.mp4'))
            self._shift_episode_paths(
                video_paths, episode_index_to_delete, self.EPISODE_INDEX_WIDTH
            )
        else:
            raise FileNotFoundError(
                f'Target video directory does not exist : {video_chunk_dir}'
            )

        # 6. Shift image directories
        if images_root_dir.exists():
            image_dirs: List[Path] = []
            for cam_obs_dir in images_root_dir.iterdir():
                if cam_obs_dir.is_dir():
                    image_dirs.extend([d for d in cam_obs_dir.glob('episode_*') if d.is_dir()])
            self._shift_episode_paths(
                image_dirs, episode_index_to_delete, self.EPISODE_INDEX_WIDTH
            )

        # 7. Update meta json/jsonl files
        check_episode_file = False
        meta_dir = dataset_dir / 'meta'
        for name in ['episodes_stats.jsonl', 'episodes.jsonl', 'episodes.json']:
            path = meta_dir / name
            if path.exists():
                self._rewrite_meta_for_delete(
                    path, episode_index_to_delete, -1, self.verbose
                )
                check_episode_file = True

        if not check_episode_file:
            raise FileNotFoundError(
                    f'Meta file does not exist : {path}'
                )

        # 8. Update info.json counts
        info_path = meta_dir / 'info.json'
        if info_path.exists():
            try:
                meta_info = FileIO.read_json(info_path, default={}) or {}
                if isinstance(meta_info.get('total_episodes'), int):
                    meta_info['total_episodes'] = max(
                        0, meta_info['total_episodes'] - 1
                    )
                if isinstance(meta_info.get('total_frames'), int):
                    meta_info['total_frames'] = max(
                        0, meta_info['total_frames'] - removed_frame_count
                    )
                if removed_video_count > 0 and isinstance(meta_info.get('total_videos'), int):
                    meta_info['total_videos'] = max(
                        0, meta_info['total_videos'] - removed_video_count
                    )
                if (
                    isinstance(meta_info.get('splits'), dict) and
                    isinstance(meta_info['splits'].get('train'), str)
                ):
                    start_str, _, end_str = meta_info['splits']['train'].partition(':')
                    with suppress(ValueError):
                        start_idx = int(start_str)
                        new_end_idx = int(end_str) - 1
                        if meta_info['total_episodes'] == 0:
                            meta_info['splits']['train'] = '0:0'
                        elif start_idx <= new_end_idx:
                            meta_info['splits']['train'] = f'{start_idx}:{new_end_idx}'
                        else:
                            meta_info['splits']['train'] = f'{start_idx}:{start_idx}'
                FileIO.write_json(info_path, meta_info)
            except Exception as e:
                self._log(f'Error updating info.json: {e}', logging.WARNING)

        else:
            raise FileNotFoundError(
                f'Info file does not exist : {info_path}'
            )

        self._log(
            f'Episode {episode_index_to_delete} deleted successfully in {dataset_dir}')
        return DeleteResult(
            dataset_dir,
            episode_index_to_delete,
            removed_frame_count,
            removed_video_count,
            True
        )

    def _adjust_parquet_episode_index(
        self,
        path: Path,
        off: int,
        verbose: bool
    ) -> int:
        try:
            df = pd.read_parquet(path)
            nrows = len(df)
            if 'episode_index' in df.columns and pd.api.types.is_integer_dtype(
                    df['episode_index']):
                df['episode_index'] += off  # e.g., off = -1
                df.to_parquet(path, index=False)
            return nrows
        except Exception as e:
            if verbose:
                self._log(f'Could not patch Parquet {path}: {e}')
            return 0

    def _rewrite_meta_for_delete(
        self,
        path: Path,
        episode_index_to_remove: int,
        offset_for_shifting: int,
        verbose: bool
    ):
        try:
            content = path.read_text().strip()
            if not content:  # File is empty
                if verbose:
                    self._log(f'{path.name} is empty, skipping.')
                return
        except Exception as e:
            if verbose:
                self._log(f'Could not read {path.name}: {e}')
            return

        is_json_list_format = content.startswith('[') and content.endswith(']')
        new_data_list_for_output = []

        original_data_list_parsed = []
        if is_json_list_format:
            try:
                original_data_list_parsed = json.loads(content)
                if not isinstance(original_data_list_parsed, list):  # Should be a list
                    if verbose:
                        self._log(
                            f'Content of {path.name} looks like JSON '
                            f'list but is not a list. Processing as JSONL.'
                        )
                    is_json_list_format = False  # Fallback
            except json.JSONDecodeError as e:
                if verbose:
                    self._log(
                        f'Invalid JSON in {path.name}: {e}. Attempting line-by-line (JSONL).')
                is_json_list_format = False  # Fallback to JSONL

        items_to_process = original_data_list_parsed if (
            is_json_list_format) else content.splitlines()

        for item in items_to_process:
            obj_to_process = None
            is_decodable_json = False
            raw_line_if_not_json = item

            if is_json_list_format:
                obj_to_process = item
                is_decodable_json = True
            elif isinstance(item, str) and item.strip():
                try:
                    obj_to_process = json.loads(item)
                    is_decodable_json = True
                except json.JSONDecodeError:
                    is_decodable_json = False

            if is_decodable_json and isinstance(obj_to_process, dict):
                current_ep_idx = obj_to_process.get('episode_index')
                if current_ep_idx == episode_index_to_remove:
                    if verbose:
                        self._log(
                            f'Removing episode {episode_index_to_remove} entry from {path.name}')
                    continue  # Skip this episode
                if isinstance(current_ep_idx, int) and current_ep_idx > episode_index_to_remove:
                    obj_to_process = self._shift_delete_patch_indices_recursive(
                        obj_to_process, offset_for_shifting
                    )

            # Add to output list
            if is_json_list_format:
                new_data_list_for_output.append(obj_to_process)
            else:
                if is_decodable_json:
                    new_data_list_for_output.append(
                        json.dumps(obj_to_process, separators=(',', ':')))
                else:
                    new_data_list_for_output.append(raw_line_if_not_json)

        # Write back to file
        if is_json_list_format:
            path.write_text(
                json.dumps(new_data_list_for_output, indent=2) + '\n'
            )
        else:  # JSONL
            path.write_text(
                '\n'.join(new_data_list_for_output) + ('\n' if new_data_list_for_output else '')
            )

        if verbose:
            self._log(f'{path.name} updated.')

    def get_dataset_info(self, dataset_dir: str) -> dict:
        if dataset_dir is None or dataset_dir == '':
            self._log('Dataset path is empty', logging.ERROR)
            raise ValueError('Dataset path is empty')
        dataset_path = Path(dataset_dir).resolve()
        if not dataset_path.is_dir():
            self._log(
                f'Dataset path not found: {dataset_path}', logging.ERROR
            )
            raise FileNotFoundError(f'Dataset path not found: {dataset_path}')
        info_path = dataset_path / 'meta' / 'info.json'
        info_data = FileIO.read_json(info_path, default={}) or {}
        return info_data
