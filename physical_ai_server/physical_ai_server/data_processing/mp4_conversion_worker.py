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
# Author: Claude AI Assistant

"""
Chained Dataset Conversion Worker.

Background process that converts rosbag2 episodes through a 3-stage pipeline:
  Stage 1: rosbag → rosbag + MP4 (RosbagToMp4Converter)
  Stage 2: rosbag + MP4 → LeRobot v2.1 (RosbagToLerobotConverter)
  Stage 3: LeRobot v2.1 → LeRobot v3.0 (RosbagToLerobotV30Converter)

Follows the HfApiWorker pattern using multiprocessing.Process.

Output structure:
    /workspace/rosbag2/{robot}_{task}/
    ├── 0/                    # Original episode
    ├── 0_converted/          # Stage 1 output (MP4)
    │   ├── episode.mcap
    │   ├── cam_*.mp4
    │   ├── robot.urdf
    │   └── meshes/
    ├── 1/
    └── 1_converted/
    /workspace/rosbag2/{robot}_{task}_lerobot_v21/   # Stage 2 output (v2.1)
    /workspace/rosbag2/{robot}_{task}_lerobot_v30/  # Stage 3 output (v3.0)
"""

import logging
import multiprocessing
import os
from pathlib import Path
import queue
import time
from typing import Dict, List, Optional


def _convert_single_episode_worker(episode_dir, output_dir, fps, use_hw, enable_smoothing):
    """Top-level function for ProcessPoolExecutor (must be picklable).

    Creates a fresh RosbagToMp4Converter instance in each worker process
    and converts a single episode to MP4 format.
    """
    from physical_ai_server.data_processing.convert_rosbag2mp4 import RosbagToMp4Converter
    converter = RosbagToMp4Converter(
        fps=fps,
        use_hardware_encoding=use_hw,
        enable_timestamp_smoothing=enable_smoothing,
    )
    results = converter.convert_episode(str(episode_dir), str(output_dir))
    # Check if any camera conversion succeeded
    success = any(
        result.success for result in results.values()
        if hasattr(result, 'success')
    )
    return str(episode_dir), success, results


class Mp4ConversionWorker:
    """
    Background worker for MP4 conversion.

    Uses multiprocessing.Process to run conversion in a separate process,
    following the HfApiWorker pattern.
    """

    def __init__(self):
        self.input_queue = multiprocessing.Queue()
        self.output_queue = multiprocessing.Queue()
        self.progress_queue = multiprocessing.Queue()
        self.process = None
        self.logger = logging.getLogger('Mp4ConversionWorker')

        # Task state management
        self.is_processing = False
        self.current_task = None
        self.start_time = None

        # Progress tracking
        self.current_progress = {
            'current': 0,
            'total': 0,
            'percentage': 0.0,
            'current_episode': '',
            'dataset_path': ''
        }

        # Basic config for the main process logger
        logging.basicConfig(
            level=logging.INFO,
            format='%(name)s - %(levelname)s - %(message)s'
        )

    def start(self) -> bool:
        """Start the worker process."""
        if self.process and self.process.is_alive():
            self.logger.warning('MP4 conversion worker process is already running.')
            return False

        try:
            self.logger.info('Starting MP4 conversion worker process...')

            self.process = multiprocessing.Process(
                target=self._worker_process_loop,
                args=(
                    self.input_queue,
                    self.output_queue,
                    self.progress_queue
                )
            )

            self.process.start()
            self.logger.info(
                f'MP4 conversion worker process started with PID: {self.process.pid}'
            )
            return True

        except Exception as e:
            self.logger.error(f'Failed to start MP4 conversion worker: {str(e)}')
            return False

    def stop(self, timeout: float = 3.0):
        """Stop the worker process."""
        if not self.is_alive():
            self.logger.info(
                'MP4 conversion worker process is not running or already stopped.'
            )
            return

        try:
            self.logger.info('Sending shutdown signal to MP4 conversion worker...')
            try:
                self.input_queue.put_nowait(None)
            except Exception:
                pass

            grace_timeout = min(max(timeout, 0.0), 1.0)
            if grace_timeout > 0:
                self.process.join(grace_timeout)

            if self.process.is_alive():
                self.logger.warning(
                    'MP4 conversion worker did not terminate gracefully. '
                    'Forcing termination now.'
                )
                self.process.kill()
                self.process.join(1.0)
        except Exception as e:
            self.logger.error(f'Error stopping MP4 conversion worker process: {e}')
        finally:
            self.process = None
            self.is_processing = False
            self.current_task = None
            self.start_time = None

    def is_alive(self) -> bool:
        """Check if the worker process is alive."""
        return self.process and self.process.is_alive()

    def send_request(self, request_data: dict) -> bool:
        """
        Send a conversion request to the worker.

        Args:
            request_data: Dict containing:
                - dataset_path: Path to the dataset directory
                - robot_type: Robot type string

        Returns:
            True if request was sent successfully.
        """
        if self.is_alive():
            self.input_queue.put(request_data)
            self.is_processing = True
            self.current_task = request_data
            self.start_time = time.time()
            return True
        else:
            self.logger.error(
                'Cannot send request, MP4 conversion worker process is not running.'
            )
            return False

    def get_result(self, block: bool = False, timeout: float = 0.1) -> Optional[tuple]:
        """Get result from the output queue."""
        try:
            return self.output_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return None

    def check_task_status(self) -> dict:
        """Check the current task status and return appropriate message."""
        result = {
            'operation': 'convert_mp4',
            'status': 'Idle',
            'dataset_path': '',
            'message': '',
            'progress': {
                'current': 0,
                'total': 0,
                'percentage': 0.0,
            }
        }

        if not self.is_alive():
            self.logger.error('MP4 conversion worker process died')
            result['status'] = 'Failed'
            result['message'] = 'MP4 conversion worker process died'
            return result

        if not self.is_processing:
            result['status'] = 'Idle'
            return result

        try:
            if self.current_task:
                result['dataset_path'] = self.current_task.get('dataset_path', '')

            # Check for progress updates from worker process
            self.current_progress = self._get_progress_from_queue()
            current = self.current_progress.get('current', 0)
            total = self.current_progress.get('total', 0)
            percentage = self.current_progress.get('percentage', 0.0)
            result['progress']['current'] = current
            result['progress']['total'] = total
            result['progress']['percentage'] = percentage

            # Check for task result
            task_result = self.get_result(block=False, timeout=0.1)
            if task_result:
                status, message = task_result
                if status == 'success':
                    log_message = f'MP4 conversion completed successfully:\n{message}'
                    self.logger.info(log_message)
                    self.is_processing = False
                    self.current_task = None

                    result['status'] = 'Success'
                    result['message'] = log_message
                    return result
                elif status == 'error':
                    log_message = f'MP4 conversion failed:\n{message}'
                    self.logger.error(log_message)
                    self.is_processing = False
                    self.current_task = None

                    result['status'] = 'Failed'
                    result['message'] = log_message
                    return result

            # Still processing
            result['status'] = 'Converting'
            current_episode = self.current_progress.get('current_episode', '')
            if current_episode:
                result['message'] = f'Converting episode {current_episode}'

            return result

        except Exception as e:
            log_message = f'Error checking MP4 conversion task status: {str(e)}'
            self.logger.error(log_message)
            result['status'] = 'Failed'
            result['message'] = log_message
            return result

    def is_busy(self) -> bool:
        """Check if the worker is currently processing a task."""
        return self.is_processing

    def _get_progress_from_queue(self) -> dict:
        """Get the latest progress information from worker process."""
        latest_progress = None
        try:
            while True:
                try:
                    latest_progress = self.progress_queue.get(block=False, timeout=0.01)
                except queue.Empty:
                    break
        except Exception as e:
            self.logger.error(f'Error updating progress from worker: {e}')

        return latest_progress if latest_progress else self.current_progress

    @staticmethod
    def _worker_process_loop(input_queue, output_queue, progress_queue):
        """
        Main loop for the worker process.

        Processes conversion requests from the input queue and sends
        results to the output queue.
        """
        logging.basicConfig(
            level=logging.INFO,
            format='[MP4_CONVERSION_WORKER] %(levelname)s: %(message)s'
        )
        logger = logging.getLogger('mp4_conversion_worker')

        try:
            logger.info(f'MP4 conversion worker process started with PID: {os.getpid()}')
            logger.info('Worker is ready and waiting for requests')

            request_count = 0
            last_log_time = time.time()

            while True:
                try:
                    current_time = time.time()
                    if current_time - last_log_time > 30.0:
                        logger.info(
                            f'Worker still alive, processed {request_count} requests so far'
                        )
                        last_log_time = current_time

                    try:
                        data = input_queue.get(timeout=1.0)

                        if data is None:
                            logger.info('Received shutdown signal')
                            break

                        request_count += 1
                        logger.info(f'*** Received MP4 conversion request #{request_count} ***')

                        dataset_path = data.get('dataset_path')
                        robot_type = data.get('robot_type', '')
                        robot_config_path = data.get('robot_config_path', '')
                        source_folders = data.get('source_folders', [])

                        logger.info(f'Processing chained conversion for: {dataset_path}')

                        is_merge_mode = len(source_folders) > 0

                        # Stage 0: Merge episodes (only in merge mode)
                        if is_merge_mode:
                            logger.info('=== Stage 0: Merging episodes ===')
                            success, message = Mp4ConversionWorker._merge_episodes(
                                source_folders, dataset_path,
                                progress_queue, logger)
                            if not success:
                                output_queue.put(('error', f'[Merge] {message}'))
                                continue
                            logger.info(f'Merge completed: {message}')

                        # Stage 1: MP4 conversion
                        logger.info('=== Stage 1/3: Converting to MP4 ===')
                        success, message = Mp4ConversionWorker._convert_dataset(
                            dataset_path=dataset_path,
                            progress_queue=progress_queue,
                            logger=logger,
                            is_merge_mode=is_merge_mode
                        )
                        if not success:
                            logger.error(f'Stage 1 failed: {message}')
                            output_queue.put(('error', f'[Stage 1/3 MP4] {message}'))
                            continue

                        # Stage 2: LeRobot v2.1 conversion
                        logger.info('=== Stage 2/3: Converting to LeRobot v2.1 ===')
                        success, message = Mp4ConversionWorker._convert_to_lerobot_v21(
                            dataset_path=dataset_path,
                            robot_config_path=robot_config_path,
                            progress_queue=progress_queue,
                            logger=logger,
                            is_merge_mode=is_merge_mode
                        )
                        if not success:
                            logger.error(f'Stage 2 failed: {message}')
                            output_queue.put(('error', f'[Stage 2/3 LeRobot v2.1] {message}'))
                            continue

                        # Stage 3: LeRobot v3.0 conversion
                        logger.info('=== Stage 3/3: Converting to LeRobot v3.0 ===')
                        success, message = Mp4ConversionWorker._convert_to_lerobot_v30(
                            dataset_path=dataset_path,
                            robot_config_path=robot_config_path,
                            progress_queue=progress_queue,
                            logger=logger,
                            is_merge_mode=is_merge_mode
                        )
                        if not success:
                            logger.error(f'Stage 3 failed: {message}')
                            output_queue.put(('error', f'[Stage 3/3 LeRobot v3.0] {message}'))
                            continue

                        # Cleanup intermediate Stage 1 outputs ({episode}_converted).
                        try:
                            import shutil as _shutil
                            removed = 0
                            for d in Path(dataset_path).iterdir():
                                if d.is_dir() and d.name.endswith('_converted'):
                                    _shutil.rmtree(str(d))
                                    removed += 1
                            if removed:
                                logger.info(
                                    f'Cleaned up {removed} *_converted '
                                    f'intermediate folder(s) under {dataset_path}'
                                )
                        except Exception as cleanup_err:
                            logger.warning(
                                f'Failed to remove *_converted folders: {cleanup_err}'
                            )

                        # Make the lerobot outputs world-readable. The v3.0
                        # converter runs inside the lerobot container as root
                        # with a restrictive umask (0o077), which leaves files
                        # unreadable from the host filesystem (e.g. VSCode).
                        try:
                            import os as _os
                            v21_dir = Path(str(dataset_path) + '_lerobot_v21')
                            v30_dir = Path(str(dataset_path) + '_lerobot_v30')
                            for root_dir in (v21_dir, v30_dir):
                                if not root_dir.exists():
                                    continue
                                for p in root_dir.rglob('*'):
                                    try:
                                        if p.is_dir():
                                            _os.chmod(p, 0o755)
                                        else:
                                            _os.chmod(p, 0o644)
                                    except Exception:
                                        pass
                                _os.chmod(root_dir, 0o755)
                        except Exception as chmod_err:
                            logger.warning(
                                f'Failed to relax permissions on outputs: {chmod_err}'
                            )

                        logger.info(f'All stages completed for: {dataset_path}')
                        output_queue.put(('success', 'All stages completed successfully'))

                    except queue.Empty:
                        continue

                except Exception as e:
                    error_msg = f'MP4 conversion operation error: {str(e)}'
                    logger.error(error_msg)
                    import traceback
                    logger.error(f'Traceback: {traceback.format_exc()}')
                    output_queue.put(('error', error_msg))

        except Exception as e:
            error_msg = f'MP4 conversion worker initialization error: {str(e)}'
            logger.error(error_msg)
            import traceback
            logger.error(f'Traceback: {traceback.format_exc()}')
            output_queue.put(('error', error_msg))

        logger.info('MP4 conversion worker process shutting down')

    @staticmethod
    def _merge_episodes(
        source_folders: List[str],
        output_path: str,
        progress_queue: multiprocessing.Queue,
        logger: logging.Logger
    ) -> tuple:
        """
        Merge episodes from multiple source folders using symlinks.

        Creates symlinks in output_path with consecutive episode numbers
        pointing to the original episode directories.

        Args:
            source_folders: List of source folder paths.
            output_path: Path where merged symlinks will be created.
            progress_queue: Queue for progress updates.
            logger: Logger instance.

        Returns:
            Tuple of (success: bool, message: str).
        """
        try:
            output_path = Path(output_path)
            output_path.mkdir(parents=True, exist_ok=True)

            episode_counter = 0
            for src_folder in source_folders:
                src_path = Path(src_folder)
                if not src_path.exists():
                    return False, f'Source folder not found: {src_path}'

                episode_dirs = sorted(
                    [d for d in src_path.iterdir()
                     if d.is_dir() and d.name.isdigit()],
                    key=lambda d: int(d.name)
                )

                for ep_dir in episode_dirs:
                    link_path = output_path / str(episode_counter)
                    link_path.symlink_to(ep_dir.resolve())
                    logger.info(f'Symlink: {ep_dir} -> {link_path}')
                    episode_counter += 1

            # Report merge completion (0% ~ 5%)
            progress_queue.put({
                'current': episode_counter,
                'total': episode_counter,
                'percentage': 5.0,
                'current_episode': '',
                'dataset_path': str(output_path),
                'stage': 'merge'
            })

            return True, (
                f'Merged {episode_counter} episodes '
                f'from {len(source_folders)} folders'
            )

        except Exception as e:
            import traceback
            logger.error(f'Merge error: {traceback.format_exc()}')
            return False, f'Merge error: {str(e)}'

    @staticmethod
    def _convert_dataset(
        dataset_path: str,
        progress_queue: multiprocessing.Queue,
        logger: logging.Logger,
        is_merge_mode: bool = False
    ) -> tuple:
        """
        Convert all episodes in a dataset to MP4 format.

        Args:
            dataset_path: Path to the dataset directory.
            progress_queue: Queue for progress updates.
            logger: Logger instance.

        Returns:
            Tuple of (success: bool, message: str).
        """
        try:
            dataset_path = Path(dataset_path)
            if not dataset_path.exists():
                return False, f'Dataset path does not exist: {dataset_path}'

            # Find all episode directories (numeric folders)
            episode_dirs = sorted([
                d for d in dataset_path.iterdir()
                if d.is_dir() and d.name.isdigit()
            ])

            if not episode_dirs:
                return False, f'No episode directories found in {dataset_path}'

            total_episodes = len(episode_dirs)
            logger.info(f'Found {total_episodes} episodes to convert')

            converted_count = 0
            failed_episodes = []

            # Progress range depends on mode:
            #   merge mode: Stage 1 = 5% ~ 35%
            #   single mode: Stage 1 = 0% ~ 33%
            progress_start = 5.0 if is_merge_mode else 0.0
            progress_end = 35.0 if is_merge_mode else 33.0

            # Parallel episode conversion using ProcessPoolExecutor
            # Each worker creates its own RosbagToMp4Converter (stateless, picklable args)
            # max_workers=min(4, total_episodes): 2 episodes × 4 cameras = 8 NVENC sessions
            from concurrent.futures import ProcessPoolExecutor, as_completed

            max_workers = min(4, total_episodes)
            logger.info(
                f'Starting parallel MP4 conversion with {max_workers} workers'
            )

            # Report initial progress
            progress_queue.put({
                'current': 0,
                'total': total_episodes,
                'percentage': progress_start,
                'current_episode': '',
                'dataset_path': str(dataset_path),
                'stage': 'mp4'
            })

            # Build episode task list
            episode_tasks = []
            for episode_dir in episode_dirs:
                episode_id = episode_dir.name
                output_dir = dataset_path / f'{episode_id}_converted'
                episode_tasks.append((episode_dir, output_dir, episode_id))

            completed_count = 0
            with ProcessPoolExecutor(max_workers=max_workers) as executor:
                futures = {}
                for episode_dir, output_dir, episode_id in episode_tasks:
                    future = executor.submit(
                        _convert_single_episode_worker,
                        episode_dir, output_dir,
                        15, True, True  # fps, use_hw, enable_smoothing
                    )
                    futures[future] = episode_id

                for future in as_completed(futures):
                    episode_id = futures[future]
                    completed_count += 1

                    # Update progress
                    stage_progress = completed_count / total_episodes
                    overall_progress = (
                        progress_start
                        + stage_progress * (progress_end - progress_start)
                    )
                    progress_queue.put({
                        'current': completed_count,
                        'total': total_episodes,
                        'percentage': overall_progress,
                        'current_episode': episode_id,
                        'dataset_path': str(dataset_path),
                        'stage': 'mp4'
                    })

                    try:
                        _, success, _ = future.result()
                        if success:
                            converted_count += 1
                            logger.info(
                                f'Episode {episode_id} converted successfully '
                                f'({completed_count}/{total_episodes})'
                            )
                        else:
                            failed_episodes.append(episode_id)
                            logger.warning(
                                f'Episode {episode_id} conversion had issues'
                            )
                    except Exception as e:
                        failed_episodes.append(episode_id)
                        logger.error(
                            f'Error converting episode {episode_id}: {str(e)}'
                        )

            # Final progress update for Stage 1
            progress_data = {
                'current': total_episodes,
                'total': total_episodes,
                'percentage': progress_end,
                'current_episode': '',
                'dataset_path': str(dataset_path),
                'stage': 'mp4'
            }
            progress_queue.put(progress_data)

            # Build result message
            if converted_count == total_episodes:
                return True, (
                    f'Successfully converted all {total_episodes} episodes '
                    f'in {dataset_path}'
                )
            elif converted_count > 0:
                return True, (
                    f'Converted {converted_count}/{total_episodes} episodes. '
                    f'Failed episodes: {", ".join(failed_episodes)}'
                )
            else:
                return False, (
                    f'Failed to convert any episodes. '
                    f'Failed episodes: {", ".join(failed_episodes)}'
                )

        except Exception as e:
            import traceback
            logger.error(f'Conversion error: {traceback.format_exc()}')
            return False, f'Conversion error: {str(e)}'

    @staticmethod
    def _convert_to_lerobot_v21(
        dataset_path: str,
        robot_config_path: str,
        progress_queue: multiprocessing.Queue,
        logger: logging.Logger,
        is_merge_mode: bool = False
    ) -> tuple:
        """
        Stage 2: Convert _converted folders to LeRobot v2.1 format.

        Args:
            dataset_path: Path to the dataset directory.
            robot_config_path: Path to robot config YAML file.
            progress_queue: Queue for progress updates.
            logger: Logger instance.
            is_merge_mode: Whether running in merge mode (affects progress ranges).

        Returns:
            Tuple of (success: bool, message: str).
        """
        try:
            from physical_ai_server.data_processing.rosbag_to_lerobot_converter import (
                ConversionConfig,
                RosbagToLerobotConverter
            )
        except ImportError as e:
            return False, f'Failed to import LeRobot v2.1 converter: {str(e)}'

        try:
            dataset_path = Path(dataset_path)
            output_dir = Path(str(dataset_path) + '_lerobot_v21')
            repo_id = dataset_path.name

            # Progress range: merge mode = 35%~68%, single mode = 33%~66%
            progress_start = 35.0 if is_merge_mode else 33.0
            progress_end = 68.0 if is_merge_mode else 66.0

            # Collect _converted folders as bag_paths
            bag_paths = sorted([
                d for d in dataset_path.iterdir()
                if d.is_dir() and d.name.endswith('_converted')
            ])

            if not bag_paths:
                return False, f'No _converted folders found in {dataset_path}'

            logger.info(
                f'Found {len(bag_paths)} converted episodes for LeRobot v2.1'
            )

            # Report stage start
            progress_queue.put({
                'current': 0,
                'total': len(bag_paths),
                'percentage': progress_start,
                'current_episode': '',
                'dataset_path': str(dataset_path),
                'stage': 'lerobot_v21'
            })

            config = ConversionConfig(
                repo_id=repo_id,
                output_dir=output_dir,
                robot_config_path=robot_config_path if robot_config_path else None,
            )

            converter = RosbagToLerobotConverter(config, logger)
            success = converter.convert_multiple_rosbags(bag_paths)

            # Report stage completion
            progress_queue.put({
                'current': len(bag_paths),
                'total': len(bag_paths),
                'percentage': progress_end,
                'current_episode': '',
                'dataset_path': str(dataset_path),
                'stage': 'lerobot_v21'
            })

            if success:
                return True, f'LeRobot v2.1 conversion completed: {output_dir}'
            else:
                return False, f'LeRobot v2.1 conversion failed for {dataset_path}'

        except Exception as e:
            import traceback
            logger.error(f'LeRobot v2.1 conversion error: {traceback.format_exc()}')
            return False, f'LeRobot v2.1 conversion error: {str(e)}'

    @staticmethod
    def _convert_to_lerobot_v30(
        dataset_path: str,
        robot_config_path: str,
        progress_queue: multiprocessing.Queue,
        logger: logging.Logger,
        is_merge_mode: bool = False
    ) -> tuple:
        """
        Stage 3: Convert LeRobot v2.1 dataset to v3.0 using lerobot's built-in converter.

        Runs the conversion via subprocess calling docker exec on the lerobot_server
        container, which has lerobot installed with the convert_dataset_v21_to_v30 script.
        Both containers share /workspace so the v2.1 output is directly accessible.

        Args:
            dataset_path: Path to the original dataset directory.
            robot_config_path: Path to robot config YAML file (unused for v3.0).
            progress_queue: Queue for progress updates.
            logger: Logger instance.
            is_merge_mode: Whether running in merge mode (affects progress ranges).

        Returns:
            Tuple of (success: bool, message: str).
        """
        import subprocess

        try:
            dataset_path = Path(dataset_path)
            v21_dir = Path(str(dataset_path) + '_lerobot_v21')
            repo_id = dataset_path.name

            # Progress range: merge mode = 68%~100%, single mode = 66%~100%
            progress_start = 68.0 if is_merge_mode else 66.0

            if not v21_dir.exists():
                return False, f'LeRobot v2.1 output not found: {v21_dir}'

            logger.info(f'Converting LeRobot v2.1 -> v3.0 via lerobot container')

            # Report stage start
            progress_queue.put({
                'current': 0,
                'total': 1,
                'percentage': progress_start,
                'current_episode': '',
                'dataset_path': str(dataset_path),
                'stage': 'lerobot_v30'
            })

            # The v2.1 dataset is at /workspace/rosbag2/{name}_lerobot_v21/{repo_id}
            # lerobot's convert_dataset expects --root to be the parent dir
            # and --repo-id to be the folder name inside it.
            # But our v2.1 output is directly at v21_dir (no repo_id subfolder).
            # We need to create a wrapper dir structure:
            #   {v21_dir_parent}/{repo_id}_lerobot_v21/{repo_id}/  <- actual data
            # OR use the script's local path logic.
            #
            # The convert_dataset function does:
            #   root = Path(root) / repo_id
            # So we set --root to v21_dir's parent and --repo-id to v21_dir's name.

            root_parent = str(v21_dir.parent)
            folder_name = v21_dir.name  # e.g., ffw_sg2_rev1_task_2602101048_lerobot_v21

            cmd = [
                'docker', 'exec', 'lerobot_server',
                '/lerobot/.venv/bin/python', '-m',
                'lerobot.datasets.v30.convert_dataset_v21_to_v30',
                f'--repo-id={folder_name}',
                f'--root={root_parent}',
                '--push-to-hub=false',
                '--force-conversion',
            ]

            logger.info(f'Running: {" ".join(cmd)}')

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=600
            )

            logger.info(f'v3.0 converter stdout:\n{result.stdout}')
            if result.stderr:
                logger.info(f'v3.0 converter stderr:\n{result.stderr}')

            # Report stage completion
            progress_queue.put({
                'current': 1,
                'total': 1,
                'percentage': 100.0,
                'current_episode': '',
                'dataset_path': str(dataset_path),
                'stage': 'lerobot_v30'
            })

            if result.returncode == 0:
                # The converter renames in-place:
                #   original -> {name}_old, {name}_v30 -> original
                # So the v2.1 dir now contains v3.0 data.
                # Rename to make it clear:
                #   {name}_lerobot_v21 (now v3.0) -> {name}_lerobot_v30
                #   {name}_lerobot_v21_old (original v2.1) -> {name}_lerobot_v21
                v21_old = Path(str(v21_dir) + '_old')
                v30_dir = Path(str(dataset_path) + '_lerobot_v30')

                if v30_dir.exists():
                    import shutil
                    shutil.rmtree(str(v30_dir))

                # v21_dir now has v3.0 data, rename it
                v21_dir.rename(v30_dir)
                # v21_old has original v2.1 data, restore it
                if v21_old.exists():
                    v21_old.rename(v21_dir)

                return True, f'LeRobot v3.0 conversion completed: {v30_dir}'
            else:
                return False, (
                    f'LeRobot v3.0 conversion failed (exit code {result.returncode}): '
                    f'{result.stderr or result.stdout}'
                )

        except subprocess.TimeoutExpired:
            return False, 'LeRobot v3.0 conversion timed out (600s)'
        except Exception as e:
            import traceback
            logger.error(f'LeRobot v3.0 conversion error: {traceback.format_exc()}')
            return False, f'LeRobot v3.0 conversion error: {str(e)}'
