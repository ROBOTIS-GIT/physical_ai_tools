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

import logging
import multiprocessing
import os
import queue
import time

from physical_ai_server.data_processing.data_manager import DataManager


class HfApiWorker:

    def __init__(self):
        self.input_queue = multiprocessing.Queue()
        self.output_queue = multiprocessing.Queue()
        self.progress_queue = multiprocessing.Queue()
        self.process = None
        self.logger = logging.getLogger('HfApiWorker')

        # Task state management
        self.is_processing = False
        self.current_task = None
        self.start_time = None

        # Progress tracking
        self.current_progress = {
            'current': 0,
            'total': 0,
            'percentage': 0.0,
            'is_downloading': False,
            'repo_id': '',
            'repo_type': ''
        }
        self.last_logged_current_progress = -1  # Track last logged current value

        # Basic config for the main process logger
        logging.basicConfig(
            level=logging.INFO,
            format='%(name)s - %(levelname)s - %(message)s')

    def start(self):
        if self.process and self.process.is_alive():
            self.logger.warning('HF API worker process is already running.')
            return False

        try:
            self.logger.info('Starting HF API worker process...')

            self.process = multiprocessing.Process(
                target=self._worker_process_loop,
                args=(
                    self.input_queue,
                    self.output_queue,
                    self.progress_queue
                )
            )

            self.process.start()
            self.logger.info(f'HF API worker process started with PID: {self.process.pid}')
            return True

        except Exception as e:
            self.logger.error(f'Failed to start HF API worker: {str(e)}')
            return False

    def stop(self, timeout=3.0):
        if not self.is_alive():
            self.logger.info('HF API worker process is not running or already stopped.')
            return

        try:
            self.logger.info('Sending shutdown signal to HF API worker...')
            # Send graceful shutdown signal first
            try:
                self.input_queue.put_nowait(None)
            except Exception:
                # If queue is full/unavailable, proceed to force terminate
                pass

            # Give a very short grace period, then force terminate if still alive
            grace_timeout = min(max(timeout, 0.0), 1.0)
            if grace_timeout > 0:
                self.process.join(grace_timeout)

            if self.process.is_alive():
                self.logger.warning(
                    'HF API worker did not terminate gracefully. Forcing termination now.')
                self.process.kill()
                # Ensure the process is reaped promptly
                self.process.join(1.0)
        except Exception as e:
            self.logger.error(f'Error stopping HF API worker process: {e}')
        finally:
            self.process = None
            # Reset state
            self.is_processing = False
            self.current_task = None
            self.start_time = None

    def is_alive(self):
        return self.process and self.process.is_alive()

    def send_request(self, request_data):
        if self.is_alive():
            self.input_queue.put(request_data)
            self.is_processing = True
            self.current_task = request_data
            self.start_time = time.time()
            return True
        else:
            self.logger.error('Cannot send request, HF API worker process is not running.')
            return False

    def get_result(self, block=False, timeout=0.1):
        try:
            return self.output_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return None

    def check_task_status(self) -> dict:
        """Check the current task status and return appropriate message."""
        result = {
            'operation': '',
            'status': 'Idle',
            'repo_id': '',
            'local_path': '',
            'message': '',
            'progress': {
                'current': 0,
                'total': 0,
                'percentage': 0.0,
            }
        }

        mode = None

        if not self.is_alive():
            self.logger.error('HF API worker process died')
            result['status'] = 'Failed'

        if not self.is_processing:
            result['message'] = 'HF API worker process died'
            return result
            result['status'] = 'Idle'
            return result

        try:
            if self.current_task:
                mode = self.current_task.get('mode', 'Processing')
                result['repo_id'] = self.current_task.get('repo_id', '')
                result['local_path'] = self.current_task.get('local_path', '')

            # Check for download progress
            if mode == 'download' or mode == 'upload':
                # Check for progress updates from worker process
                self.current_progress = self.get_progress_from_progress_queue()
                current = self.current_progress.get('current', 0)
                total = self.current_progress.get('total', 0)
                percentage = self.current_progress.get('percentage', 0.0)
                result['progress']['current'] = current
                result['progress']['total'] = total
                result['progress']['percentage'] = percentage

                # Only log when current value changes
                if current != self.last_logged_current_progress:
                    # self.logger.info(f'{mode.capitalize()} {current}/{total} ({percentage}%)')
                    self.last_logged_current_progress = current

            # Check for task result
            task_result = self.get_result(block=False, timeout=0.1)
            if task_result:
                status, message = task_result
                if status == 'success':
                    log_message = f'✅ HF API task completed successfully:\n{message}'
                    self.logger.info(log_message)
                    self.is_processing = False
                    self.current_task = None

                    result['operation'] = mode
                    result['status'] = 'Success'
                    result['message'] = log_message
                    return result
                elif status == 'error':
                    log_message = f'❌ HF API task failed:\n{message}'
                    self.logger.error(log_message)
                    self.is_processing = False
                    self.current_task = None

                    result['operation'] = mode
                    result['status'] = 'Failed'
                    result['message'] = log_message
                    return result

            # Still processing - return appropriate status message
            if mode:
                if mode == 'upload':
                    result['operation'] = mode
                    result['status'] = 'Uploading'
                    return result
                elif mode == 'download':
                    result['operation'] = mode
                    result['status'] = 'Downloading'
                    return result
                elif mode == 'delete':
                    result['operation'] = mode
                    result['status'] = 'Deleting'
                    return result
                elif mode in ['get_dataset_list', 'get_model_list']:
                    result['operation'] = mode
                    result['status'] = 'Fetching'
                    return result
                else:
                    result['operation'] = 'Unknown'
                    result['status'] = 'Processing'
                    return result

            result['status'] = 'Processing'
            return result

        except Exception as e:
            log_message = f'Error checking HF API task status: {str(e)}'
            self.logger.error(log_message)
            result['operation'] = mode if mode else 'Unknown'
            result['status'] = 'Failed'
            result['message'] = log_message
            return result

    def is_busy(self):
        """Check if the worker is currently processing a task."""
        return self.is_processing

    def get_progress_from_progress_queue(self):
        """Get the latest progress information from worker process and clear queue."""
        latest_progress = None
        try:
            # Drain the queue and keep only the latest progress data
            while True:
                try:
                    latest_progress = self.progress_queue.get(block=False, timeout=0.01)
                except queue.Empty:
                    break
        except Exception as e:
            self.logger.error(f'Error updating progress from worker: {e}')

        # Return the latest progress or current progress if no new data
        return latest_progress if latest_progress else self.current_progress

    @staticmethod
    def _worker_process_loop(input_queue, output_queue, progress_queue):
        # Set up logging for the worker process
        logging.basicConfig(
            level=logging.INFO,
            format='[HF_API_WORKER] %(levelname)s: %(message)s')
        logger = logging.getLogger('hf_api_worker')

        try:
            logger.info(f'HF API worker process started with PID: {os.getpid()}')
            logger.info('Worker is ready and waiting for requests')

            # Set progress queue for DataManager
            DataManager.set_progress_queue(progress_queue)

            request_count = 0
            last_log_time = time.time()

            while True:
                try:
                    # Log periodic status
                    current_time = time.time()
                    if current_time - last_log_time > 30.0:  # Log every 30 seconds
                        msg = f'Worker still alive, processed {request_count} requests so far'
                        logger.info(msg)
                        logger.info(f'Input queue size: {input_queue.qsize()}')
                        last_log_time = current_time

                    # Check for new requests
                    try:
                        data = input_queue.get(timeout=1.0)

                        if data is None:  # Shutdown signal
                            logger.info('Received shutdown signal')
                            break

                        request_count += 1
                        logger.info(f'*** Received HF API request #{request_count} ***')

                        mode = data.get('mode')
                        repo_id = data.get('repo_id')
                        repo_type = data.get('repo_type')
                        local_dir = data.get('local_dir')
                        author = data.get('author')

                        logger.info(f'Processing {mode} request for repo: {repo_id}')

                        # Process the request based on mode
                        if mode == 'upload':
                            logger.info(f'Starting upload for repo: {repo_id}')
                            result = DataManager.upload_huggingface_repo(
                                repo_id=repo_id,
                                repo_type=repo_type,
                                local_dir=local_dir
                            )
                            if result:
                                message = f'Uploaded Hugging Face repo: {repo_id}'
                                logger.info(f'✅ Upload completed: {repo_id}')
                                output_queue.put(('success', message))
                            else:
                                message = (f'Failed to upload Hugging Face repo'
                                           f'\n{repo_id}, '
                                           f'\nPlease check the repo ID and try again.')
                                logger.error(f'❌ Upload failed: {repo_id}')
                                output_queue.put(('error', message))

                        elif mode == 'download':
                            logger.info(f'Starting download for repo: {repo_id}')
                            result = DataManager.download_huggingface_repo(
                                repo_id=repo_id,
                                repo_type=repo_type
                            )
                            if result:
                                message = f'Downloaded Hugging Face repo: {repo_id}'
                                logger.info(f'✅ Download completed: {repo_id}')
                                output_queue.put(('success', message))
                            else:
                                message = (f'Failed to download Hugging Face repo:'
                                           f'\n{repo_id}, '
                                           f'\nPlease check the repo ID and try again.')
                                logger.error(f'❌ Download failed: {repo_id}')
                                output_queue.put(('error', message))

                        elif mode == 'delete':
                            logger.info(f'Starting delete for repo: {repo_id}')
                            DataManager.delete_huggingface_repo(
                                repo_id=repo_id,
                                repo_type=repo_type
                            )
                            message = f'Deleted Hugging Face repo: {repo_id}'
                            logger.info(f'✅ Delete completed: {repo_id}')
                            output_queue.put(('success', message))

                        elif mode == 'get_dataset_list':
                            logger.info(f'Starting dataset list fetch for author: {author}')
                            DataManager.get_huggingface_repo_list(
                                author=author,
                                data_type='dataset'
                            )
                            message = f'Got dataset list for author: {author}'
                            logger.info(f'✅ Dataset list fetch completed: {author}')
                            output_queue.put(('success', message))

                        elif mode == 'get_model_list':
                            logger.info(f'Starting model list fetch for author: {author}')
                            DataManager.get_huggingface_repo_list(
                                author=author,
                                data_type='model'
                            )
                            message = f'Got model list for author: {author}'
                            logger.info(f'✅ Model list fetch completed: {author}')
                            output_queue.put(('success', message))

                        else:
                            error_msg = f'Unknown mode: {mode}'
                            logger.error(error_msg)
                            output_queue.put(('error', error_msg))

                    except queue.Empty:
                        continue

                except Exception as e:
                    error_msg = f'HF API operation error: {str(e)}'
                    logger.error(error_msg)
                    import traceback
                    logger.error(f'Traceback: {traceback.format_exc()}')
                    output_queue.put(('error', error_msg))

        except Exception as e:
            error_msg = f'HF API worker initialization error: {str(e)}'
            logger.error(error_msg)
            import traceback
            logger.error(f'Traceback: {traceback.format_exc()}')
            output_queue.put(('error', error_msg))

        logger.info('HF API worker process shutting down')
