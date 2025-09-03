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
        self.process = None
        self.logger = logging.getLogger('HfApiWorker')

        # Task state management
        self.is_processing = False
        self.current_task = None
        self.start_time = None

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
                    self.output_queue
                )
            )

            self.process.start()
            self.logger.info(f'HF API worker process started with PID: {self.process.pid}')
            return True

        except Exception as e:
            self.logger.error(f'Failed to start HF API worker: {str(e)}')
            return False

    def stop(self, timeout=5.0):
        if not self.is_alive():
            self.logger.info('HF API worker process is not running or already stopped.')
            return

        try:
            self.logger.info('Sending shutdown signal to HF API worker...')
            self.input_queue.put(None)
            self.process.join(timeout)
            if self.process.is_alive():
                self.logger.warning(
                    'HF API worker process did not terminate gracefully. Forcing termination.')
                self.process.terminate()
                self.process.join()
            self.logger.info('HF API worker process stopped.')
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

    def check_task_status(self):
        """Check the current task status and return appropriate message."""
        if not self.is_alive():
            self.logger.error('HF API worker process died')
            return 'Failed'

        if not self.is_processing:
            return 'Idle'

        try:
            # Check for task result
            result = self.get_result(block=False, timeout=0.1)
            if result:
                status, message = result
                if status == 'success':
                    self.logger.info(f'✅ HF API task completed successfully: {message}')
                    self.is_processing = False
                    self.current_task = None
                    return 'Success'
                elif status == 'error':
                    self.logger.error(f'❌ HF API task failed: {message}')
                    self.is_processing = False
                    self.current_task = None
                    return 'Failed'

            # Still processing - return appropriate status message
            if self.current_task:
                mode = self.current_task.get('mode', 'Processing')
                if mode == 'upload':
                    return 'Uploading'
                elif mode == 'download':
                    return 'Downloading'
                elif mode == 'delete':
                    return 'Deleting'
                elif mode in ['get_dataset_list', 'get_model_list']:
                    return 'Fetching'
                else:
                    return 'Processing'

            return 'Processing'

        except Exception as e:
            self.logger.error(f'Error checking HF API task status: {str(e)}')
            return 'Failed'

    def is_busy(self):
        """Check if the worker is currently processing a task."""
        return self.is_processing

    @staticmethod
    def _worker_process_loop(input_queue, output_queue):
        # Set up logging for the worker process
        logging.basicConfig(
            level=logging.INFO,
            format='[HF_API_WORKER] %(levelname)s: %(message)s')
        logger = logging.getLogger('hf_api_worker')

        try:
            logger.info(f'HF API worker process started with PID: {os.getpid()}')
            logger.info('Worker is ready and waiting for requests')

            request_count = 0
            last_log_time = time.time()

            while True:
                try:
                    # Log periodic status
                    current_time = time.time()
                    if current_time - last_log_time > 30.0:  # Log every 30 seconds
                        logger.info(f'Worker still alive, processed {request_count} requests so far')
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
                            DataManager.upload_huggingface_repo(
                                repo_id=repo_id,
                                repo_type=repo_type,
                                local_dir=local_dir
                            )
                            message = f'Uploaded Hugging Face repo: {repo_id}'
                            logger.info(f'✅ Upload completed: {repo_id}')
                            output_queue.put(('success', message))

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
                                message = f'Failed to download Hugging Face repo: {repo_id}, Please check the repo ID and try again.'
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
