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
# Author: Dongyun Kim, Kiwoong Park

import threading
import time

from physical_ai_interfaces.msg import HFOperationStatus
from physical_ai_server.data_processing.data_manager import DataManager
from physical_ai_server.timer.timer_manager import TimerManager

from .base_handler import BaseServiceHandler


class HFServiceHandler(BaseServiceHandler):
    """
    Service handler for HuggingFace related operations.

    Manages HuggingFace user authentication, repository operations,
    and status monitoring.
    """

    def __init__(self, node, hf_api_worker, hf_status_publisher):
        """
        Initialize HuggingFace service handler.

        Args:
            node: ROS2 node instance
            hf_api_worker: HfApiWorker instance for background operations
            hf_status_publisher: Publisher for HF operation status
        """
        super().__init__(node)
        self.hf_api_worker = hf_api_worker
        self.hf_status_publisher = hf_status_publisher
        self.hf_cancel_on_progress = False
        self.hf_status_timer = None
        self._hf_idle_count = 0
        self._last_hf_status = {}

    def set_hf_user_callback(self, request, response):
        """
        Register HuggingFace user token.

        Args:
            request: SetHFUser service request containing token
            response: SetHFUser service response

        Returns:
            response: Modified response with registration status
        """
        request_hf_token = request.token
        if DataManager.register_huggingface_token(request_hf_token):
            self.logger.info('Hugging Face user token registered successfully')
            response.user_id_list = DataManager.get_huggingface_user_id()
            return self._create_success_response(
                response,
                'Hugging Face user token registered successfully'
            )
        else:
            self.logger.error('Failed to register Hugging Face user token')
            response.user_id_list = []
            return self._create_error_response(
                response,
                'Failed to register token, Please check your token'
            )

    def get_hf_user_callback(self, request, response):
        """
        Retrieve registered HuggingFace user information.

        Args:
            request: GetHFUser service request
            response: GetHFUser service response

        Returns:
            response: Modified response with user ID list
        """
        user_ids = DataManager.get_huggingface_user_id()
        if user_ids is not None:
            response.user_id_list = user_ids
            self.logger.info(f'Hugging Face user IDs: {user_ids}')
            return self._create_success_response(
                response,
                'Hugging Face user IDs retrieved successfully'
            )
        else:
            self.logger.error('Failed to retrieve Hugging Face user ID')
            response.user_id_list = []
            return self._create_error_response(
                response,
                'Failed to retrieve Hugging Face user ID'
            )

    def control_hf_server_callback(self, request, response):
        """
        Control HuggingFace server operations.

        Handles upload, download, and cancel operations for HF repositories.

        Args:
            request: ControlHfServer service request
            response: ControlHfServer service response

        Returns:
            response: Modified response with operation status
        """
        try:
            mode = request.mode
            repo_id = request.repo_id
            local_dir = request.local_dir
            repo_type = request.repo_type
            author = request.author

            if self.hf_cancel_on_progress:
                return self._create_error_response(
                    response,
                    'HF API Worker is currently canceling'
                )

            if mode == 'cancel':
                # Immediate cleanup - force stop the worker
                try:
                    self.hf_cancel_on_progress = True
                    self._cleanup_hf_api_worker_with_threading()
                    return self._create_success_response(
                        response,
                        'Cancellation started.'
                    )
                except Exception as e:
                    self.logger.error(f'Error during cancel: {e}')
                    return self._create_error_response(
                        response,
                        f'Error during cancel: {e}'
                    )
                finally:
                    self.hf_cancel_on_progress = False

            # Restart HF API Worker if it does not exist or is not running
            if self.hf_api_worker is None or not self.hf_api_worker.is_alive():
                self.logger.info('HF API Worker not running, restarting...')
                self._init_hf_api_worker()

            # Return error if the worker is busy
            if self.hf_api_worker.is_busy():
                self.logger.warning(
                    'HF API Worker is currently busy with another task'
                )
                return self._create_error_response(
                    response,
                    'HF API Worker is currently busy with another task'
                )

            # Prepare request data for the worker
            request_data = {
                'mode': mode,
                'repo_id': repo_id,
                'local_dir': local_dir,
                'repo_type': repo_type,
                'author': author
            }

            # Send request to HF API Worker
            if self.hf_api_worker.send_request(request_data):
                self.logger.info(
                    f'HF API request sent successfully: {mode} for {repo_id}'
                )
                return self._create_success_response(
                    response,
                    f'HF API request started: {mode} for {repo_id}'
                )
            else:
                self.logger.error('Failed to send request to HF API Worker')
                return self._create_error_response(
                    response,
                    'Failed to send request to HF API Worker'
                )

        except Exception as e:
            self.logger.error(f'Error in HF server callback: {str(e)}')
            return self._create_error_response(
                response,
                f'Error in HF server callback: {str(e)}'
            )

    def hf_status_timer_callback(self):
        """
        Timer callback to check HF API Worker status and publish updates.

        Monitors HF operation status and automatically shuts down worker
        after idle period.
        """
        if self.hf_api_worker is None:
            return

        try:
            status = self.hf_api_worker.check_task_status()
            self._publish_hf_operation_status_msg(status)

            # Log status changes (avoid spamming logs)
            last_status = self._last_hf_status.get('status', 'Unknown')
            current_status = status.get('status', 'Unknown')

            if last_status != current_status:
                self.logger.info(
                    f'HF API Status changed: {last_status} -> {current_status}'
                )

            self._last_hf_status = status

            # Idle status count and automatic shutdown
            if status.get('status', 'Unknown') == 'Idle':
                self._hf_idle_count += 1
                if self._hf_idle_count >= 5:
                    self.logger.info(
                        'HF API Worker idle for 5 cycles, shutting down '
                        'worker and timer.'
                    )
                    self._cleanup_hf_api_worker()
            else:
                self._hf_idle_count = 0

        except Exception as e:
            self.logger.error(f'Error in HF status timer callback: {str(e)}')

    def _publish_hf_operation_status_msg(self, status):
        """
        Publish HF operation status message.

        Args:
            status: Dictionary containing status information
        """
        status_msg = HFOperationStatus()
        status_msg.operation = status.get('operation', 'Unknown')
        status_msg.status = status.get('status', 'Unknown')
        status_msg.repo_id = status.get('repo_id', '')
        status_msg.local_path = status.get('local_path', '')
        status_msg.message = status.get('message', '')

        progress_progress = status.get('progress', {})
        status_msg.progress_current = (
            progress_progress.get('current', 0)
        )
        status_msg.progress_total = (
            progress_progress.get('total', 0)
        )
        status_msg.progress_percentage = (
            progress_progress.get('percentage', 0.0)
        )

        self.hf_status_publisher.publish(status_msg)

    def _init_hf_api_worker(self):
        """Initialize HF API Worker and status monitoring timer."""
        from physical_ai_server.data_processing.hf_api_worker import (
            HfApiWorker
        )

        try:
            self.hf_api_worker = HfApiWorker()
            if self.hf_api_worker.start():
                self.logger.info('HF API Worker started successfully')
                self._hf_idle_count = 0

                # Initialize status monitoring timer
                self.hf_status_timer = TimerManager(node=self.node)
                self.hf_status_timer.set_timer(
                    timer_name='hf_status',
                    timer_frequency=2.0,
                    callback_function=self.hf_status_timer_callback
                )
                self.hf_status_timer.start(timer_name='hf_status')
            else:
                self.logger.error('Failed to start HF API Worker')
        except Exception as e:
            self.logger.error(f'Error initializing HF API Worker: {str(e)}')

    def _cleanup_hf_api_worker_with_threading(self):
        """
        Non-blocking cleanup of HF API Worker using threading.

        Starts a separate thread to run cleanup, preventing the main process
        from blocking during shutdown.
        """
        def cleanup_worker_thread():
            """Worker thread to run _cleanup_hf_api_worker."""
            try:
                self._cleanup_hf_api_worker()
            except Exception as e:
                self.logger.error(f'Error in cleanup worker thread: {e}')

        try:
            if self.hf_status_timer is None and self.hf_api_worker is None:
                self.logger.info('No HF API components to cleanup')
                return

            self.logger.info('Starting non-blocking HF API Worker cleanup...')

            # Start cleanup thread
            cleanup_thread = threading.Thread(
                target=cleanup_worker_thread,
                daemon=True
            )
            cleanup_thread.start()

            # Reset references immediately
            self.hf_status_timer = None
            self.hf_api_worker = None

            self.logger.info('HF API Worker cleanup thread started')

            # Publish cancel status messages
            for i in range(3):
                self._publish_hf_operation_status_msg({
                    'status': 'Idle',
                    'operation': 'stop',
                    'repo_id': '',
                    'local_path': '',
                    'message': 'Canceled by stop command',
                    'progress': {
                        'current': 0,
                        'total': 0,
                        'percentage': 0.0,
                    }
                })
                time.sleep(0.5)

        except Exception as e:
            self.logger.error(
                f'Error starting non-blocking HF API Worker cleanup: {str(e)}'
            )
            # Fallback to blocking cleanup if threading fails
            self._cleanup_hf_api_worker()
        finally:
            self.hf_cancel_on_progress = False

    def _cleanup_hf_api_worker(self):
        """Cleanup HF API Worker and related timers."""
        try:
            if self.hf_status_timer is not None:
                self.hf_status_timer.stop(timer_name='hf_status')
                self.hf_status_timer = None

            if self.hf_api_worker is not None:
                self.hf_api_worker.stop()
                self.hf_api_worker = None

            self.logger.info('HF API Worker cleaned up successfully')
        except Exception as e:
            self.logger.error(f'Error cleaning up HF API Worker: {str(e)}')

    def cleanup(self):
        """Cleanup handler resources on shutdown."""
        self._cleanup_hf_api_worker()
