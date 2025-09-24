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
# Author: Kiwoong Park

import io
import re
import sys
import time

from tqdm import tqdm


class HuggingFaceProgressTqdm(tqdm):
    """Custom tqdm class for HuggingFace download progress tracking."""

    def __init__(self, *args, **kwargs):
        # Set default parameters for better visibility
        kwargs.setdefault('desc', 'Downloading')
        kwargs.setdefault('unit', 'files')
        kwargs.setdefault('unit_scale', True)
        kwargs.setdefault('miniters', 1)

        # Extract custom parameters
        self.progress_queue = kwargs.pop('progress_queue', None)
        self.print_progress = kwargs.pop('print_progress', True)

        super().__init__(*args, **kwargs)
        self.last_update = time.time()

    def update(self, n=1):
        super().update(n)

        percentage = (
            round((self.n / self.total * 100), 2)
            if self.total and self.total > 0 else 0.0
        )

        # Create progress info
        progress_info = {
            'current': self.n,
            'total': self.total if self.total else 0,
            'percentage': percentage,
            'is_downloading': True,
        }

        # Send progress to queue if available (for multiprocessing)
        if self.progress_queue:
            try:
                self.progress_queue.put(progress_info, block=False)
            except Exception:
                pass  # Queue might be full, skip this update

        # Print progress every 10 files or every 0.5 seconds, whichever comes first
        current_time = time.time()
        should_print = self.print_progress and (
            (current_time - self.last_update >= 0.5) or  # Time-based
            (self.n % 10 == 0) or  # Every 10 files
            (self.n == self.total)  # Final file
        )

        if should_print:
            if self.total and self.total > 0:
                # Use logging for better visibility in multiprocessing
                import logging
                logger = logging.getLogger('hf_progress')
                progress_msg = f'📥 {self.n}/{self.total} files ({percentage:.1f}%)'
                logger.info(progress_msg)
            self.last_update = current_time


class HuggingFaceLogCapture(io.StringIO):
    """
    Captures HuggingFace upload output and parses progress information.

    This class redirects stdout to capture HuggingFace upload logs while
    simultaneously parsing progress information and sending it to a progress
    queue.
    """

    def __init__(self, progress_queue=None):
        super().__init__()
        self.captured_output = []
        self.progress_queue = progress_queue

    def write(self, text):
        # Filter out noisy timestamp lines
        if text.strip() and self._should_show_line(text.strip()):
            # Print to original stdout (console)
            sys.__stdout__.write(text)
            sys.__stdout__.flush()

        # Capture for logging and parse progress
        if text.strip():  # Only log non-empty lines
            line = text.strip()
            self.captured_output.append(line)

            # Parse progress from HuggingFace output
            self._parse_and_send_progress(line)

            # Use sys.__stdout__ directly to avoid recursion
            if False:
                sys.__stdout__.write(f'[HF Upload Log] {line}\n')
                sys.__stdout__.flush()

        return super().write(text)

    def _should_show_line(self, line):
        """Filter out noisy lines that shouldn't be displayed."""
        # Filter out timestamp separator lines like
        # "---------- 2025-09-17 12:11:00 (0:00:01) ----------"
        if line.startswith('----------') and '(' in line and ')' in line:
            return False

        # Show important lines
        if any(keyword in line for keyword in [
            'Files:', 'Workers:', 'Processing Files', 'New Data Upload',
            'Scanning', 'Found', 'Error', 'Warning', 'Exception'
        ]):
            return True

        # Show file upload progress lines
        if '.mp4:' in line or '.parquet:' in line or 'episode_' in line:
            return True

        # Hide other noise
        return False

    def _parse_and_send_progress(self, line):
        """Parse HuggingFace upload progress and send to progress_queue."""
        if not self.progress_queue:
            return

        try:
            # Parse HF upload progress line format:
            # "Files: hashed X/Y (size) | pre-uploaded: A/B (size) | committed: C/D (size)"
            if 'Files:' in line and 'hashed' in line:
                # Extract all progress indicators
                hashed_match = re.search(r'hashed (\d+)/(\d+)', line)
                pre_uploaded_match = re.search(
                    r'pre-uploaded: (\d+)/(\d+)', line
                )
                committed_match = re.search(r'committed: (\d+)/(\d+)', line)

                if hashed_match:
                    hashed_current = int(hashed_match.group(1))
                    hashed_total = int(hashed_match.group(2))

                    pre_uploaded_current = 0
                    pre_uploaded_total = 0
                    if pre_uploaded_match:
                        pre_uploaded_current = int(pre_uploaded_match.group(1))
                        pre_uploaded_total = int(pre_uploaded_match.group(2))

                    committed_current = 0
                    committed_total = 0
                    if committed_match:
                        committed_current = int(committed_match.group(1))
                        committed_total = int(committed_match.group(2))

                    # Calculate composite progress (weighted average)
                    # Hashing: 30%, Pre-upload: 40%, Commit: 30%
                    if hashed_total > 0:
                        hash_progress = (hashed_current / hashed_total) * 30

                        preupload_progress = 0
                        if pre_uploaded_total > 0:
                            preupload_progress = (
                                pre_uploaded_current / pre_uploaded_total
                            ) * 40

                        commit_progress = 0
                        if committed_total > 0:
                            commit_progress = (
                                committed_current / committed_total
                            ) * 30

                        # Total composite progress
                        composite_percent = (
                            hash_progress + preupload_progress + commit_progress
                        )

                        # Use total files as the consistent total (usually hashed_total)
                        total_files = hashed_total

                        # Calculate equivalent current files based on composite progress
                        current_files = int(
                            (composite_percent / 100) * total_files
                        )

                        # Determine current stage for display
                        if (committed_current == committed_total and
                                committed_total > 0):
                            stage = '✅ Upload Complete'
                            current_files = total_files  # Show 100% completion
                        elif committed_current > 0:
                            stage = '📤 Committing'
                        elif pre_uploaded_current > 0:
                            stage = '⬆️ Uploading'
                        else:
                            stage = '🔄 Hashing'

                        progress_data = {
                            'type': 'upload_progress',
                            'current': current_files,
                            'total': total_files,
                            'percentage': composite_percent
                        }

                        self.progress_queue.put(progress_data)
                        print('📊 Upload Progress: '
                              f'{composite_percent:.1f}% - {stage}: '
                              f'{current_files}/{total_files} files')

        except Exception as e:
            # Don't let parsing errors break the upload
            print(f'Progress parsing error: {e}')

    def flush(self):
        sys.__stdout__.flush()
        return super().flush()
