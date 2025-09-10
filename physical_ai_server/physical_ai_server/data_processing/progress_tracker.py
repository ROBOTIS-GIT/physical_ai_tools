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

import time
from tqdm import tqdm


class HuggingFaceProgressTqdm(tqdm):
    """Custom tqdm class for HuggingFace download progress tracking"""

    def __init__(self, *args, **kwargs):
        # Set default parameters for better visibility
        kwargs.setdefault('desc', 'Downloading')
        kwargs.setdefault('unit', 'files')
        kwargs.setdefault('unit_scale', True)
        kwargs.setdefault('miniters', 1)

        # Extract custom parameters
        self.progress_queue = kwargs.pop('progress_queue', None)
        self.print_progress = kwargs.pop('print_progress', False)

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
                progress_msg = (
                    f"[HF Download] Progress: {self.n}/{self.total} "
                    f"files ({percentage}%)"
                )
                print(progress_msg, flush=True)
            else:
                progress_msg = (
                    f"[HF Download] Progress: {self.n} files downloaded"
                )
                print(progress_msg, flush=True)
            self.last_update = current_time
