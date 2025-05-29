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
from pathlib import Path
import subprocess
import time
from typing import List, Optional, Union

import numpy as np


class VideoBufferEncoder:

    def __init__(
        self,
        fps: int = 30,
        vcodec: str = 'libx264',
        pix_fmt: str = 'yuv420p',
        g: Optional[int] = 2,
        crf: Optional[int] = 23,
        fast_decode: int = 0,
    ):
        self.buffer = []
        self.fps = fps
        self.vcodec = vcodec
        self.pix_fmt = pix_fmt
        self.g = g
        self.crf = crf
        self.fast_decode = fast_decode

    def set_buffer(self, frames: List[np.ndarray]) -> None:
        self.buffer = frames

    def clear_buffer(self) -> None:
        self.buffer = []

    def encode_video(self, video_path: Union[str, Path]) -> None:
        raise NotImplementedError('Must be implemented by subclasses')


class FFmpegBufferEncoder(VideoBufferEncoder):

    def __init__(
        self,
        *args,
        chunk_size: int = 100,
        preset: str = 'medium',
        crf: Optional[int] = 23,
        clear_after_encode: bool = True,
        **kwargs
    ):

        # Override default crf with the one provided in the constructor arguments
        kwargs['crf'] = crf
        super().__init__(*args, **kwargs)
        self.process = None
        self.chunk_size = chunk_size
        self.preset = preset
        self.clear_after_encode = clear_after_encode

        # Encoding status tracking
        self.is_encoding = False
        self.encoding_completed = False
        self.encoding_started_at = None
        self.encoding_finished_at = None
        self.total_frames_encoded = 0
        self.total_chunks_encoded = 0
        self.current_chunk = 0
        self.output_path = None

    def is_encoding_completed(self) -> bool:
        return self.encoding_completed

    def get_encoding_status(self) -> dict:
        status = {
            'is_encoding': self.is_encoding,
            'encoding_completed': self.encoding_completed,
            'total_frames': len(self.buffer),
            'total_frames_encoded': self.total_frames_encoded,
            'total_chunks': (
                len(self.buffer) + self.chunk_size - 1) // self.chunk_size if self.buffer else 0,
            'chunks_encoded': self.total_chunks_encoded,
            'current_chunk': self.current_chunk,
            'progress_percentage': (
                self.total_frames_encoded / len(self.buffer) * 100) if self.buffer else 0,
            'output_path': str(self.output_path) if self.output_path else None,
        }

        if self.encoding_started_at:
            status['started_at'] = self.encoding_started_at
            status['elapsed_time'] = (
                self.encoding_finished_at or time.time()) - self.encoding_started_at
            if self.encoding_finished_at:
                status['finished_at'] = self.encoding_finished_at
                status['encoding_time'] = self.encoding_finished_at - self.encoding_started_at
                if self.output_path and self.output_path.exists():
                    status['file_size'] = os.path.getsize(self.output_path)
                    status['file_size_kb'] = status['file_size'] / 1024

                if self.total_frames_encoded > 0:
                    status['fps'] = self.total_frames_encoded / status['encoding_time']

        return status

    def encode_video(self, video_path: Union[str, Path]) -> None:
        if not self.buffer:
            raise ValueError('No frames in buffer to encode')

        # Reset encoding status
        self.is_encoding = True
        self.encoding_completed = False
        self.encoding_started_at = time.time()
        self.encoding_finished_at = None
        self.total_frames_encoded = 0
        self.total_chunks_encoded = 0
        self.current_chunk = 0

        video_path = Path(video_path)
        self.output_path = video_path
        video_path.parent.mkdir(parents=True, exist_ok=True)

        # Get the size of the first image
        height, width = self.buffer[0].shape[:2]
        total_frames = len(self.buffer)

        # Directly construct FFmpeg command
        cmd = ['ffmpeg']
        cmd.extend(['-f', 'rawvideo'])
        cmd.extend(['-vcodec', 'rawvideo'])
        cmd.extend(['-s', f'{width}x{height}'])
        cmd.extend(['-pix_fmt', 'rgb24'])
        cmd.extend(['-r', str(self.fps)])
        cmd.extend(['-i', '-'])
        cmd.extend(['-an'])
        cmd.extend(['-vcodec', self.vcodec])
        cmd.extend(['-pix_fmt', self.pix_fmt])
        cmd.extend(['-preset', self.preset])

        if self.g is not None:
            cmd.extend(['-g', str(self.g)])

        if self.crf is not None:
            cmd.extend(['-crf', str(self.crf)])

        if self.fast_decode:
            if self.vcodec == 'libsvtav1':
                cmd.extend(['-svtav1-params', f'fast-decode={self.fast_decode}'])
            else:
                cmd.extend(['-tune', 'fastdecode'])

        cmd.extend(['-loglevel', 'warning'])  # Use warning level for more verbose logs
        cmd.extend(['-y'])
        cmd.append(str(video_path))

        # Start FFmpeg process
        try:
            self.process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=10**8  # Large buffer size
            )
            # Process frames in chunks to avoid buffer overflow
            for i in range(0, total_frames, self.chunk_size):
                self.current_chunk = i // self.chunk_size + 1
                chunk = self.buffer[i:i+self.chunk_size]

                # Send each frame in the chunk to FFmpeg
                for j, frame in enumerate(chunk):
                    try:
                        self.process.stdin.write(frame.tobytes())
                        self.total_frames_encoded += 1
                    except BrokenPipeError as e:
                        stderr_output = self.process.stderr.read().decode()
                        self.is_encoding = False
                        raise RuntimeError(
                            f'Error in FFmpeg stream processing: {stderr_output}') from e

                # Flush after each chunk
                self.process.stdin.flush()
                self.total_chunks_encoded += 1

            # Close input pipe and wait for process to complete
            self.process.stdin.close()
            self.process.wait(timeout=60)  # Increased timeout for large videos

            stderr = self.process.stderr.read().decode()
            if self.process.returncode != 0:
                self.is_encoding = False
                raise RuntimeError(
                    f'FFmpeg encoding failed (code: {self.process.returncode}): {stderr}')

            if not video_path.exists():
                self.is_encoding = False
                raise OSError(f'Video encoding did not work. File not found: {video_path}')

            # Update encoding status
            self.is_encoding = False
            self.encoding_completed = True
            self.encoding_finished_at = time.time()

        except Exception as e:
            print(f'Exception occurred: {str(e)}')
            self.is_encoding = False
            self.encoding_completed = False
            if self.process:
                # Force terminate process if running
                self.process.kill()
            raise
        finally:
            # Clean up buffer if configured to do so
            if self.clear_after_encode:
                self.clear_buffer()
