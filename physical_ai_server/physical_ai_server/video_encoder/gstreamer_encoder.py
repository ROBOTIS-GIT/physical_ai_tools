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
import time
from typing import List, Optional, Union

import numpy as np
from physical_ai_server.video_encoder.encoder_base import VideoEncoder


try:
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst, GLib
    GSTREAMER_AVAILABLE = True
except ImportError:
    GSTREAMER_AVAILABLE = False


class GStreamerEncoder(VideoEncoder):
    QUALITY_PRESETS = {
        # Resolution: (bitrate_low, bitrate_medium, bitrate_high)
        (640, 360): (800000, 1200000, 1500000),      # 0.8, 1.2, 1.5 Mbps
        (1280, 720): (2500000, 3500000, 5000000),    # 2.5, 3.5, 5 Mbps
        (1920, 1080): (5000000, 8000000, 12000000),  # 5, 8, 12 Mbps
        (3840, 2160): (15000000, 25000000, 40000000)  # 15, 25, 40 Mbps (4K)
    }

    def __init__(
        self,
        *args,
        bitrate: Optional[int] = None,
        preset_level: int = 1,
        quality: str = 'medium',
        clear_after_encode: bool = True,
        **kwargs
    ):
        if not GSTREAMER_AVAILABLE:
            raise ImportError('GStreamer Python bindings not available')

        super().__init__(*args, **kwargs)

        self.preset_level = preset_level
        self.quality = quality
        self.clear_after_encode = clear_after_encode

        # GStreamer-specific properties
        self.pipeline = None
        self.appsrc = None
        self.loop = None
        self.width = None
        self.height = None

        # Encoding status tracking (similar to FFmpegBufferEncoder)
        self.is_encoding = False
        self.encoding_completed = False
        self.encoding_started_at = None
        self.encoding_finished_at = None
        self.total_frames_encoded = 0
        self.output_path = None
        self.encoding_error = None

        # Auto-select bitrate if not specified
        if bitrate is None:
            self.bitrate = None  # Will be calculated when buffer is set
        else:
            self.bitrate = bitrate

        Gst.init(None)

    def set_buffer(self, frames: List[np.ndarray]) -> None:
        super().set_buffer(frames)

        if self.buffer:
            # Extract dimensions from first frame
            height, width = self.buffer[0].shape[:2]
            self.height = height
            self.width = width

            # Auto-calculate bitrate if not set
            if self.bitrate is None:
                self.bitrate = self._get_optimal_bitrate(width, height, self.quality)

    def _get_optimal_bitrate(self, width: int, height: int, quality: str) -> int:
        resolution = (width, height)

        # Try exact resolution match first
        if resolution in self.QUALITY_PRESETS:
            presets = self.QUALITY_PRESETS[resolution]
        else:
            # Calculate bitrate based on pixel count for custom resolutions
            pixel_count = width * height
            base_720p = 1280 * 720  # 921,600 pixels

            # Scale from 720p medium quality (3.5 Mbps)
            base_bitrate = 3500000
            scale_factor = pixel_count / base_720p

            medium_bitrate = int(base_bitrate * scale_factor)
            low_bitrate = int(medium_bitrate * 0.7)
            high_bitrate = int(medium_bitrate * 1.4)

            presets = (low_bitrate, medium_bitrate, high_bitrate)

        quality_map = {'low': 0, 'medium': 1, 'high': 2}
        quality_index = quality_map.get(quality.lower(), 1)

        return presets[quality_index]

    def is_encoding_completed(self) -> bool:
        return self.encoding_completed

    def get_encoding_status(self) -> dict:

        status = {
            'is_encoding': self.is_encoding,
            'encoding_completed': self.encoding_completed,
            'total_frames': len(self.buffer),
            'total_frames_encoded': self.total_frames_encoded,
            'progress_percentage': (
                self.total_frames_encoded / len(self.buffer) * 100) if self.buffer else 0,
            'output_path': str(self.output_path) if self.output_path else None,
            'encoding_error': self.encoding_error,
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
                    status['encoding_fps'] = self.total_frames_encoded / status['encoding_time']

        return status

    def _create_pipeline(self, output_path: str) -> bool:
        pipeline_str = (
            f'appsrc name=source caps=video/x-raw,format=RGB,width={self.width},'
            f'height={self.height},framerate={self.fps}/1 '
            f'! queue max-size-buffers=50 '
            f'! videoconvert ! nvvidconv '
            f'! nvv4l2h264enc bitrate={self.bitrate} preset-level={self.preset_level} '
            f'! h264parse ! qtmux ! filesink location={output_path}'
        )

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsrc = self.pipeline.get_by_name('source')

            # Configure appsrc for optimal performance
            self.appsrc.set_property('block', False)
            self.appsrc.set_property('is-live', True)
            self.appsrc.set_property('format', Gst.Format.TIME)

            return True
        except Exception as e:
            self.encoding_error = f'Pipeline creation failed: {e}'
            return False

    def _setup_message_handler(self):
        self.loop = GLib.MainLoop()
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()

        def on_message(bus, message):
            msg_type = message.type
            if msg_type == Gst.MessageType.EOS:
                self.encoding_completed = True
                self.is_encoding = False
                self.encoding_finished_at = time.time()
                self.loop.quit()
            elif msg_type == Gst.MessageType.ERROR:
                err, debug = message.parse_error()
                self.encoding_error = f'Encoding error: {err}'
                self.is_encoding = False
                self.loop.quit()
            return True

        bus.connect('message', on_message)

    def _feed_frame_data(self):
        frame_duration = Gst.SECOND // self.fps

        def feed_data():
            try:
                for i, frame in enumerate(self.buffer):
                    # Convert numpy array to RGB bytes if needed
                    if frame.dtype != np.uint8:
                        frame = (frame * 255).astype(np.uint8)

                    # Ensure RGB format (H, W, 3)
                    if len(frame.shape) == 3 and frame.shape[2] == 3:
                        img_bytes = frame.tobytes()
                    else:
                        raise ValueError(f'Unsupported frame shape: {frame.shape}')

                    # Create GStreamer buffer
                    buffer = Gst.Buffer.new_allocate(None, len(img_bytes), None)
                    buffer.fill(0, img_bytes)
                    buffer.pts = i * frame_duration
                    buffer.duration = frame_duration

                    # Push buffer to pipeline
                    ret = self.appsrc.emit('push-buffer', buffer)
                    if ret != Gst.FlowReturn.OK:
                        break

                    self.total_frames_encoded += 1

                # Signal end of stream
                self.appsrc.emit('end-of-stream')
            except Exception as e:
                self.encoding_error = f'Frame feeding error: {e}'
                self.is_encoding = False
                self.loop.quit()

            return False

        GLib.idle_add(feed_data)

    def encode_video(self, video_path: Union[str, Path]) -> None:
        if not self.buffer:
            raise ValueError('No frames in buffer to encode')

        if self.width is None or self.height is None:
            raise ValueError('Frame dimensions not set. Call set_buffer() first.')

        self.is_encoding = True
        self.encoding_completed = False
        self.encoding_started_at = time.time()
        self.encoding_finished_at = None
        self.total_frames_encoded = 0
        self.encoding_error = None

        video_path = Path(video_path)
        self.output_path = video_path
        video_path.parent.mkdir(parents=True, exist_ok=True)

        try:
            if not self._create_pipeline(str(video_path)):
                raise RuntimeError(f'Failed to create pipeline: {self.encoding_error}')

            self._setup_message_handler()

            self.pipeline.set_state(Gst.State.PLAYING)

            self._feed_frame_data()

            self.loop.run()

            if self.encoding_error:
                raise RuntimeError(f'GStreamer encoding failed: {self.encoding_error}')

            if not video_path.exists():
                raise OSError(f'Video encoding did not work. File not found: {video_path}')

        except Exception as e:
            self.is_encoding = False
            self.encoding_completed = False
            print(f'Encoding failed: {e}')
            raise
        finally:
            self._cleanup()
            if self.clear_after_encode:
                self.clear_buffer()

    def _cleanup(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None

        self.appsrc = None
        self.loop = None

    def get_last_error(self) -> Optional[str]:
        return self.encoding_error
