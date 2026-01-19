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
# Author: Generated with assistance

"""
ROSbag Image to MP4 Converter.

Converts ROSbag files containing CompressedImage topics to the rosbag + mp4 format.

Input format:
    - MCAP file with sensor_msgs/msg/CompressedImage topics

Output format:
    output_dir/
    ├── metadata.yaml           # ROSbag metadata
    ├── rosbag_0.mcap          # MCAP file with ImageMetadata instead of images
    ├── robot_config.yaml      # Camera and topic configuration
    └── videos/
        ├── camera_name_1.mp4  # MP4 video for each camera topic
        └── camera_name_2.mp4

Usage:
    python rosbag_image_to_mp4_converter.py --input /path/to/input.mcap --output /path/to/output_dir
    python rosbag_image_to_mp4_converter.py --hf-repo RobotisSW/box_folding --output /path/to/output_dir
"""

import argparse
import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml


@dataclass
class ImageFrame:
    """Container for a single image frame."""

    timestamp_ns: int
    frame_data: bytes
    width: int
    height: int
    encoding: str
    topic: str


@dataclass
class TopicInfo:
    """Information about a topic for processing."""

    topic_name: str
    schema_name: str
    message_encoding: str
    schema_id: int
    channel_id: int


@dataclass
class ConversionResult:
    """Result of the conversion process."""

    success: bool
    message: str
    output_dir: Optional[Path] = None
    video_files: List[str] = field(default_factory=list)
    frame_counts: Dict[str, int] = field(default_factory=dict)


class RosbagImageToMp4Converter:
    """
    Converts ROSbag files with CompressedImage topics to rosbag + mp4 format.

    This converter:
    1. Reads all messages from input MCAP file
    2. Extracts CompressedImage messages and converts to MP4 videos
    3. Creates new MCAP file with ImageMetadata messages instead of images
    4. Generates robot_config.yaml with camera mapping
    """

    # Image-related message types to convert
    IMAGE_TYPES = {
        "sensor_msgs/msg/CompressedImage",
        "sensor_msgs/msg/Image",
    }

    def __init__(
        self,
        fps: int = 30,
        video_codec: str = "libx264",
        crf: int = 23,
        logger=None,
    ):
        """
        Initialize converter.

        Args:
            fps: Target FPS for output videos (used if auto-detection fails)
            video_codec: FFmpeg video codec
            crf: Constant Rate Factor for video quality (lower = better)
            logger: Optional logger instance
        """
        self.fps = fps
        self.video_codec = video_codec
        self.crf = crf
        self.logger = logger

    def _log(self, level: str, msg: str):
        if self.logger:
            getattr(self.logger, level)(msg)
        else:
            print(f"[{level.upper()}] {msg}")

    def _log_info(self, msg: str):
        self._log("info", msg)

    def _log_warning(self, msg: str):
        self._log("warning", msg)

    def _log_error(self, msg: str):
        self._log("error", msg)

    def convert(
        self,
        input_path: Path,
        output_dir: Path,
    ) -> ConversionResult:
        """
        Convert a single MCAP file to rosbag + mp4 format.

        Args:
            input_path: Path to input MCAP file
            output_dir: Output directory for converted data

        Returns:
            ConversionResult with status and details
        """
        input_path = Path(input_path)
        output_dir = Path(output_dir)

        if not input_path.exists():
            return ConversionResult(
                success=False, message=f"Input file does not exist: {input_path}"
            )

        self._log_info(f"Converting: {input_path}")
        self._log_info(f"Output dir: {output_dir}")

        # Create output directory structure
        output_dir.mkdir(parents=True, exist_ok=True)
        videos_dir = output_dir / "videos"
        videos_dir.mkdir(exist_ok=True)

        try:
            # Step 1: Read and categorize all messages
            self._log_info("Step 1: Reading MCAP file...")
            image_frames_by_topic, non_image_messages, topic_infos = (
                self._read_and_categorize_messages(input_path)
            )

            if not image_frames_by_topic:
                return ConversionResult(
                    success=False, message="No image topics found in MCAP file"
                )

            self._log_info(f"Found {len(image_frames_by_topic)} image topics")
            for topic, frames in image_frames_by_topic.items():
                self._log_info(f"  {topic}: {len(frames)} frames")

            # Step 2: Encode images to MP4 videos
            self._log_info("Step 2: Encoding MP4 videos...")
            video_files = {}
            frame_counts = {}
            detected_fps_by_topic = {}

            for topic, frames in image_frames_by_topic.items():
                camera_name = self._topic_to_camera_name(topic)
                video_filename = f"{camera_name}.mp4"
                video_path = videos_dir / video_filename

                detected_fps = self._calculate_fps(frames)
                detected_fps_by_topic[topic] = detected_fps
                self._log_info(
                    f"  {topic} -> {video_filename} (FPS: {detected_fps:.2f})"
                )

                success = self._encode_video(frames, video_path, detected_fps)
                if success:
                    video_files[topic] = f"videos/{video_filename}"
                    frame_counts[camera_name] = len(frames)
                else:
                    self._log_warning(f"Failed to encode video for {topic}")

            # Step 3: Write new MCAP file with ImageMetadata
            self._log_info("Step 3: Writing new MCAP file...")
            mcap_output_path = output_dir / "rosbag_0.mcap"
            self._write_converted_mcap(
                mcap_output_path,
                image_frames_by_topic,
                non_image_messages,
                topic_infos,
                video_files,
            )

            # Step 4: Generate metadata files
            self._log_info("Step 4: Generating metadata files...")
            self._write_metadata_yaml(output_dir, input_path)
            self._write_robot_config(
                output_dir,
                image_frames_by_topic,
                topic_infos,
                detected_fps_by_topic,
            )

            self._log_info("Conversion completed successfully!")

            return ConversionResult(
                success=True,
                message="Conversion completed successfully",
                output_dir=output_dir,
                video_files=list(video_files.values()),
                frame_counts=frame_counts,
            )

        except Exception as e:
            self._log_error(f"Conversion failed: {e}")
            import traceback

            traceback.print_exc()
            return ConversionResult(success=False, message=f"Conversion failed: {e}")

    def _read_and_categorize_messages(
        self,
        input_path: Path,
    ) -> Tuple[Dict[str, List[ImageFrame]], List[Tuple], Dict[str, TopicInfo]]:
        """
        Read MCAP file and categorize messages into image and non-image.

        Returns:
            - image_frames_by_topic: Dict mapping topic -> list of ImageFrame
            - non_image_messages: List of (schema, channel, message, raw_data) for non-image messages
            - topic_infos: Dict mapping topic -> TopicInfo
        """
        from mcap.reader import make_reader

        image_frames_by_topic: Dict[str, List[ImageFrame]] = {}
        non_image_messages: List[Tuple] = []
        topic_infos: Dict[str, TopicInfo] = {}

        with open(input_path, "rb") as f:
            reader = make_reader(f)
            summary = reader.get_summary()

            # Build topic info mapping
            if summary and summary.channels:
                for channel_id, channel in summary.channels.items():
                    schema = summary.schemas.get(channel.schema_id)
                    if schema:
                        topic_infos[channel.topic] = TopicInfo(
                            topic_name=channel.topic,
                            schema_name=schema.name,
                            message_encoding=channel.message_encoding,
                            schema_id=channel.schema_id,
                            channel_id=channel_id,
                        )

        # Second pass: read all messages
        with open(input_path, "rb") as f:
            reader = make_reader(f)

            for schema, channel, message in reader.iter_messages():
                topic = channel.topic
                topic_info = topic_infos.get(topic)

                if topic_info and topic_info.schema_name in self.IMAGE_TYPES:
                    # This is an image message - decode and store
                    frame = self._decode_image_message(
                        message.data,
                        message.log_time,
                        topic,
                        topic_info.schema_name,
                    )
                    if frame:
                        if topic not in image_frames_by_topic:
                            image_frames_by_topic[topic] = []
                        image_frames_by_topic[topic].append(frame)
                else:
                    # Non-image message - store for later writing
                    non_image_messages.append((schema, channel, message))

        # Sort frames by timestamp
        for topic in image_frames_by_topic:
            image_frames_by_topic[topic].sort(key=lambda f: f.timestamp_ns)

        return image_frames_by_topic, non_image_messages, topic_infos

    def _decode_image_message(
        self,
        data: bytes,
        timestamp_ns: int,
        topic: str,
        schema_name: str,
    ) -> Optional[ImageFrame]:
        """Decode a ROS2 image message from CDR serialized data."""
        try:
            if schema_name == "sensor_msgs/msg/CompressedImage":
                return self._decode_compressed_image(data, timestamp_ns, topic)
            elif schema_name == "sensor_msgs/msg/Image":
                return self._decode_raw_image(data, timestamp_ns, topic)
        except Exception as e:
            self._log_warning(f"Failed to decode image from {topic}: {e}")
        return None

    def _decode_compressed_image(
        self,
        data: bytes,
        timestamp_ns: int,
        topic: str,
    ) -> Optional[ImageFrame]:
        """Decode sensor_msgs/msg/CompressedImage from CDR data."""
        try:
            # CDR deserialization for CompressedImage
            # Structure: Header (stamp, frame_id) + format (string) + data (sequence<uint8>)
            offset = 0

            # Skip CDR header (4 bytes)
            offset += 4

            # Read Header.stamp (sec: int32, nanosec: uint32)
            # We'll use the message timestamp instead
            offset += 8

            # Read Header.frame_id (string)
            frame_id_len = int.from_bytes(data[offset : offset + 4], "little")
            offset += 4 + frame_id_len
            # Align to 4 bytes
            offset = (offset + 3) & ~3

            # Read format (string)
            format_len = int.from_bytes(data[offset : offset + 4], "little")
            offset += 4
            format_str = data[offset : offset + format_len - 1].decode(
                "utf-8"
            )  # -1 for null terminator
            offset += format_len
            # Align to 4 bytes
            offset = (offset + 3) & ~3

            # Read data (sequence<uint8>)
            data_len = int.from_bytes(data[offset : offset + 4], "little")
            offset += 4
            image_data = data[offset : offset + data_len]

            # Decode image to get dimensions
            nparr = np.frombuffer(image_data, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if img is None:
                return None

            height, width = img.shape[:2]

            return ImageFrame(
                timestamp_ns=timestamp_ns,
                frame_data=image_data,
                width=width,
                height=height,
                encoding=format_str,
                topic=topic,
            )

        except Exception as e:
            self._log_warning(f"Failed to decode CompressedImage: {e}")
            return None

    def _decode_raw_image(
        self,
        data: bytes,
        timestamp_ns: int,
        topic: str,
    ) -> Optional[ImageFrame]:
        """Decode sensor_msgs/msg/Image from CDR data."""
        try:
            offset = 0

            # Skip CDR header (4 bytes)
            offset += 4

            # Read Header.stamp
            offset += 8

            # Read Header.frame_id
            frame_id_len = int.from_bytes(data[offset : offset + 4], "little")
            offset += 4 + frame_id_len
            offset = (offset + 3) & ~3

            # Read height (uint32)
            height = int.from_bytes(data[offset : offset + 4], "little")
            offset += 4

            # Read width (uint32)
            width = int.from_bytes(data[offset : offset + 4], "little")
            offset += 4

            # Read encoding (string)
            encoding_len = int.from_bytes(data[offset : offset + 4], "little")
            offset += 4
            encoding = data[offset : offset + encoding_len - 1].decode("utf-8")
            offset += encoding_len
            offset = (offset + 3) & ~3

            # Read is_bigendian (uint8)
            offset += 1
            offset = (offset + 3) & ~3

            # Read step (uint32)
            step = int.from_bytes(data[offset : offset + 4], "little")
            offset += 4

            # Read data (sequence<uint8>)
            data_len = int.from_bytes(data[offset : offset + 4], "little")
            offset += 4
            image_data = data[offset : offset + data_len]

            # Convert raw image to compressed for storage
            if encoding in ["rgb8", "bgr8"]:
                channels = 3
            elif encoding in ["mono8", "mono16"]:
                channels = 1
            else:
                channels = 3

            img = np.frombuffer(image_data, dtype=np.uint8).reshape(
                height, width, channels
            )
            if encoding == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            # Encode as JPEG for storage
            _, compressed = cv2.imencode(".jpg", img)

            return ImageFrame(
                timestamp_ns=timestamp_ns,
                frame_data=compressed.tobytes(),
                width=width,
                height=height,
                encoding=encoding,
                topic=topic,
            )

        except Exception as e:
            self._log_warning(f"Failed to decode Image: {e}")
            return None

    def _calculate_fps(self, frames: List[ImageFrame]) -> float:
        """Calculate FPS from frame timestamps."""
        if len(frames) < 2:
            return self.fps

        timestamps = [f.timestamp_ns for f in frames]
        time_diffs = [
            (timestamps[i + 1] - timestamps[i]) / 1e9
            for i in range(len(timestamps) - 1)
        ]

        # Filter out outliers (time gaps > 1 second)
        valid_diffs = [d for d in time_diffs if 0 < d < 1.0]

        if not valid_diffs:
            return self.fps

        avg_diff = np.median(valid_diffs)  # Use median to be robust to outliers

        if avg_diff > 0:
            return 1.0 / avg_diff

        return self.fps

    def _encode_video(
        self,
        frames: List[ImageFrame],
        output_path: Path,
        fps: float,
    ) -> bool:
        """Encode frames to MP4 video using FFmpeg or OpenCV fallback."""
        if not frames:
            return False

        # Get dimensions from first frame
        first_frame = frames[0]
        width = first_frame.width
        height = first_frame.height

        # Ensure even dimensions for H.264
        width = width if width % 2 == 0 else width - 1
        height = height if height % 2 == 0 else height - 1

        if self._ffmpeg_available():
            return self._encode_video_ffmpeg(frames, output_path, fps, width, height)
        else:
            self._log_info("FFmpeg not available, using OpenCV VideoWriter")
            return self._encode_video_opencv(frames, output_path, fps, width, height)

    def _ffmpeg_available(self) -> bool:
        """Check if FFmpeg is available."""
        try:
            subprocess.run(["ffmpeg", "-version"], capture_output=True, check=True)
            return True
        except (subprocess.CalledProcessError, FileNotFoundError):
            return False

    def _encode_video_ffmpeg(
        self,
        frames: List[ImageFrame],
        output_path: Path,
        fps: float,
        width: int,
        height: int,
    ) -> bool:
        """Encode video using FFmpeg."""
        cmd = [
            "ffmpeg",
            "-y",
            "-f",
            "rawvideo",
            "-vcodec",
            "rawvideo",
            "-s",
            f"{width}x{height}",
            "-pix_fmt",
            "bgr24",
            "-r",
            str(fps),
            "-i",
            "-",
            "-an",
            "-vcodec",
            self.video_codec,
            "-pix_fmt",
            "yuv420p",
            "-crf",
            str(self.crf),
            "-preset",
            "medium",
            "-g",
            "2",
            "-loglevel",
            "warning",
            str(output_path),
        ]

        try:
            process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            for frame in frames:
                nparr = np.frombuffer(frame.frame_data, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                if img is None:
                    continue

                if img.shape[1] != width or img.shape[0] != height:
                    img = cv2.resize(img, (width, height))

                process.stdin.write(img.tobytes())

            process.stdin.close()
            process.wait(timeout=600)

            if process.returncode != 0:
                stderr = process.stderr.read().decode()
                self._log_error(f"FFmpeg error: {stderr}")
                return False

            return output_path.exists()

        except Exception as e:
            self._log_error(f"FFmpeg encoding failed: {e}")
            return False

    def _encode_video_opencv(
        self,
        frames: List[ImageFrame],
        output_path: Path,
        fps: float,
        width: int,
        height: int,
    ) -> bool:
        """Encode video using OpenCV VideoWriter."""
        try:
            codecs = [
                ("avc1", ".mp4"),
                ("mp4v", ".mp4"),
                ("XVID", ".avi"),
            ]

            writer = None
            final_path = output_path

            for codec, ext in codecs:
                fourcc = cv2.VideoWriter_fourcc(*codec)
                test_path = output_path.with_suffix(ext)
                writer = cv2.VideoWriter(str(test_path), fourcc, fps, (width, height))
                if writer.isOpened():
                    final_path = test_path
                    self._log_info(f"Using codec: {codec}")
                    break
                writer.release()
                writer = None

            if writer is None or not writer.isOpened():
                self._log_error("Failed to initialize any video codec")
                return False

            for frame in frames:
                nparr = np.frombuffer(frame.frame_data, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                if img is None:
                    continue

                if img.shape[1] != width or img.shape[0] != height:
                    img = cv2.resize(img, (width, height))

                writer.write(img)

            writer.release()

            if final_path != output_path and final_path.exists():
                if output_path.suffix != final_path.suffix:
                    output_path = output_path.with_suffix(final_path.suffix)
                if final_path != output_path:
                    shutil.move(str(final_path), str(output_path))

            return output_path.exists() or final_path.exists()

        except Exception as e:
            self._log_error(f"OpenCV encoding failed: {e}")
            return False

        # Get dimensions from first frame
        first_frame = frames[0]
        width = first_frame.width
        height = first_frame.height

        # Ensure even dimensions for H.264
        width = width if width % 2 == 0 else width - 1
        height = height if height % 2 == 0 else height - 1

        # Build FFmpeg command
        cmd = [
            "ffmpeg",
            "-y",  # Overwrite output
            "-f",
            "rawvideo",
            "-vcodec",
            "rawvideo",
            "-s",
            f"{width}x{height}",
            "-pix_fmt",
            "bgr24",
            "-r",
            str(fps),
            "-i",
            "-",  # Read from stdin
            "-an",  # No audio
            "-vcodec",
            self.video_codec,
            "-pix_fmt",
            "yuv420p",
            "-crf",
            str(self.crf),
            "-preset",
            "medium",
            "-g",
            "2",  # GOP size
            "-loglevel",
            "warning",
            str(output_path),
        ]

        try:
            process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            for frame in frames:
                # Decode compressed image
                nparr = np.frombuffer(frame.frame_data, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                if img is None:
                    continue

                # Resize if needed
                if img.shape[1] != width or img.shape[0] != height:
                    img = cv2.resize(img, (width, height))

                process.stdin.write(img.tobytes())

            process.stdin.close()
            process.wait(timeout=600)

            if process.returncode != 0:
                stderr = process.stderr.read().decode()
                self._log_error(f"FFmpeg error: {stderr}")
                return False

            return output_path.exists()

        except Exception as e:
            self._log_error(f"Video encoding failed: {e}")
            return False

    def _write_converted_mcap(
        self,
        output_path: Path,
        image_frames_by_topic: Dict[str, List[ImageFrame]],
        non_image_messages: List[Tuple],
        topic_infos: Dict[str, TopicInfo],
        video_files: Dict[str, str],
    ):
        """Write new MCAP file with ImageMetadata instead of image data."""
        from mcap.writer import Writer

        with open(output_path, "wb") as f:
            writer = Writer(f)
            writer.start()

            # Track schemas and channels we've registered
            registered_schemas = {}  # schema_name -> schema_id
            registered_channels = {}  # topic -> channel_id

            # Define ImageMetadata schema
            image_metadata_schema = """
std_msgs/Header header

# Frame index in MP4 video file
uint32 frame_index

# Original image dimensions
uint32 width
uint32 height

# Original encoding (e.g., "rgb8", "bgr8", "mono8")
string encoding

# Path to MP4 file (relative to bag directory)
string video_file_path

# Topic name that this frame came from
string source_topic
"""

            # Register ImageMetadata schema
            metadata_schema_id = writer.register_schema(
                name="rosbag_recorder/msg/ImageMetadata",
                encoding="ros2msg",
                data=image_metadata_schema.encode("utf-8"),
            )

            # Register channels for ImageMetadata topics
            for topic in image_frames_by_topic.keys():
                metadata_topic = f"{topic}/metadata"
                channel_id = writer.register_channel(
                    schema_id=metadata_schema_id,
                    topic=metadata_topic,
                    message_encoding="cdr",
                )
                registered_channels[metadata_topic] = channel_id

            # Copy non-image schemas and messages
            schema_id_mapping = {}  # old_schema_id -> new_schema_id
            channel_id_mapping = {}  # old_channel_id -> new_channel_id

            # First pass: collect unique schemas from non-image messages
            seen_schemas = {}
            for schema, channel, message in non_image_messages:
                if schema.name not in seen_schemas:
                    seen_schemas[schema.name] = schema

            # Register non-image schemas
            for schema_name, schema in seen_schemas.items():
                if schema_name not in self.IMAGE_TYPES:
                    new_id = writer.register_schema(
                        name=schema.name,
                        encoding=schema.encoding,
                        data=schema.data,
                    )
                    registered_schemas[schema_name] = new_id

            # Register non-image channels
            seen_channels = {}
            for schema, channel, message in non_image_messages:
                if (
                    channel.topic not in seen_channels
                    and schema.name not in self.IMAGE_TYPES
                ):
                    seen_channels[channel.topic] = (schema, channel)

            for topic, (schema, channel) in seen_channels.items():
                if schema.name in registered_schemas:
                    new_channel_id = writer.register_channel(
                        schema_id=registered_schemas[schema.name],
                        topic=topic,
                        message_encoding=channel.message_encoding,
                    )
                    registered_channels[topic] = new_channel_id

            # Collect all messages to write in timestamp order
            all_messages = []

            # Add ImageMetadata messages
            for topic, frames in image_frames_by_topic.items():
                metadata_topic = f"{topic}/metadata"
                channel_id = registered_channels[metadata_topic]
                video_file = video_files.get(topic, "")

                for frame_idx, frame in enumerate(frames):
                    # Create CDR-serialized ImageMetadata message
                    metadata_data = self._create_image_metadata_cdr(
                        timestamp_ns=frame.timestamp_ns,
                        frame_index=frame_idx,
                        width=frame.width,
                        height=frame.height,
                        encoding=frame.encoding,
                        video_file_path=video_file,
                        source_topic=topic,
                    )

                    all_messages.append(
                        (
                            frame.timestamp_ns,
                            channel_id,
                            metadata_data,
                        )
                    )

            # Add non-image messages
            for schema, channel, message in non_image_messages:
                if channel.topic in registered_channels:
                    all_messages.append(
                        (
                            message.log_time,
                            registered_channels[channel.topic],
                            message.data,
                        )
                    )

            # Sort by timestamp and write
            all_messages.sort(key=lambda x: x[0])

            for log_time, channel_id, data in all_messages:
                writer.add_message(
                    channel_id=channel_id,
                    log_time=log_time,
                    data=data,
                    publish_time=log_time,
                )

            writer.finish()

    def _create_image_metadata_cdr(
        self,
        timestamp_ns: int,
        frame_index: int,
        width: int,
        height: int,
        encoding: str,
        video_file_path: str,
        source_topic: str,
    ) -> bytes:
        """Create CDR-serialized ImageMetadata message."""
        import struct

        # CDR serialization (little endian)
        data = bytearray()

        # CDR header (4 bytes): encapsulation kind (LE) + options
        data.extend(b"\x00\x01\x00\x00")  # CDR_LE

        # Header.stamp
        sec = timestamp_ns // 1_000_000_000
        nanosec = timestamp_ns % 1_000_000_000
        data.extend(struct.pack("<I", sec))  # sec (int32 as uint32)
        data.extend(struct.pack("<I", nanosec))  # nanosec (uint32)

        # Header.frame_id (string)
        frame_id = ""
        frame_id_bytes = (frame_id + "\x00").encode("utf-8")
        data.extend(struct.pack("<I", len(frame_id_bytes)))
        data.extend(frame_id_bytes)
        # Align to 4 bytes
        while len(data) % 4 != 0:
            data.append(0)

        # frame_index (uint32)
        data.extend(struct.pack("<I", frame_index))

        # width (uint32)
        data.extend(struct.pack("<I", width))

        # height (uint32)
        data.extend(struct.pack("<I", height))

        # encoding (string)
        encoding_bytes = (encoding + "\x00").encode("utf-8")
        data.extend(struct.pack("<I", len(encoding_bytes)))
        data.extend(encoding_bytes)
        while len(data) % 4 != 0:
            data.append(0)

        # video_file_path (string)
        video_bytes = (video_file_path + "\x00").encode("utf-8")
        data.extend(struct.pack("<I", len(video_bytes)))
        data.extend(video_bytes)
        while len(data) % 4 != 0:
            data.append(0)

        # source_topic (string)
        topic_bytes = (source_topic + "\x00").encode("utf-8")
        data.extend(struct.pack("<I", len(topic_bytes)))
        data.extend(topic_bytes)

        return bytes(data)

    def _topic_to_camera_name(self, topic: str) -> str:
        """Convert topic name to camera name for video file."""
        # Remove leading slashes and replace remaining with underscores
        name = topic.lstrip("/")
        name = name.replace("/", "_")

        # Common patterns
        topic_lower = topic.lower()
        if "zed" in topic_lower:
            return "cam_head"
        elif "camera_left" in topic_lower:
            return "cam_wrist_left"
        elif "camera_right" in topic_lower:
            return "cam_wrist_right"

        return name

    def _write_metadata_yaml(self, output_dir: Path, input_path: Path):
        """Write ROSbag metadata.yaml file compatible with rosbag2."""
        from mcap.reader import make_reader

        output_mcap = output_dir / "rosbag_0.mcap"

        if not output_mcap.exists():
            self._log_warning(f"Output MCAP not found: {output_mcap}")
            return

        min_time = float("inf")
        max_time = float("-inf")
        topic_counts: Dict[str, int] = {}
        topic_types: Dict[str, str] = {}

        with open(output_mcap, "rb") as f:
            reader = make_reader(f)
            summary = reader.get_summary()

            if summary and summary.channels:
                for channel_id, channel in summary.channels.items():
                    schema = summary.schemas.get(channel.schema_id)
                    if schema:
                        topic_types[channel.topic] = schema.name

        with open(output_mcap, "rb") as f:
            reader = make_reader(f)
            for schema, channel, message in reader.iter_messages():
                topic = channel.topic
                min_time = min(min_time, message.log_time)
                max_time = max(max_time, message.log_time)
                topic_counts[topic] = topic_counts.get(topic, 0) + 1

        total_message_count = sum(topic_counts.values())
        duration_ns = max_time - min_time if total_message_count > 0 else 0

        topics_with_message_count = []
        for topic, count in sorted(topic_counts.items()):
            topic_type = topic_types.get(topic, "unknown")
            topics_with_message_count.append(
                {
                    "topic_metadata": {
                        "name": topic,
                        "type": topic_type,
                        "serialization_format": "cdr",
                        "offered_qos_profiles": "",
                    },
                    "message_count": count,
                }
            )

        metadata = {
            "rosbag2_bagfile_information": {
                "version": 8,
                "storage_identifier": "mcap",
                "relative_file_paths": ["rosbag_0.mcap"],
                "duration": {
                    "nanoseconds": int(duration_ns),
                },
                "starting_time": {
                    "nanoseconds_since_epoch": int(min_time)
                    if total_message_count > 0
                    else 0,
                },
                "message_count": total_message_count,
                "topics_with_message_count": topics_with_message_count,
                "compression_format": "",
                "compression_mode": "",
            }
        }

        metadata_path = output_dir / "metadata.yaml"
        with open(metadata_path, "w") as f:
            yaml.dump(metadata, f, default_flow_style=False, sort_keys=False)

    def _write_robot_config(
        self,
        output_dir: Path,
        image_frames_by_topic: Dict[str, List[ImageFrame]],
        topic_infos: Dict[str, TopicInfo],
        detected_fps_by_topic: Dict[str, float],
    ):
        """Write robot_config.yaml with camera mapping and topic configuration."""
        # Build camera_topics mapping
        camera_topics = {}
        camera_mapping = {}

        for topic in image_frames_by_topic.keys():
            camera_name = self._topic_to_camera_name(topic)
            camera_topics[camera_name] = topic
            camera_mapping[topic] = camera_name

        # Detect state and action topics from non-image topic infos
        state_topics = {}
        action_topics = {}

        for topic, info in topic_infos.items():
            if info.schema_name in self.IMAGE_TYPES:
                continue

            if "JointState" in info.schema_name:
                if "leader" not in topic.lower() and "action" not in topic.lower():
                    state_topics["follower"] = topic
            elif "JointTrajectory" in info.schema_name:
                if "left" in topic.lower():
                    if "joystick" in topic.lower():
                        action_topics["joystick_left"] = topic
                    else:
                        action_topics["arm_left"] = topic
                elif "right" in topic.lower():
                    if "joystick" in topic.lower():
                        action_topics["joystick_right"] = topic
                    else:
                        action_topics["arm_right"] = topic

        # Calculate average FPS
        fps_values = list(detected_fps_by_topic.values())
        avg_fps = int(round(sum(fps_values) / len(fps_values))) if fps_values else 30

        config = {
            "robot_type": "ai_worker",  # Default, can be overridden
            "fps": avg_fps,
            "camera_topics": camera_topics,
            "camera_mapping": camera_mapping,
            "state_topics": state_topics,
            "action_topics": action_topics,
        }

        config_path = output_dir / "robot_config.yaml"
        with open(config_path, "w") as f:
            yaml.dump(config, f, default_flow_style=False)


def convert_from_huggingface(
    repo_id: str,
    output_dir: Path,
    converter: RosbagImageToMp4Converter,
) -> ConversionResult:
    """
    Download and convert a dataset from HuggingFace.

    Args:
        repo_id: HuggingFace dataset repo ID (e.g., "RobotisSW/box_folding")
        output_dir: Output directory
        converter: Converter instance

    Returns:
        ConversionResult
    """
    from huggingface_hub import list_repo_files, hf_hub_download

    print(f"Fetching dataset: {repo_id}")

    # List MCAP files in the repo
    try:
        files = list(list_repo_files(repo_id, repo_type="dataset"))
        mcap_files = [f for f in files if f.endswith(".mcap")]
    except Exception as e:
        return ConversionResult(
            success=False, message=f"Failed to list files in {repo_id}: {e}"
        )

    if not mcap_files:
        return ConversionResult(
            success=False, message=f"No MCAP files found in {repo_id}"
        )

    print(f"Found {len(mcap_files)} MCAP file(s)")

    # Process each MCAP file
    all_results = []

    for mcap_file in mcap_files:
        print(f"\nProcessing: {mcap_file}")

        # Download the file
        try:
            local_path = hf_hub_download(
                repo_id=repo_id,
                filename=mcap_file,
                repo_type="dataset",
            )
        except Exception as e:
            print(f"Failed to download {mcap_file}: {e}")
            continue

        # Create output subdirectory
        episode_name = Path(mcap_file).stem
        episode_output_dir = output_dir / episode_name

        # Convert
        result = converter.convert(Path(local_path), episode_output_dir)
        all_results.append(result)

        if result.success:
            print(f"  Success: {result.output_dir}")
            print(f"  Videos: {result.video_files}")
        else:
            print(f"  Failed: {result.message}")

    # Summarize
    successful = sum(1 for r in all_results if r.success)

    return ConversionResult(
        success=successful > 0,
        message=f"Converted {successful}/{len(mcap_files)} episodes",
        output_dir=output_dir,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Convert ROSbag with CompressedImage to rosbag + mp4 format"
    )

    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--input", "-i", type=str, help="Path to input MCAP file")
    input_group.add_argument(
        "--hf-repo",
        type=str,
        help="HuggingFace dataset repo ID (e.g., RobotisSW/box_folding)",
    )

    parser.add_argument(
        "--output", "-o", type=str, required=True, help="Output directory"
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Target FPS (default: auto-detect, fallback to 30)",
    )
    parser.add_argument(
        "--codec", type=str, default="libx264", help="Video codec (default: libx264)"
    )
    parser.add_argument(
        "--crf",
        type=int,
        default=23,
        help="Video quality CRF (default: 23, lower = better)",
    )

    args = parser.parse_args()

    converter = RosbagImageToMp4Converter(
        fps=args.fps,
        video_codec=args.codec,
        crf=args.crf,
    )

    output_dir = Path(args.output)

    if args.input:
        result = converter.convert(Path(args.input), output_dir)
    else:
        result = convert_from_huggingface(args.hf_repo, output_dir, converter)

    if result.success:
        print(f"\nConversion completed successfully!")
        print(f"Output: {result.output_dir}")
        if result.video_files:
            print(f"Videos: {result.video_files}")
        if result.frame_counts:
            print(f"Frame counts: {result.frame_counts}")
    else:
        print(f"\nConversion failed: {result.message}")
        exit(1)


if __name__ == "__main__":
    main()
