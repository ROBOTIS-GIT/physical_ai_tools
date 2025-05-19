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

import numpy as np
import subprocess
import cv2
from pathlib import Path
from typing import List, Optional, Union
import os
import time
import tempfile

class VideoBufferEncoder:
    
    def __init__(
        self,
        fps: int = 30,
        vcodec: str = "libx264",
        pix_fmt: str = "yuv420p",
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
        """Encode images in the buffer to video (to be implemented by subclasses)"""
        raise NotImplementedError("Must be implemented by subclasses")


class FFmpegBufferEncoder(VideoBufferEncoder):
    
    def __init__(
        self, 
        *args, 
        chunk_size: int = 100,  # Process frames in chunks to avoid buffer overflow
        preset: str = "medium",  # Encoding preset (ultrafast, superfast, veryfast, faster, fast, medium, slow, slower, veryslow)
        crf: Optional[int] = 23, # Default quality setting (23 is a good balance between quality and size)
        clear_after_encode: bool = True,  # Whether to clear the buffer after encoding
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
            "is_encoding": self.is_encoding,
            "encoding_completed": self.encoding_completed,
            "total_frames": len(self.buffer),
            "total_frames_encoded": self.total_frames_encoded,
            "total_chunks": (len(self.buffer) + self.chunk_size - 1) // self.chunk_size if self.buffer else 0,
            "chunks_encoded": self.total_chunks_encoded,
            "current_chunk": self.current_chunk,
            "progress_percentage": (self.total_frames_encoded / len(self.buffer) * 100) if self.buffer else 0,
            "output_path": str(self.output_path) if self.output_path else None,
        }
        
        if self.encoding_started_at:
            status["started_at"] = self.encoding_started_at
            status["elapsed_time"] = (self.encoding_finished_at or time.time()) - self.encoding_started_at
            
            if self.encoding_finished_at:
                status["finished_at"] = self.encoding_finished_at
                status["encoding_time"] = self.encoding_finished_at - self.encoding_started_at
                
                if self.output_path and self.output_path.exists():
                    status["file_size"] = os.path.getsize(self.output_path)
                    status["file_size_kb"] = status["file_size"] / 1024
                
                if self.total_frames_encoded > 0:
                    status["fps"] = self.total_frames_encoded / status["encoding_time"]
        
        return status
    
    def encode_video(self, video_path: Union[str, Path]) -> None:
        if not self.buffer:
            raise ValueError("No frames in buffer to encode")
        
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
        
        print(f"Starting FFmpeg encoding: {total_frames} frames at {self.fps} fps")
        print(f"Settings: chunk_size={self.chunk_size}, preset='{self.preset}', crf={self.crf}")
        
        # Directly construct FFmpeg command
        cmd = ["ffmpeg"]
        cmd.extend(["-f", "rawvideo"])
        cmd.extend(["-vcodec", "rawvideo"])
        cmd.extend(["-s", f"{width}x{height}"])
        cmd.extend(["-pix_fmt", "bgr24"])  # OpenCV uses BGR order
        cmd.extend(["-r", str(self.fps)])
        cmd.extend(["-i", "-"])
        cmd.extend(["-an"])
        cmd.extend(["-vcodec", self.vcodec])
        cmd.extend(["-pix_fmt", self.pix_fmt])
        cmd.extend(["-preset", self.preset])  # Add preset for encoding speed
        
        if self.g is not None:
            cmd.extend(["-g", str(self.g)])
        
        if self.crf is not None:
            cmd.extend(["-crf", str(self.crf)])
        
        if self.fast_decode:
            if self.vcodec == "libsvtav1":
                cmd.extend(["-svtav1-params", f"fast-decode={self.fast_decode}"])
            else:
                cmd.extend(["-tune", "fastdecode"])
        
        cmd.extend(["-loglevel", "warning"])  # Use warning level for more verbose logs
        cmd.extend(["-y"])
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
                chunk_size = len(chunk)
                print(f"Processing chunk {self.current_chunk}/{(total_frames+self.chunk_size-1)//self.chunk_size}: frames {i}-{i+chunk_size-1}")
                
                # Send each frame in the chunk to FFmpeg
                for j, frame in enumerate(chunk):
                    try:
                        self.process.stdin.write(frame.tobytes())
                        self.total_frames_encoded += 1
                    except BrokenPipeError as e:
                        stderr_output = self.process.stderr.read().decode()
                        print(f"Error processing frame {i+j}: {e}")
                        print(f"FFmpeg error message: {stderr_output}")
                        self.is_encoding = False
                        raise RuntimeError(f"Error in FFmpeg stream processing: {stderr_output}") from e
                
                # Flush after each chunk
                self.process.stdin.flush()
                self.total_chunks_encoded += 1
            
            # Close input pipe and wait for process to complete
            self.process.stdin.close()
            self.process.wait(timeout=60)  # Increased timeout for large videos
            
            stderr = self.process.stderr.read().decode()
            if self.process.returncode != 0:
                print(f"FFmpeg error message: {stderr}")
                self.is_encoding = False
                raise RuntimeError(f"FFmpeg encoding failed (code: {self.process.returncode}): {stderr}")
            
            if not video_path.exists():
                self.is_encoding = False
                raise OSError(f"Video encoding did not work. File not found: {video_path}")
            
            # Update encoding status
            self.is_encoding = False
            self.encoding_completed = True
            self.encoding_finished_at = time.time()
            
            print(f"FFmpeg encoding completed successfully: {video_path}")
            print(f"File size: {os.path.getsize(video_path)/1024:.1f} KB")
            print(f"Encoding time: {self.encoding_finished_at - self.encoding_started_at:.2f} seconds")
            print(f"Encoding speed: {self.total_frames_encoded / (self.encoding_finished_at - self.encoding_started_at):.1f} fps")
            
        except Exception as e:
            print(f"Exception occurred: {str(e)}")
            self.is_encoding = False
            self.encoding_completed = False
            if self.process:
                # Force terminate process if running
                try:
                    self.process.kill()
                except:
                    pass
            raise
        finally:
            # Clean up buffer if configured to do so
            if self.clear_after_encode:
                self.clear_buffer()


def test_ffmpeg_encoder():
    print("\n=== FFmpegBufferEncoder Performance Test ===")
    
    # Create test frames
    print("Pre-generating test frames...")
    width, height = 640, 480
    num_frames = 3600  # 10 seconds at 30fps
    
    frames = []
    for i in range(num_frames):
        # Create a simple test frame with frame number and gradient
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Generate different color pattern for each frame
        r = (i * 10) % 255
        g = (i * 20) % 255
        b = (i * 30) % 255
        
        # Create gradient
        for y in range(0, height, 4):
            for x in range(0, width, 4):
                color = [
                    (r + y // 8) % 255,
                    (g + x // 8) % 255,
                    (b + (y + x) // 16) % 255
                ]
                img[y:y+4, x:x+4] = color
        
        # Display frame number
        cv2.putText(
            img, 
            f"Frame {i}", 
            (50, 50), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            1, 
            (255, 255, 255), 
            2
        )
        
        frames.append(img)
    
    print(f"Generated {len(frames)} test frames")
    
    # Create output directory for test results
    output_dir = Path('/home/dongyun/ros2_ws/src')
    output_dir.mkdir(exist_ok=True)
    
    # Test the encoder with default settings
    output_file = output_dir / "test_output.mp4"
    
    # Create encoder
    encoder = FFmpegBufferEncoder(
        fps=30,
        chunk_size=50,
        preset="ultrafast",
        crf=28
    )
    
    # Set buffer directly (no copying)
    print(f"\nSetting buffer directly (direct reference)")
    start_set = time.time()
    encoder.set_buffer(frames)
    set_time = time.time() - start_set
    print(f"Time to set {num_frames} frames: {set_time:.2f} seconds")
    
    # Measure encoding time
    print(f"Encoding video with FFmpegBufferEncoder")
    print(f"Output file: {output_file}")
    
    # Start encoding in a separate thread
    import threading
    start_time = time.time()
    
    encoding_thread = threading.Thread(target=encoder.encode_video, args=(output_file,))
    encoding_thread.start()
    
    # Monitor encoding status
    while encoding_thread.is_alive():
        status = encoder.get_encoding_status()
        if status["is_encoding"] and status["total_frames"] > 0:
            progress = status["progress_percentage"]
            elapsed = time.time() - start_time
            print(f"Progress: {progress:.1f}% - Chunk: {status['current_chunk']}/{status['total_chunks']} - Time: {elapsed:.1f}s", end="\r")
            
        # Sleep briefly to avoid busy waiting
        time.sleep(0.5)
    
    # Wait for encoding to complete
    encoding_thread.join()
    encode_time = time.time() - start_time
    
    # Check if encoding completed successfully
    if encoder.is_encoding_completed():
        print("\nEncoding completed successfully!")
    else:
        print("\nEncoding failed or was interrupted")
    
    # Get final status
    status = encoder.get_encoding_status()
    
    # Calculate performance metrics
    fps = num_frames / encode_time if encode_time > 0 else 0
    file_size = os.path.getsize(output_file) if output_file.exists() else 0
    
    # Print results
    print("\n=== Results ===")
    print(f"Frames encoded: {num_frames}")
    print(f"Encoding time: {encode_time:.2f} seconds")
    print(f"Encoding speed: {fps:.1f} fps")
    print(f"Output file size: {file_size/1024:.1f} KB")
    print(f"Compression ratio: {(width * height * 3 * num_frames) / file_size:.1f}:1")
    
    # Print detailed status
    print("\n=== Detailed Encoding Status ===")
    for key, value in status.items():
        print(f"{key}: {value}")
    
    print("\nFFmpegBufferEncoder is ideal for efficiently encoding large batches of image frames")
    print("while minimizing memory usage through chunk-based processing.")


if __name__ == "__main__":
    # Run the simplified test
    test_ffmpeg_encoder()