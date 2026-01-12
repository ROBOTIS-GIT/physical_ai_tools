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

"""Simple HTTP server for serving video files and replay data."""

import json
import mimetypes
import os
import re
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from typing import Optional
from urllib.parse import unquote, urlparse


class VideoFileHandler(SimpleHTTPRequestHandler):
    """HTTP request handler with Range Request support for video streaming."""

    # Base directories that are allowed to be served
    allowed_base_paths = []
    # Replay data handler instance (set by server)
    replay_data_handler = None

    def __init__(self, *args, **kwargs):
        # Don't call super().__init__ here, let it be called by HTTPServer
        super().__init__(*args, **kwargs)

    def translate_path(self, path):
        """Translate URL path to file system path."""
        # Parse the path
        path = urlparse(path).path
        path = unquote(path)

        # Remove /video/ prefix if present
        if path.startswith('/video/'):
            path = path[7:]  # Remove '/video/'

        # Ensure absolute path (add leading / if missing)
        if path and not path.startswith('/'):
            path = '/' + path

        # Ensure the path doesn't escape allowed directories
        path = os.path.normpath(path)

        # Check if path is within allowed base paths
        for base_path in self.allowed_base_paths:
            if path.startswith(base_path):
                return path

        # If no allowed base path matches, return the path as-is
        # (will result in 404 if file doesn't exist)
        return path

    def do_GET(self):
        """Handle GET requests with Range support."""
        parsed_path = urlparse(self.path).path
        parsed_path = unquote(parsed_path)

        # Handle replay data requests
        if parsed_path.startswith('/replay-data/'):
            self._handle_replay_data_request(parsed_path)
            return

        # Handle rosbag list requests
        if parsed_path.startswith('/rosbag-list/'):
            self._handle_rosbag_list_request(parsed_path)
            return

        # Handle video file requests
        path = self.translate_path(self.path)

        if not os.path.isfile(path):
            self.send_error(404, "File not found")
            return

        file_size = os.path.getsize(path)
        content_type, _ = mimetypes.guess_type(path)
        if content_type is None:
            content_type = 'application/octet-stream'

        # Check for Range header
        range_header = self.headers.get('Range')

        try:
            if range_header:
                # Parse Range header
                range_match = re.match(r'bytes=(\d*)-(\d*)', range_header)
                if range_match:
                    start = range_match.group(1)
                    end = range_match.group(2)

                    start = int(start) if start else 0
                    end = int(end) if end else file_size - 1

                    # Validate range
                    if start >= file_size:
                        self.send_error(416, "Range Not Satisfiable")
                        return

                    end = min(end, file_size - 1)
                    length = end - start + 1

                    # Send partial content response
                    self.send_response(206)
                    self.send_header('Content-Type', content_type)
                    self.send_header('Content-Length', str(length))
                    self.send_header('Content-Range', f'bytes {start}-{end}/{file_size}')
                    self.send_header('Accept-Ranges', 'bytes')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()

                    # Send the requested range in chunks
                    self._send_file_chunked(path, start, length)
                    return

            # No Range header - send entire file
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(file_size))
            self.send_header('Accept-Ranges', 'bytes')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            self._send_file_chunked(path, 0, file_size)

        except (BrokenPipeError, ConnectionResetError):
            # Client disconnected, ignore
            pass

    def _send_file_chunked(self, path, start, length):
        """Send file in chunks to handle large files efficiently."""
        chunk_size = 1024 * 1024  # 1MB chunks
        with open(path, 'rb') as f:
            f.seek(start)
            remaining = length
            while remaining > 0:
                chunk = f.read(min(chunk_size, remaining))
                if not chunk:
                    break
                self.wfile.write(chunk)
                remaining -= len(chunk)

    def _send_json_error(self, code, message):
        """Send JSON error response with CORS headers."""
        error_response = json.dumps({
            'success': False,
            'message': message
        })
        error_bytes = error_response.encode('utf-8')

        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(error_bytes)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(error_bytes)

    def _handle_replay_data_request(self, parsed_path):
        """Handle replay data API requests."""
        # Extract bag path from URL: /replay-data/<bag_path>
        bag_path = parsed_path[13:]  # Remove '/replay-data/'

        # Ensure absolute path (add leading / if missing)
        if bag_path and not bag_path.startswith('/'):
            bag_path = '/' + bag_path

        if not bag_path:
            self._send_json_error(400, "Missing bag_path parameter")
            return

        if not self.replay_data_handler:
            self._send_json_error(500, "Replay data handler not configured")
            return

        # Check if path exists
        if not os.path.isdir(bag_path):
            self._send_json_error(404, f"Bag path not found: {bag_path}")
            return

        try:
            # Get replay data
            result = self.replay_data_handler.get_replay_data(bag_path)

            # Convert to JSON
            json_data = json.dumps(result, ensure_ascii=False)
            json_bytes = json_data.encode('utf-8')

            # Send response
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(json_bytes)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()

            self.wfile.write(json_bytes)

        except Exception as e:
            error_response = json.dumps({
                'success': False,
                'message': f'Error processing replay data: {str(e)}'
            })
            error_bytes = error_response.encode('utf-8')

            self.send_response(500)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(error_bytes)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            self.wfile.write(error_bytes)

    def _handle_rosbag_list_request(self, parsed_path):
        """Handle rosbag list API requests."""
        # Extract folder path from URL: /rosbag-list/<folder_path>
        folder_path = parsed_path[13:]  # Remove '/rosbag-list/'

        # Ensure absolute path (add leading / if missing)
        if folder_path and not folder_path.startswith('/'):
            folder_path = '/' + folder_path

        if not folder_path:
            self._send_json_error(400, "Missing folder_path parameter")
            return

        if not self.replay_data_handler:
            self._send_json_error(500, "Replay data handler not configured")
            return

        # Check if path exists
        if not os.path.isdir(folder_path):
            self._send_json_error(404, f"Folder not found: {folder_path}")
            return

        try:
            # Get rosbag list
            result = self.replay_data_handler.get_rosbag_list(folder_path)

            # Convert to JSON
            json_data = json.dumps(result, ensure_ascii=False)
            json_bytes = json_data.encode('utf-8')

            # Send response
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(json_bytes)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()

            self.wfile.write(json_bytes)

        except Exception as e:
            error_response = json.dumps({
                'success': False,
                'message': f'Error getting rosbag list: {str(e)}'
            })
            error_bytes = error_response.encode('utf-8')

            self.send_response(500)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(error_bytes)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            self.wfile.write(error_bytes)

    def do_HEAD(self):
        """Handle HEAD requests with CORS support."""
        path = self.translate_path(self.path)

        if not os.path.isfile(path):
            self.send_error(404, "File not found")
            return

        file_size = os.path.getsize(path)
        content_type, _ = mimetypes.guess_type(path)
        if content_type is None:
            content_type = 'application/octet-stream'

        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(file_size))
        self.send_header('Accept-Ranges', 'bytes')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, HEAD, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Range')
        self.end_headers()

    def do_OPTIONS(self):
        """Handle CORS preflight requests."""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, HEAD, PUT, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Range, Content-Type')
        self.end_headers()

    def do_PUT(self):
        """Handle PUT requests for updating data."""
        parsed_path = urlparse(self.path).path
        parsed_path = unquote(parsed_path)

        # Handle task markers update
        if parsed_path.startswith('/task-markers/'):
            self._handle_task_markers_update(parsed_path)
            return

        self._send_json_error(404, "Unknown PUT endpoint")

    def _handle_task_markers_update(self, parsed_path):
        """Handle task markers update API requests."""
        # Extract bag path from URL: /task-markers/<bag_path>
        bag_path = parsed_path[14:]  # Remove '/task-markers/'

        # Ensure absolute path
        if bag_path and not bag_path.startswith('/'):
            bag_path = '/' + bag_path

        if not bag_path:
            self._send_json_error(400, "Missing bag_path parameter")
            return

        if not self.replay_data_handler:
            self._send_json_error(500, "Replay data handler not configured")
            return

        # Check if path exists
        if not os.path.isdir(bag_path):
            self._send_json_error(404, f"Bag path not found: {bag_path}")
            return

        try:
            # Read request body
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self._send_json_error(400, "Empty request body")
                return

            body = self.rfile.read(content_length)
            data = json.loads(body.decode('utf-8'))
            task_markers = data.get('task_markers', [])
            trim_points = data.get('trim_points', None)
            exclude_regions = data.get('exclude_regions', None)

            # Update task markers, trim points, and exclude regions
            result = self.replay_data_handler.update_task_markers(
                bag_path, task_markers, trim_points, exclude_regions
            )

            # Send response
            json_data = json.dumps(result, ensure_ascii=False)
            json_bytes = json_data.encode('utf-8')

            self.send_response(200 if result['success'] else 500)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(json_bytes)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            self.wfile.write(json_bytes)

        except json.JSONDecodeError as e:
            self._send_json_error(400, f"Invalid JSON: {str(e)}")
        except Exception as e:
            self._send_json_error(500, f"Error updating task markers: {str(e)}")

    def log_message(self, format, *args):
        """Suppress default logging."""
        pass


class VideoFileServer:
    """Simple HTTP server for serving video files and replay data."""

    def __init__(
        self,
        port: int = 8082,
        allowed_paths: Optional[list] = None,
        replay_data_handler=None
    ):
        """
        Initialize VideoFileServer.

        Args:
            port: Port number to listen on
            allowed_paths: List of base paths that are allowed to be served
            replay_data_handler: ReplayDataHandler instance for serving replay data
        """
        self.port = port
        self.allowed_paths = allowed_paths or [str(Path.home())]
        self.replay_data_handler = replay_data_handler
        self.server: Optional[HTTPServer] = None
        self.server_thread: Optional[threading.Thread] = None
        self._running = False

    def start(self):
        """Start the video file server in a background thread."""
        if self._running:
            return

        # Configure the handler with allowed paths and replay handler
        VideoFileHandler.allowed_base_paths = self.allowed_paths
        VideoFileHandler.replay_data_handler = self.replay_data_handler

        try:
            self.server = HTTPServer(('0.0.0.0', self.port), VideoFileHandler)
            self._running = True

            self.server_thread = threading.Thread(
                target=self._serve_forever,
                daemon=True
            )
            self.server_thread.start()

        except Exception as e:
            self._running = False
            raise RuntimeError(f"Failed to start video server on port {self.port}: {e}")

    def _serve_forever(self):
        """Server loop using serve_forever for better request handling."""
        if self.server:
            self.server.serve_forever()

    def stop(self):
        """Stop the video file server."""
        self._running = False
        if self.server:
            self.server.shutdown()
            self.server = None
        if self.server_thread:
            self.server_thread.join(timeout=2.0)
            self.server_thread = None

    def set_replay_data_handler(self, handler):
        """Set or update the replay data handler."""
        self.replay_data_handler = handler
        VideoFileHandler.replay_data_handler = handler

    def get_video_url(self, file_path: str) -> str:
        """
        Get the URL for a video file.

        Args:
            file_path: Absolute path to the video file

        Returns:
            URL to access the video file
        """
        return f"http://localhost:{self.port}/video/{file_path}"

    def get_replay_data_url(self, bag_path: str) -> str:
        """
        Get the URL for replay data.

        Args:
            bag_path: Absolute path to the bag directory

        Returns:
            URL to access the replay data
        """
        return f"http://localhost:{self.port}/replay-data/{bag_path}"

    @property
    def is_running(self) -> bool:
        """Check if server is running."""
        return self._running
