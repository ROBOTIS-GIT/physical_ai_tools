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

"""Simple HTTP server for serving video files with Range Request support."""

import mimetypes
import os
import re
import threading
from functools import partial
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from typing import Optional
from urllib.parse import unquote, urlparse


class VideoFileHandler(SimpleHTTPRequestHandler):
    """HTTP request handler with Range Request support for video streaming."""

    # Base directories that are allowed to be served
    allowed_base_paths = []

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

                # Send the requested range
                with open(path, 'rb') as f:
                    f.seek(start)
                    self.wfile.write(f.read(length))
                return

        # No Range header - send entire file
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(file_size))
        self.send_header('Accept-Ranges', 'bytes')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        with open(path, 'rb') as f:
            self.wfile.write(f.read())

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
        self.send_header('Access-Control-Allow-Methods', 'GET, HEAD, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Range')
        self.end_headers()

    def log_message(self, format, *args):
        """Suppress default logging."""
        pass


class VideoFileServer:
    """Simple HTTP server for serving video files."""

    def __init__(self, port: int = 8765, allowed_paths: Optional[list] = None):
        """
        Initialize VideoFileServer.

        Args:
            port: Port number to listen on
            allowed_paths: List of base paths that are allowed to be served
        """
        self.port = port
        self.allowed_paths = allowed_paths or [str(Path.home())]
        self.server: Optional[HTTPServer] = None
        self.server_thread: Optional[threading.Thread] = None
        self._running = False

    def start(self):
        """Start the video file server in a background thread."""
        if self._running:
            return

        # Configure the handler with allowed paths
        VideoFileHandler.allowed_base_paths = self.allowed_paths

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

    def get_video_url(self, file_path: str) -> str:
        """
        Get the URL for a video file.

        Args:
            file_path: Absolute path to the video file

        Returns:
            URL to access the video file
        """
        return f"http://localhost:{self.port}/video/{file_path}"

    @property
    def is_running(self) -> bool:
        """Check if server is running."""
        return self._running
