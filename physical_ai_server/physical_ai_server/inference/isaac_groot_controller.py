#!/usr/bin/env python3
"""
Docker container controller for managing isaac_gr00t inference server.

This module provides a simple interface to start/stop the inference server
running in the isaac_gr00t container from the physical_ai_server ROS node.
"""

import subprocess
import time
from typing import Optional, Tuple


class IsaacGr00tServerController:
    """Controller for isaac_gr00t inference server."""
    
    CONTAINER_NAME = "isaac_gr00t"
    SCRIPT_PATH = "/workspace/scripts/server_inference_async.py"
    LOG_PATH = "/tmp/inference_server.log"
    
    def __init__(self, logger=None):
        """
        Initialize the controller.
        
        Parameters
        ----------
        logger : optional
            ROS logger instance for logging messages
        
        """
        self.logger = logger
        self._check_docker_available()
    
    def _log_info(self, message: str):
        """Log info message."""
        if self.logger:
            self.logger.info(message)
        else:
            print(f"INFO: {message}")
    
    def _log_error(self, message: str):
        """Log error message."""
        if self.logger:
            self.logger.error(message)
        else:
            print(f"ERROR: {message}")
    
    def _log_warn(self, message: str):
        """Log warning message."""
        if self.logger:
            self.logger.warn(message)
        else:
            print(f"WARN: {message}")
    
    def _check_docker_available(self) -> bool:
        """Check if docker command is available."""
        try:
            subprocess.run(
                ["docker", "--version"],
                check=True,
                capture_output=True,
                text=True,
                timeout=5
            )
            return True
        except (FileNotFoundError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
            self._log_error(
                "Docker CLI not found. Install with: apt-get update && apt-get install -y docker.io"
            )
            return False
    
    def is_container_running(self) -> bool:
        """
        Check if isaac_gr00t container is running.
        
        Returns
        -------
        bool
            True if container is running, False otherwise
        
        """
        try:
            result = subprocess.run(
                ["docker", "ps", "--filter", f"name={self.CONTAINER_NAME}", "--format", "{{.Names}}"],
                check=True,
                capture_output=True,
                text=True,
                timeout=5
            )
            return self.CONTAINER_NAME in result.stdout
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            return False
    
    def is_server_running(self) -> bool:
        """
        Check if server_inference_async.py is running in the container.
        
        Returns
        -------
        bool
            True if server is running, False otherwise
        
        """
        if not self.is_container_running():
            return False
        
        try:
            result = subprocess.run(
                ["docker", "exec", self.CONTAINER_NAME, "pgrep", "-f", "server_inference_async.py"],
                capture_output=True,
                text=True,
                timeout=5
            )
            return result.returncode == 0 and result.stdout.strip() != ""
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            return False
    
    def start_server(self, host: str = "localhost", port: int = 5555) -> Tuple[bool, str]:
        """
        Start the inference server.
        
        Parameters
        ----------
        host : str, optional
            Server host address (default: "localhost")
        port : int, optional
            Server port (default: 5555)
        
        Returns
        -------
        tuple[bool, str]
            (success, message)
        
        """
        if not self.is_container_running():
            msg = f"Container '{self.CONTAINER_NAME}' is not running"
            self._log_error(msg)
            return False, msg
        
        if self.is_server_running():
            msg = "Inference server is already running"
            self._log_info(msg)
            return True, msg
        
        self._log_info(f"Starting inference server on {host}:{port}...")
        try:
            # Start server in background using nohup
            cmd = [
                "docker", "exec", "-d", self.CONTAINER_NAME,
                "bash", "-c",
                f"cd /workspace && nohup python3 {self.SCRIPT_PATH} "
                f"--host {host} --port {port} > {self.LOG_PATH} 2>&1 &"
            ]
            subprocess.run(cmd, check=True, timeout=10)
            
            # Wait and verify
            time.sleep(2)
            if self.is_server_running():
                msg = "Inference server started successfully"
                self._log_info(msg)
                return True, msg
            else:
                msg = "Server failed to start. Check logs."
                self._log_error(msg)
                return False, msg
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            msg = f"Error starting server: {e}"
            self._log_error(msg)
            return False, msg
    
    def stop_server(self) -> Tuple[bool, str]:
        """
        Stop the inference server.
        
        Returns
        -------
        tuple[bool, str]
            (success, message)
        
        """
        if not self.is_container_running():
            msg = f"Container '{self.CONTAINER_NAME}' is not running"
            self._log_error(msg)
            return False, msg
        
        if not self.is_server_running():
            msg = "Inference server is not running"
            self._log_info(msg)
            return True, msg
        
        self._log_info("Stopping inference server...")
        try:
            # Kill the process
            subprocess.run(
                ["docker", "exec", self.CONTAINER_NAME, "pkill", "-f", "server_inference_async.py"],
                check=True,
                timeout=5
            )
            
            # Wait and verify
            time.sleep(1)
            if not self.is_server_running():
                msg = "Inference server stopped successfully"
                self._log_info(msg)
                return True, msg
            else:
                msg = "Server may still be running"
                self._log_warn(msg)
                return False, msg
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            msg = f"Error stopping server: {e}"
            self._log_error(msg)
            return False, msg
    
    def get_status(self) -> dict:
        """
        Get server status.
        
        Returns
        -------
        dict
            Status information with keys: 'container_running', 'server_running'
        
        """
        container_running = self.is_container_running()
        server_running = self.is_server_running()
        
        return {
            'container_running': container_running,
            'server_running': server_running
        }
    
    def get_logs(self, lines: int = 50) -> Optional[str]:
        """
        Get server logs.
        
        Parameters
        ----------
        lines : int, optional
            Number of log lines to retrieve (default: 50)
        
        Returns
        -------
        str or None
            Log content or None if error
        
        """
        if not self.is_container_running():
            self._log_error(f"Container '{self.CONTAINER_NAME}' is not running")
            return None
        
        try:
            result = subprocess.run(
                ["docker", "exec", self.CONTAINER_NAME, "tail", "-n", str(lines), self.LOG_PATH],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                return result.stdout
            else:
                return None
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            return None
