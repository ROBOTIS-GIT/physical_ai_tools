#!/usr/bin/env python3
"""
Control isaac_gr00t inference server from physical_ai_server container.

This script allows starting/stopping the server_inference_async.py script
running in the isaac_gr00t container.
"""

import subprocess
import sys
import time


class IsaacGr00tController:
    """Controller for isaac_gr00t inference server."""
    
    CONTAINER_NAME = "isaac_gr00t"
    SCRIPT_PATH = "/workspace/scripts/server_inference_async.py"
    
    def __init__(self):
        """Initialize the controller."""
        self.check_docker_available()
    
    def check_docker_available(self):
        """Check if docker command is available."""
        try:
            subprocess.run(
                ["docker", "--version"],
                check=True,
                capture_output=True,
                text=True
            )
        except FileNotFoundError:
            print("Error: Docker CLI not found. Please install docker CLI in this container.")
            print("Run: apt-get update && apt-get install -y docker.io")
            sys.exit(1)
        except subprocess.CalledProcessError as e:
            print(f"Error checking docker: {e}")
            sys.exit(1)
    
    def is_container_running(self):
        """Check if isaac_gr00t container is running."""
        try:
            result = subprocess.run(
                ["docker", "ps", "--filter", f"name={self.CONTAINER_NAME}", "--format", "{{.Names}}"],
                check=True,
                capture_output=True,
                text=True
            )
            return self.CONTAINER_NAME in result.stdout
        except subprocess.CalledProcessError:
            return False
    
    def is_server_running(self):
        """Check if server_inference_async.py is running in the container."""
        if not self.is_container_running():
            return False
        
        try:
            result = subprocess.run(
                ["docker", "exec", self.CONTAINER_NAME, "pgrep", "-f", "server_inference_async.py"],
                capture_output=True,
                text=True
            )
            return result.returncode == 0 and result.stdout.strip() != ""
        except subprocess.CalledProcessError:
            return False
    
    def start_server(self, host="localhost", port=5555):
        """Start the inference server."""
        if not self.is_container_running():
            print(f"Error: Container '{self.CONTAINER_NAME}' is not running")
            return False
        
        if self.is_server_running():
            print("Server is already running")
            return True
        
        print(f"Starting inference server on {host}:{port}...")
        try:
            # Start server in background using nohup
            cmd = [
                "docker", "exec", "-d", self.CONTAINER_NAME,
                "bash", "-c",
                f"cd /workspace && nohup python3 {self.SCRIPT_PATH} --host {host} --port {port} > /tmp/inference_server.log 2>&1 &"
            ]
            subprocess.run(cmd, check=True)
            
            # Wait a bit and check if it started
            time.sleep(2)
            if self.is_server_running():
                print("✅ Server started successfully")
                return True
            else:
                print("❌ Server failed to start. Check logs with: docker exec isaac_gr00t cat /tmp/inference_server.log")
                return False
        except subprocess.CalledProcessError as e:
            print(f"Error starting server: {e}")
            return False
    
    def stop_server(self):
        """Stop the inference server."""
        if not self.is_container_running():
            print(f"Error: Container '{self.CONTAINER_NAME}' is not running")
            return False
        
        if not self.is_server_running():
            print("Server is not running")
            return True
        
        print("Stopping inference server...")
        try:
            # Kill the process
            subprocess.run(
                ["docker", "exec", self.CONTAINER_NAME, "pkill", "-f", "server_inference_async.py"],
                check=True
            )
            
            # Wait a bit and verify
            time.sleep(1)
            if not self.is_server_running():
                print("✅ Server stopped successfully")
                return True
            else:
                print("❌ Server may still be running")
                return False
        except subprocess.CalledProcessError as e:
            print(f"Error stopping server: {e}")
            return False
    
    def status(self):
        """Get server status."""
        container_running = self.is_container_running()
        server_running = self.is_server_running()
        
        print(f"Container '{self.CONTAINER_NAME}': {'✅ Running' if container_running else '❌ Not running'}")
        print(f"Inference Server: {'✅ Running' if server_running else '❌ Not running'}")
        
        return container_running and server_running
    
    def logs(self, lines=50):
        """Show server logs."""
        if not self.is_container_running():
            print(f"Error: Container '{self.CONTAINER_NAME}' is not running")
            return
        
        try:
            result = subprocess.run(
                ["docker", "exec", self.CONTAINER_NAME, "tail", "-n", str(lines), "/tmp/inference_server.log"],
                capture_output=True,
                text=True
            )
            if result.returncode == 0:
                print(result.stdout)
            else:
                print("No logs found or error reading logs")
        except subprocess.CalledProcessError as e:
            print(f"Error reading logs: {e}")


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Control isaac_gr00t inference server")
    parser.add_argument(
        "command",
        choices=["start", "stop", "status", "restart", "logs"],
        help="Command to execute"
    )
    parser.add_argument("--host", default="localhost", help="Server host (default: localhost)")
    parser.add_argument("--port", type=int, default=5555, help="Server port (default: 5555)")
    parser.add_argument("--lines", type=int, default=50, help="Number of log lines to show (default: 50)")
    
    args = parser.parse_args()
    
    controller = IsaacGr00tController()
    
    if args.command == "start":
        controller.start_server(args.host, args.port)
    elif args.command == "stop":
        controller.stop_server()
    elif args.command == "status":
        controller.status()
    elif args.command == "restart":
        controller.stop_server()
        time.sleep(1)
        controller.start_server(args.host, args.port)
    elif args.command == "logs":
        controller.logs(args.lines)


if __name__ == "__main__":
    main()
