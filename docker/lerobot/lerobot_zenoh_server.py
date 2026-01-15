#!/usr/bin/env python3
"""
LeRobot Zenoh Server - Receives training/inference commands via Zenoh

This server runs inside the LeRobot Docker container and communicates with
physical_ai_server (ROS2) through Zenoh.

Supported commands:
- Training: Start, stop, resume, status
- Inference: Start, stop, status
- Model management: Load, unload, list

Communication flow:
  physical_ai_server (ROS2 + Zenoh) <--Zenoh--> lerobot_zenoh_server (Docker)
"""

import os
import sys
import json
import signal
import logging
import threading
import subprocess
from pathlib import Path
from typing import Optional, Dict, Any
from dataclasses import dataclass, asdict
from enum import Enum

# Add zenoh_ros2_sdk to path if mounted as volume
zenoh_sdk_path = os.environ.get("ZENOH_SDK_PATH", "/zenoh_sdk")
if os.path.exists(zenoh_sdk_path):
    sys.path.insert(0, zenoh_sdk_path)

import zenoh

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("lerobot_zenoh_server")


class CommandType(Enum):
    """Supported command types"""
    # Training commands
    TRAIN_START = "train_start"
    TRAIN_STOP = "train_stop"
    TRAIN_RESUME = "train_resume"
    TRAIN_STATUS = "train_status"
    
    # Inference commands
    INFER_START = "infer_start"
    INFER_STOP = "infer_stop"
    INFER_STATUS = "infer_status"
    
    # Model commands
    MODEL_LOAD = "model_load"
    MODEL_UNLOAD = "model_unload"
    MODEL_LIST = "model_list"


class TaskStatus(Enum):
    """Task execution status"""
    IDLE = "idle"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    STOPPED = "stopped"


@dataclass
class CommandRequest:
    """Request message structure"""
    command: str
    params: Dict[str, Any]
    request_id: str = ""
    
    @classmethod
    def from_json(cls, data: bytes) -> "CommandRequest":
        """Parse from JSON bytes"""
        parsed = json.loads(data.decode('utf-8'))
        return cls(
            command=parsed.get("command", ""),
            params=parsed.get("params", {}),
            request_id=parsed.get("request_id", "")
        )


@dataclass
class CommandResponse:
    """Response message structure"""
    success: bool
    message: str
    data: Dict[str, Any] = None
    request_id: str = ""
    
    def to_json(self) -> bytes:
        """Serialize to JSON bytes"""
        result = {
            "success": self.success,
            "message": self.message,
            "data": self.data or {},
            "request_id": self.request_id
        }
        return json.dumps(result).encode('utf-8')


class LeRobotZenohServer:
    """
    Zenoh server for LeRobot - handles training and inference commands
    """
    
    def __init__(
        self,
        router_ip: str = None,
        router_port: int = None,
        shm_enabled: bool = None
    ):
        """
        Initialize the Zenoh server
        
        Args:
            router_ip: Zenoh router IP (default from env)
            router_port: Zenoh router port (default from env)
            shm_enabled: Enable shared memory transport (default from env)
        """
        self.router_ip = router_ip or os.environ.get("ZENOH_ROUTER_IP", "127.0.0.1")
        self.router_port = router_port or int(os.environ.get("ZENOH_ROUTER_PORT", "7447"))
        self.shm_enabled = shm_enabled if shm_enabled is not None else \
            os.environ.get("ZENOH_SHM_ENABLED", "false").lower() == "true"
        
        self.session: Optional[zenoh.Session] = None
        self.queryable = None
        
        # Task management
        self.current_task: Optional[subprocess.Popen] = None
        self.task_status = TaskStatus.IDLE
        self.task_type: Optional[str] = None
        self.task_lock = threading.Lock()
        
        # Zenoh key expressions
        self.command_key = "lerobot/command"
        self.status_key = "lerobot/status"
        self.action_key = "lerobot/action"  # For inference output (Mode 1)
        
        self._running = False
        
    def _create_config(self) -> zenoh.Config:
        """Create Zenoh configuration"""
        config = zenoh.Config()
        
        # Set connection endpoint
        endpoint = f"tcp/{self.router_ip}:{self.router_port}"
        config.insert_json5("connect/endpoints", f'["{endpoint}"]')
        
        # Enable shared memory if configured
        if self.shm_enabled:
            logger.info("Enabling Zenoh Shared Memory transport")
            config.insert_json5("transport/shared_memory/enabled", "true")
        
        return config
    
    def start(self):
        """Start the Zenoh server"""
        logger.info(f"Starting LeRobot Zenoh Server")
        logger.info(f"  Router: {self.router_ip}:{self.router_port}")
        logger.info(f"  SHM Enabled: {self.shm_enabled}")
        
        # Create Zenoh session
        config = self._create_config()
        self.session = zenoh.open(config)
        logger.info("Zenoh session opened")
        
        # Create queryable for receiving commands (RPC-style)
        self.queryable = self.session.declare_queryable(
            self.command_key,
            self._handle_command
        )
        logger.info(f"Listening for commands on: {self.command_key}")
        
        # Create publisher for status updates
        self.status_publisher = self.session.declare_publisher(self.status_key)
        logger.info(f"Publishing status on: {self.status_key}")
        
        # Create publisher for action output (inference results)
        self.action_publisher = self.session.declare_publisher(self.action_key)
        logger.info(f"Publishing actions on: {self.action_key}")
        
        self._running = True
        self._publish_status()
        
        logger.info("LeRobot Zenoh Server started successfully")
    
    def stop(self):
        """Stop the Zenoh server"""
        logger.info("Stopping LeRobot Zenoh Server")
        self._running = False
        
        # Stop any running task
        self._stop_current_task()
        
        # Close Zenoh resources
        if self.queryable:
            self.queryable.undeclare()
        if hasattr(self, 'status_publisher'):
            self.status_publisher.undeclare()
        if hasattr(self, 'action_publisher'):
            self.action_publisher.undeclare()
        if self.session:
            self.session.close()
        
        logger.info("LeRobot Zenoh Server stopped")
    
    def _handle_command(self, query: zenoh.Query):
        """Handle incoming command query"""
        try:
            # Parse request
            payload = query.payload
            if payload is None:
                response = CommandResponse(
                    success=False,
                    message="Empty request payload"
                )
            else:
                request = CommandRequest.from_json(bytes(payload))
                logger.info(f"Received command: {request.command}")
                
                # Route to appropriate handler
                response = self._route_command(request)
                response.request_id = request.request_id
            
            # Send response
            query.reply(query.key_expr, response.to_json())
            
        except Exception as e:
            logger.error(f"Error handling command: {e}")
            response = CommandResponse(
                success=False,
                message=f"Error: {str(e)}"
            )
            query.reply(query.key_expr, response.to_json())
    
    def _route_command(self, request: CommandRequest) -> CommandResponse:
        """Route command to appropriate handler"""
        handlers = {
            # Training
            CommandType.TRAIN_START.value: self._handle_train_start,
            CommandType.TRAIN_STOP.value: self._handle_train_stop,
            CommandType.TRAIN_RESUME.value: self._handle_train_resume,
            CommandType.TRAIN_STATUS.value: self._handle_status,
            
            # Inference
            CommandType.INFER_START.value: self._handle_infer_start,
            CommandType.INFER_STOP.value: self._handle_infer_stop,
            CommandType.INFER_STATUS.value: self._handle_status,
            
            # Model
            CommandType.MODEL_LOAD.value: self._handle_model_load,
            CommandType.MODEL_UNLOAD.value: self._handle_model_unload,
            CommandType.MODEL_LIST.value: self._handle_model_list,
        }
        
        handler = handlers.get(request.command)
        if handler is None:
            return CommandResponse(
                success=False,
                message=f"Unknown command: {request.command}"
            )
        
        return handler(request.params)
    
    def _handle_train_start(self, params: Dict[str, Any]) -> CommandResponse:
        """Handle training start command"""
        with self.task_lock:
            if self.task_status == TaskStatus.RUNNING:
                return CommandResponse(
                    success=False,
                    message=f"Another task is already running: {self.task_type}"
                )
            
            # Required parameters
            policy_type = params.get("policy_type", "act")
            dataset_path = params.get("dataset_path")
            
            if not dataset_path:
                return CommandResponse(
                    success=False,
                    message="Missing required parameter: dataset_path"
                )
            
            # Build training command
            cmd = [
                "python", "-m", "lerobot.scripts.train",
                f"--policy.type={policy_type}",
                f"--dataset.repo_id={dataset_path}",
            ]
            
            # Add optional parameters
            if "output_dir" in params:
                cmd.append(f"--output_dir={params['output_dir']}")
            if "num_epochs" in params:
                cmd.append(f"--training.num_epochs={params['num_epochs']}")
            if "batch_size" in params:
                cmd.append(f"--training.batch_size={params['batch_size']}")
            if "learning_rate" in params:
                cmd.append(f"--training.lr={params['learning_rate']}")
            if "wandb_project" in params:
                cmd.append(f"--wandb.project={params['wandb_project']}")
            
            # Start training process
            try:
                logger.info(f"Starting training: {' '.join(cmd)}")
                self.current_task = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True
                )
                self.task_status = TaskStatus.RUNNING
                self.task_type = "training"
                
                # Start monitoring thread
                threading.Thread(
                    target=self._monitor_task,
                    daemon=True
                ).start()
                
                return CommandResponse(
                    success=True,
                    message="Training started",
                    data={"pid": self.current_task.pid}
                )
                
            except Exception as e:
                logger.error(f"Failed to start training: {e}")
                return CommandResponse(
                    success=False,
                    message=f"Failed to start training: {str(e)}"
                )
    
    def _handle_train_stop(self, params: Dict[str, Any]) -> CommandResponse:
        """Handle training stop command"""
        return self._stop_current_task()
    
    def _handle_train_resume(self, params: Dict[str, Any]) -> CommandResponse:
        """Handle training resume command"""
        checkpoint_path = params.get("checkpoint_path")
        if not checkpoint_path:
            return CommandResponse(
                success=False,
                message="Missing required parameter: checkpoint_path"
            )
        
        # Add checkpoint to params and start training
        params["resume_from_checkpoint"] = checkpoint_path
        return self._handle_train_start(params)
    
    def _handle_infer_start(self, params: Dict[str, Any]) -> CommandResponse:
        """Handle inference start command"""
        with self.task_lock:
            if self.task_status == TaskStatus.RUNNING:
                return CommandResponse(
                    success=False,
                    message=f"Another task is already running: {self.task_type}"
                )
            
            # Required parameters
            model_path = params.get("model_path")
            
            if not model_path:
                return CommandResponse(
                    success=False,
                    message="Missing required parameter: model_path"
                )
            
            # Build inference command
            cmd = [
                "python", "-m", "lerobot.scripts.eval",
                f"--policy.path={model_path}",
            ]
            
            # Add optional parameters
            if "env_name" in params:
                cmd.append(f"--env.name={params['env_name']}")
            if "num_episodes" in params:
                cmd.append(f"--eval.num_episodes={params['num_episodes']}")
            
            # Start inference process
            try:
                logger.info(f"Starting inference: {' '.join(cmd)}")
                self.current_task = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True
                )
                self.task_status = TaskStatus.RUNNING
                self.task_type = "inference"
                
                # Start monitoring thread
                threading.Thread(
                    target=self._monitor_task,
                    daemon=True
                ).start()
                
                return CommandResponse(
                    success=True,
                    message="Inference started",
                    data={"pid": self.current_task.pid}
                )
                
            except Exception as e:
                logger.error(f"Failed to start inference: {e}")
                return CommandResponse(
                    success=False,
                    message=f"Failed to start inference: {str(e)}"
                )
    
    def _handle_infer_stop(self, params: Dict[str, Any]) -> CommandResponse:
        """Handle inference stop command"""
        return self._stop_current_task()
    
    def _handle_status(self, params: Dict[str, Any]) -> CommandResponse:
        """Handle status query"""
        with self.task_lock:
            return CommandResponse(
                success=True,
                message="Status retrieved",
                data={
                    "status": self.task_status.value,
                    "task_type": self.task_type,
                    "pid": self.current_task.pid if self.current_task else None
                }
            )
    
    def _handle_model_load(self, params: Dict[str, Any]) -> CommandResponse:
        """Handle model load command"""
        model_path = params.get("model_path")
        if not model_path:
            return CommandResponse(
                success=False,
                message="Missing required parameter: model_path"
            )
        
        # TODO: Implement model preloading for faster inference
        return CommandResponse(
            success=True,
            message=f"Model loading not yet implemented: {model_path}"
        )
    
    def _handle_model_unload(self, params: Dict[str, Any]) -> CommandResponse:
        """Handle model unload command"""
        # TODO: Implement model unloading
        return CommandResponse(
            success=True,
            message="Model unloading not yet implemented"
        )
    
    def _handle_model_list(self, params: Dict[str, Any]) -> CommandResponse:
        """Handle model list command"""
        # List available models in cache
        cache_dir = Path(os.environ.get("HF_LEROBOT_HOME", "/root/.cache/huggingface/lerobot"))
        
        models = []
        if cache_dir.exists():
            for item in cache_dir.iterdir():
                if item.is_dir():
                    models.append(item.name)
        
        return CommandResponse(
            success=True,
            message=f"Found {len(models)} models",
            data={"models": models}
        )
    
    def _stop_current_task(self) -> CommandResponse:
        """Stop the currently running task"""
        with self.task_lock:
            if self.current_task is None or self.task_status != TaskStatus.RUNNING:
                return CommandResponse(
                    success=False,
                    message="No task is currently running"
                )
            
            try:
                self.current_task.terminate()
                self.current_task.wait(timeout=10)
                self.task_status = TaskStatus.STOPPED
                
                return CommandResponse(
                    success=True,
                    message=f"{self.task_type} stopped"
                )
                
            except subprocess.TimeoutExpired:
                self.current_task.kill()
                self.task_status = TaskStatus.STOPPED
                return CommandResponse(
                    success=True,
                    message=f"{self.task_type} force killed"
                )
                
            except Exception as e:
                return CommandResponse(
                    success=False,
                    message=f"Failed to stop task: {str(e)}"
                )
    
    def _monitor_task(self):
        """Monitor the current task and update status"""
        if self.current_task is None:
            return
        
        try:
            # Read output and wait for completion
            for line in self.current_task.stdout:
                logger.info(f"[{self.task_type}] {line.rstrip()}")
            
            return_code = self.current_task.wait()
            
            with self.task_lock:
                if return_code == 0:
                    self.task_status = TaskStatus.COMPLETED
                    logger.info(f"{self.task_type} completed successfully")
                else:
                    self.task_status = TaskStatus.FAILED
                    logger.error(f"{self.task_type} failed with code {return_code}")
                
                self._publish_status()
                
        except Exception as e:
            logger.error(f"Error monitoring task: {e}")
            with self.task_lock:
                self.task_status = TaskStatus.FAILED
    
    def _publish_status(self):
        """Publish current status to status topic"""
        if not hasattr(self, 'status_publisher'):
            return
        
        status_data = {
            "status": self.task_status.value,
            "task_type": self.task_type,
            "timestamp": str(os.popen('date -Iseconds').read().strip())
        }
        
        try:
            self.status_publisher.put(json.dumps(status_data).encode('utf-8'))
        except Exception as e:
            logger.error(f"Failed to publish status: {e}")
    
    def run_forever(self):
        """Run the server until interrupted"""
        self.start()
        
        # Set up signal handlers
        def signal_handler(signum, frame):
            logger.info(f"Received signal {signum}, shutting down...")
            self.stop()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Keep running
        logger.info("Server running. Press Ctrl+C to stop.")
        while self._running:
            try:
                # Periodic status publish
                self._publish_status()
                threading.Event().wait(timeout=5.0)
            except KeyboardInterrupt:
                break
        
        self.stop()


def main():
    """Main entry point"""
    server = LeRobotZenohServer()
    server.run_forever()


if __name__ == "__main__":
    main()
