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
# Author: Physical AI Team

import json
import os
import uuid
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, Optional

try:
    import zenoh
except ImportError:
    zenoh = None  # Zenoh not available - client will not work but dataclasses remain usable


class CommandType(Enum):
    TRAIN_START = "train_start"
    TRAIN_STOP = "train_stop"
    TRAIN_RESUME = "train_resume"
    TRAIN_STATUS = "train_status"
    INFER_START = "infer_start"
    INFER_STOP = "infer_stop"
    INFER_STATUS = "infer_status"
    MODEL_LOAD = "model_load"
    MODEL_UNLOAD = "model_unload"
    MODEL_LIST = "model_list"
    CHECKPOINT_LIST = "checkpoint_list"
    CHECKPOINT_INFO = "checkpoint_info"
    CHECKPOINT_DELETE = "checkpoint_delete"
    POLICY_LIST = "policy_list"


@dataclass
class LeRobotResponse:
    success: bool
    message: str
    data: Dict[str, Any]
    request_id: str

    @classmethod
    def from_json(cls, data: bytes) -> "LeRobotResponse":
        parsed = json.loads(data.decode('utf-8'))
        return cls(
            success=parsed.get("success", False),
            message=parsed.get("message", ""),
            data=parsed.get("data", {}),
            request_id=parsed.get("request_id", "")
        )


class ZenohLeRobotClient:
    """
    Client for communicating with LeRobot Docker container via Zenoh.
    
    Used within physical_ai_server to send training/inference commands
    to the isolated LeRobot container.
    """
    
    def __init__(
        self,
        router_ip: str = None,
        router_port: int = None,
        timeout_sec: float = 30.0
    ):
        self.router_ip = router_ip or os.environ.get("ZENOH_ROUTER_IP", "127.0.0.1")
        self.router_port = router_port or int(os.environ.get("ZENOH_ROUTER_PORT", "7447"))
        self.timeout_sec = timeout_sec
        
        self.session: Optional[zenoh.Session] = None
        self.command_key = "lerobot/command"
        self.status_key = "lerobot/status"
        self.action_key = "lerobot/action"
        self.training_log_key = "lerobot/training_log"
        
        self._status_callback: Optional[Callable] = None
        self._action_callback: Optional[Callable] = None
        self._training_log_callback: Optional[Callable] = None
        self._subscribers = []
        
    def connect(self) -> bool:
        try:
            config = zenoh.Config()
            endpoint = f"tcp/{self.router_ip}:{self.router_port}"
            config.insert_json5("connect/endpoints", f'["{endpoint}"]')
            
            shm_enabled = os.environ.get("ZENOH_SHM_ENABLED", "false").lower() == "true"
            if shm_enabled:
                config.insert_json5("transport/shared_memory/enabled", "true")
            
            self.session = zenoh.open(config)
            return True
        except Exception as e:
            print(f"[ZenohLeRobotClient] Connection failed: {e}")
            return False
    
    def disconnect(self):
        for sub in self._subscribers:
            sub.undeclare()
        self._subscribers.clear()
        
        if self.session:
            self.session.close()
            self.session = None
    
    def _send_command(
        self,
        command: str,
        params: Dict[str, Any]
    ) -> LeRobotResponse:
        if not self.session:
            return LeRobotResponse(
                success=False,
                message="Not connected to Zenoh",
                data={},
                request_id=""
            )
        
        request_id = str(uuid.uuid4())
        request_data = {
            "command": command,
            "params": params,
            "request_id": request_id
        }
        
        try:
            # New Zenoh API (1.x) - no Queue(), direct iteration
            replies = self.session.get(
                self.command_key,
                payload=json.dumps(request_data).encode('utf-8'),
                timeout=self.timeout_sec
            )
            
            for reply in replies:
                if reply.ok:
                    return LeRobotResponse.from_json(bytes(reply.ok.payload))
            
            return LeRobotResponse(
                success=False,
                message="No response from LeRobot server",
                data={},
                request_id=request_id
            )
            
        except Exception as e:
            return LeRobotResponse(
                success=False,
                message=f"Request failed: {str(e)}",
                data={},
                request_id=request_id
            )
    
    def start_training(
        self,
        policy_type: str,
        dataset_path: str,
        output_dir: str = None,
        num_epochs: int = None,
        batch_size: int = None,
        learning_rate: float = None,
        wandb_project: str = None
    ) -> LeRobotResponse:
        params = {
            "policy_type": policy_type,
            "dataset_path": dataset_path
        }
        if output_dir:
            params["output_dir"] = output_dir
        if num_epochs:
            params["num_epochs"] = num_epochs
        if batch_size:
            params["batch_size"] = batch_size
        if learning_rate:
            params["learning_rate"] = learning_rate
        if wandb_project:
            params["wandb_project"] = wandb_project
        
        return self._send_command(CommandType.TRAIN_START.value, params)
    
    def stop_training(self) -> LeRobotResponse:
        return self._send_command(CommandType.TRAIN_STOP.value, {})
    
    def resume_training(self, checkpoint_path: str) -> LeRobotResponse:
        return self._send_command(
            CommandType.TRAIN_RESUME.value,
            {"checkpoint_path": checkpoint_path}
        )
    
    def get_training_status(self) -> LeRobotResponse:
        return self._send_command(CommandType.TRAIN_STATUS.value, {})
    
    def start_inference(
        self,
        model_path: str,
        env_name: str = None,
        num_episodes: int = None
    ) -> LeRobotResponse:
        params = {"model_path": model_path}
        if env_name:
            params["env_name"] = env_name
        if num_episodes:
            params["num_episodes"] = num_episodes
        
        return self._send_command(CommandType.INFER_START.value, params)
    
    def stop_inference(self) -> LeRobotResponse:
        return self._send_command(CommandType.INFER_STOP.value, {})
    
    def get_inference_status(self) -> LeRobotResponse:
        return self._send_command(CommandType.INFER_STATUS.value, {})
    
    def load_model(self, model_path: str) -> LeRobotResponse:
        return self._send_command(
            CommandType.MODEL_LOAD.value,
            {"model_path": model_path}
        )
    
    def unload_model(self) -> LeRobotResponse:
        return self._send_command(CommandType.MODEL_UNLOAD.value, {})
    
    def list_models(self) -> LeRobotResponse:
        return self._send_command(CommandType.MODEL_LIST.value, {})
    
    def subscribe_status(self, callback: Callable[[Dict], None]):
        if not self.session:
            return False
        
        self._status_callback = callback
        
        def on_status(sample):
            try:
                data = json.loads(bytes(sample.payload).decode('utf-8'))
                callback(data)
            except Exception as e:
                print(f"[ZenohLeRobotClient] Status parse error: {e}")
        
        sub = self.session.declare_subscriber(self.status_key, on_status)
        self._subscribers.append(sub)
        return True
    
    def subscribe_actions(self, callback: Callable[[Dict], None]):
        if not self.session:
            return False
        
        self._action_callback = callback
        
        def on_action(sample):
            try:
                data = json.loads(bytes(sample.payload).decode('utf-8'))
                callback(data)
            except Exception as e:
                print(f"[ZenohLeRobotClient] Action parse error: {e}")
        
        sub = self.session.declare_subscriber(self.action_key, on_action)
        self._subscribers.append(sub)
        return True
    
    def subscribe_training_log(self, callback: Callable[[Dict], None]):
        """Subscribe to detailed training log updates"""
        if not self.session:
            return False
        
        self._training_log_callback = callback
        
        def on_training_log(sample):
            try:
                data = json.loads(bytes(sample.payload).decode('utf-8'))
                callback(data)
            except Exception as e:
                print(f"[ZenohLeRobotClient] Training log parse error: {e}")
        
        sub = self.session.declare_subscriber(self.training_log_key, on_training_log)
        self._subscribers.append(sub)
        return True
    
    # ========== Checkpoint Management ==========
    
    def list_checkpoints(self) -> LeRobotResponse:
        """
        List all available checkpoints from training runs.
        
        Returns:
            LeRobotResponse with data containing:
            - checkpoints: List of checkpoint info dicts with:
                - run_name: Training run name
                - checkpoint_name: Checkpoint identifier (step number or 'last')
                - path: Full path to pretrained_model directory
                - created_at: ISO timestamp
                - policy_type: Model type (act, diffusion, etc.)
                - dataset: Dataset used for training
                - step: Training step number
                - size_mb: Checkpoint size in MB
                - is_latest: True if this is the 'last' checkpoint
        """
        return self._send_command(CommandType.CHECKPOINT_LIST.value, {})
    
    def get_checkpoint_info(self, checkpoint_path: str) -> LeRobotResponse:
        """
        Get detailed information about a specific checkpoint.
        
        Args:
            checkpoint_path: Path to the pretrained_model directory
            
        Returns:
            LeRobotResponse with detailed checkpoint metadata including:
            - config: Full model configuration
            - files: List of model files with sizes
            - All fields from list_checkpoints
        """
        return self._send_command(
            CommandType.CHECKPOINT_INFO.value,
            {"checkpoint_path": checkpoint_path}
        )
    
    def delete_checkpoint(self, checkpoint_path: str) -> LeRobotResponse:
        """
        Delete a checkpoint.
        
        Args:
            checkpoint_path: Path to the pretrained_model directory
            
        Returns:
            LeRobotResponse indicating success/failure
            
        Note:
            Only checkpoints within the outputs directory can be deleted.
            The entire checkpoint directory (parent of pretrained_model) is removed.
        """
        return self._send_command(
            CommandType.CHECKPOINT_DELETE.value,
            {"checkpoint_path": checkpoint_path}
        )
    
    def get_policy_list(self, category: str = None) -> LeRobotResponse:
        params = {}
        if category:
            params["category"] = category
        return self._send_command(CommandType.POLICY_LIST.value, params)
