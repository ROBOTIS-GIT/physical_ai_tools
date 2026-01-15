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

import zenoh


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
        
        self._status_callback: Optional[Callable] = None
        self._action_callback: Optional[Callable] = None
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
            replies = self.session.get(
                self.command_key,
                zenoh.Queue(),
                payload=json.dumps(request_data).encode('utf-8'),
                timeout=self.timeout_sec
            )
            
            for reply in replies.receiver:
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
