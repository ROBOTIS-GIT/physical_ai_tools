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

"""
ContainerServiceClient - Unified ROS2 Service Client for container communication.

Generic client that works with any container (GR00T, LeRobot, etc.)
by parameterizing the service prefix (e.g., "/groot", "/lerobot").

Supports both inference and training services:
  - /{prefix}/infer             (StartInference)
  - /{prefix}/get_action_chunk  (GetActionChunk)
  - /{prefix}/train             (TrainModel)
  - /{prefix}/stop              (StopTraining)
  - /{prefix}/status            (TrainingStatus)
"""

from dataclasses import dataclass
import json
import logging
import socket
import struct
import threading
import time
from typing import Any, Callable, Dict, Optional

from physical_ai_interfaces.msg import TrainingProgress
from physical_ai_interfaces.srv import (
    GetActionChunk,
    StartInference,
    StopTraining,
    TrainingStatus,
    TrainModel,
)
from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

logger = logging.getLogger(__name__)


@dataclass
class ServiceResponse:
    success: bool
    message: str
    data: Dict[str, Any]
    request_id: str

    @classmethod
    def from_service_response(
        cls, response: Any, request_id: str = ''
    ) -> 'ServiceResponse':
        if response is None:
            return cls(
                success=False,
                message='No response from service (timeout or error)',
                data={},
                request_id=request_id
            )
        return cls(
            success=getattr(response, 'success', False),
            message=getattr(response, 'message', ''),
            data=cls._extract_data(response),
            request_id=request_id
        )

    @staticmethod
    def _extract_data(response: Any) -> Dict[str, Any]:
        data = {}
        for attr in [
            'job_id', 'state', 'step', 'total_steps', 'loss',
            'learning_rate', 'chunk_size', 'action_dim',
        ]:
            if hasattr(response, attr):
                data[attr] = getattr(response, attr)
        for attr in [
            'policies', 'checkpoints', 'models',
            'action_chunk', 'action_keys',
        ]:
            if hasattr(response, attr):
                data[attr] = list(getattr(response, attr))
        return data



class ContainerServiceClient:
    """Unified ROS2 service client for inference/training containers.

    Works with any container (GR00T, LeRobot, etc.) that implements
    the standard service interface, parameterized by service_prefix.
    """

    # Container name → TCP bridge port mapping
    BRIDGE_PORTS = {
        "/groot": 9100,
        "/lerobot": 9101,
    }

    def __init__(
        self,
        node: Node,
        service_prefix: str = "/groot",
        timeout_sec: float = 180.0,
        callback_group: Optional[CallbackGroup] = None,
        bridge_host: str = "127.0.0.1",
    ):
        self._node = node
        self._service_prefix = service_prefix
        self.timeout_sec = timeout_sec
        self._connected = False
        self._cancelled = threading.Event()
        self._callback_group = callback_group

        # TCP bridge connection (bypasses rmw_zenoh_cpp)
        self._bridge_host = bridge_host
        self._bridge_port = self.BRIDGE_PORTS.get(service_prefix, 9100)
        self._bridge_sock = None
        self._bridge_lock = threading.Lock()

        # ROS2 service clients (for training services only)
        self._infer_client = None
        self._stop_client = None
        self._action_chunk_client = None
        self._train_client = None
        self._status_client = None

        # Subscribers
        self._progress_sub = None

    # --- Service name properties ---

    @property
    def service_infer(self) -> str:
        return f"{self._service_prefix}/infer"

    @property
    def service_stop(self) -> str:
        return f"{self._service_prefix}/stop"

    @property
    def service_get_action_chunk(self) -> str:
        return f"{self._service_prefix}/get_action_chunk"

    @property
    def service_train(self) -> str:
        return f"{self._service_prefix}/train"

    @property
    def service_status(self) -> str:
        return f"{self._service_prefix}/status"

    @property
    def topic_progress(self) -> str:
        return f"{self._service_prefix}/progress"

    # --- Connection management ---

    def connect(self) -> bool:
        """Create ROS2 service clients for container communication."""
        if self._connected:
            return True

        if self._node is None:
            logger.error("No ROS2 node provided")
            return False

        try:
            if self._callback_group is None:
                self._callback_group = ReentrantCallbackGroup()

            self._infer_client = self._node.create_client(
                StartInference,
                self.service_infer,
                callback_group=self._callback_group,
            )
            self._stop_client = self._node.create_client(
                StopTraining,
                self.service_stop,
                callback_group=self._callback_group,
            )
            self._action_chunk_client = self._node.create_client(
                GetActionChunk,
                self.service_get_action_chunk,
                callback_group=self._callback_group,
            )
            self._train_client = self._node.create_client(
                TrainModel,
                self.service_train,
                callback_group=self._callback_group,
            )
            self._status_client = self._node.create_client(
                TrainingStatus,
                self.service_status,
                callback_group=self._callback_group,
            )

            self._connected = True
            logger.info(
                f"Connected to container services "
                f"(prefix={self._service_prefix})"
            )
            return True
        except Exception as e:
            logger.error(f"Service client connection failed: {e}")
            return False

    def disconnect(self):
        """Destroy all service clients, subscribers, and bridge connection."""
        self._bridge_disconnect()

        if self._progress_sub is not None:
            try:
                self._node.destroy_subscription(self._progress_sub)
            except Exception:
                pass
            self._progress_sub = None

        clients = [
            ("_infer_client", self._infer_client),
            ("_stop_client", self._stop_client),
            ("_action_chunk_client", self._action_chunk_client),
            ("_train_client", self._train_client),
            ("_status_client", self._status_client),
        ]
        for name, client in clients:
            if client is not None:
                try:
                    self._node.destroy_client(client)
                except Exception as e:
                    logger.debug(f"Error destroying client {name}: {e}")

        self._infer_client = None
        self._stop_client = None
        self._action_chunk_client = None
        self._train_client = None
        self._status_client = None
        self._connected = False

    # --- Core service call ---

    def _call_service(
        self, client, request, service_name: str, timeout_sec: float = None
    ) -> ServiceResponse:
        """Call a ROS2 service and return ServiceResponse."""
        if not self._connected:
            return ServiceResponse(
                success=False,
                message="Not connected to container services",
                data={},
                request_id="",
            )

        if client is None:
            return ServiceResponse(
                success=False,
                message=f"Service client not initialized: {service_name}",
                data={},
                request_id="",
            )

        timeout = timeout_sec or self.timeout_sec

        try:
            if not client.wait_for_service(timeout_sec=5.0):
                return ServiceResponse(
                    success=False,
                    message=f"Service not available: {service_name}",
                    data={},
                    request_id="",
                )

            logger.debug(f"Calling service {service_name}")
            future = client.call_async(request)

            # Wait for the executor to resolve the future.
            # MultiThreadedExecutor + dedicated client callback group ensures
            # the response is processed on a separate thread, so this never
            # deadlocks even when called from a service callback.
            done_event = threading.Event()
            future.add_done_callback(lambda _: done_event.set())

            # Wait in short intervals so we can also check _cancelled.
            deadline = time.perf_counter() + timeout
            while not done_event.is_set():
                if self._cancelled.is_set():
                    break
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    break
                done_event.wait(timeout=min(remaining, 1.0))

            if future.done():
                response = future.result()
                if response is not None:
                    return ServiceResponse.from_service_response(response)
                else:
                    return ServiceResponse(
                        success=False,
                        message=f"Service call returned None: {service_name}",
                        data={},
                        request_id="",
                    )
            else:
                future.cancel()
                reason = "cancelled" if self._cancelled.is_set() else "timed out"
                return ServiceResponse(
                    success=False,
                    message=f"Service call {reason}: {service_name}",
                    data={},
                    request_id="",
                )
        except Exception as e:
            logger.error(f"Service call to {service_name} failed: {e}")
            return ServiceResponse(
                success=False,
                message=f"Service call failed: {str(e)}",
                data={},
                request_id="",
            )

    # --- TCP Bridge communication ---

    def _bridge_connect(self):
        """Connect to the TCP bridge server in the executor container."""
        if self._bridge_sock is not None:
            return
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(600.0)
            sock.connect((self._bridge_host, self._bridge_port))
            self._bridge_sock = sock
            logger.info(
                f"Bridge connected: {self._bridge_host}:{self._bridge_port}"
            )
        except Exception as e:
            self._bridge_sock = None
            raise ConnectionError(
                f"Bridge connect failed ({self._bridge_host}:{self._bridge_port}): {e}"
            )

    def _bridge_disconnect(self):
        """Close the TCP bridge connection."""
        if self._bridge_sock is not None:
            try:
                self._bridge_sock.close()
            except Exception:
                pass
            self._bridge_sock = None

    def _bridge_call(self, request: dict, timeout: float = 600.0) -> dict:
        """Send a request to the TCP bridge and return the response."""
        with self._bridge_lock:
            try:
                if self._bridge_sock is None:
                    self._bridge_connect()
                self._bridge_sock.settimeout(timeout)

                # Send: 4-byte length + JSON
                payload = json.dumps(request).encode("utf-8")
                self._bridge_sock.sendall(
                    struct.pack(">I", len(payload)) + payload
                )

                # Recv: 4-byte length + JSON
                header = self._recv_exact(self._bridge_sock, 4)
                if not header:
                    self._bridge_disconnect()
                    return {"success": False, "message": "Bridge connection closed"}
                resp_len = struct.unpack(">I", header)[0]
                resp_data = self._recv_exact(self._bridge_sock, resp_len)
                if not resp_data:
                    self._bridge_disconnect()
                    return {"success": False, "message": "Bridge response incomplete"}
                return json.loads(resp_data)
            except Exception as e:
                self._bridge_disconnect()
                return {"success": False, "message": f"Bridge call failed: {e}"}

    @staticmethod
    def _recv_exact(sock, n):
        """Receive exactly n bytes from socket."""
        buf = b""
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    # --- Inference services (via TCP bridge) ---

    def start_inference(
        self,
        model_path: str,
        embodiment_tag: str,
        robot_type: str,
        task_instruction: str = "",
    ) -> ServiceResponse:
        """Call /{prefix}/infer via TCP bridge."""
        result = self._bridge_call({
            "action": "start_inference",
            "service_prefix": self._service_prefix,
            "model_path": model_path,
            "embodiment_tag": embodiment_tag,
            "robot_type": robot_type,
            "task_instruction": task_instruction,
        }, timeout=600.0)
        return ServiceResponse(
            success=result.get("success", False),
            message=result.get("message", ""),
            data={"action_keys": result.get("action_keys", [])},
            request_id="",
        )

    def get_action_chunk(self, task_instruction: str = "") -> ServiceResponse:
        """Call /{prefix}/get_action_chunk via TCP bridge."""
        result = self._bridge_call({
            "action": "get_action_chunk",
            "service_prefix": self._service_prefix,
            "task_instruction": task_instruction,
        }, timeout=5.0)
        return ServiceResponse(
            success=result.get("success", False),
            message=result.get("message", ""),
            data={
                "action_chunk": result.get("action_chunk", []),
                "chunk_size": result.get("chunk_size", 0),
                "action_dim": result.get("action_dim", 0),
            },
            request_id="",
        )

    def stop_inference(self) -> ServiceResponse:
        """Call /{prefix}/stop via TCP bridge."""
        result = self._bridge_call({
            "action": "stop",
            "service_prefix": self._service_prefix,
        })
        return ServiceResponse(
            success=result.get("success", False),
            message=result.get("message", ""),
            data={},
            request_id="",
        )

    # --- Training services ---

    def start_training(
        self,
        policy_type: str,
        dataset_path: str,
        output_dir: str = '',
        num_epochs: int = 0,
        batch_size: int = 0,
        learning_rate: float = 0.0,
        eval_freq: int = 0,
        log_freq: int = 0,
        save_freq: int = 0,
        wandb_project: str = '',
    ) -> ServiceResponse:
        """Call /{prefix}/train to start training."""
        request = TrainModel.Request()
        request.policy_type = policy_type
        request.dataset_path = dataset_path
        request.output_dir = output_dir
        request.steps = num_epochs
        request.batch_size = batch_size
        request.learning_rate = learning_rate
        request.eval_freq = eval_freq
        request.log_freq = log_freq
        request.save_freq = save_freq
        request.wandb_project = wandb_project
        request.push_to_hub = False

        return self._call_service(
            self._train_client, request, self.service_train
        )

    def resume_training(self, checkpoint_path: str) -> ServiceResponse:
        """Call /{prefix}/train to resume training from checkpoint."""
        request = TrainModel.Request()
        request.policy_type = ''
        request.dataset_path = ''
        request.output_dir = checkpoint_path
        request.steps = 0
        request.batch_size = 0
        request.learning_rate = 0.0
        request.eval_freq = 0
        request.log_freq = 0
        request.save_freq = 0
        request.wandb_project = ''
        request.push_to_hub = False

        return self._call_service(
            self._train_client, request, self.service_train
        )

    def stop_training(self) -> ServiceResponse:
        """Call /{prefix}/stop to stop training."""
        request = StopTraining.Request()
        return self._call_service(
            self._stop_client, request, self.service_stop
        )

    def get_training_status(self) -> ServiceResponse:
        """Call /{prefix}/status to get training status."""
        request = TrainingStatus.Request()
        return self._call_service(
            self._status_client, request, self.service_status
        )

    # --- Subscriptions ---

    def subscribe_progress(self, callback: Callable[[Dict], None]) -> bool:
        """Subscribe to /{prefix}/progress topic for training updates."""
        try:
            qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE
            )

            def on_progress(msg: TrainingProgress):
                data = {
                    'status': msg.state,
                    'step': msg.step,
                    'total_steps': msg.total_steps,
                    'epoch': msg.epoch,
                    'loss': msg.loss,
                    'learning_rate': msg.learning_rate,
                    'gradient_norm': msg.gradient_norm,
                    'samples_per_second': msg.samples_per_second,
                    'elapsed_seconds': msg.elapsed_seconds,
                    'eta_seconds': msg.eta_seconds,
                }
                callback(data)

            self._progress_sub = self._node.create_subscription(
                TrainingProgress,
                self.topic_progress,
                on_progress,
                qos
            )
            return True
        except Exception as e:
            logger.error(f'Failed to subscribe to progress: {e}')
            return False
