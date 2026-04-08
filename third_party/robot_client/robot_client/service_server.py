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
RobotServiceServer - Decorator-based service framework for training/inference executors.

Usage:
    server = RobotServiceServer(name="groot", domain_id=30)

    @server.on_train
    def handle_train(request):
        server.report_progress(total_steps=1000, step=0)
        ...

    @server.on_load_policy
    def handle_infer(request):
        return {"success": True, "action_keys": ["arm_left"]}

    @server.on_get_action
    def handle_get_action(request):
        T, D = action.shape
        return {"action_chunk": action.flatten().tolist(), "chunk_size": T, "action_dim": D}

    @server.on_stop
    def handle_stop():
        cleanup()

    if __name__ == "__main__":
        server.spin()
"""
import json
import logging
import math
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional, Callable

import numpy as np

# Add zenoh_ros2_sdk to path if not already available
_SDK_PATH = os.environ.get("ZENOH_SDK_PATH", "")
if _SDK_PATH and _SDK_PATH not in sys.path:
    sys.path.insert(0, _SDK_PATH)

from zenoh_ros2_sdk import ROS2ServiceServer, ROS2Publisher

from .messages import (
    TRAIN_MODEL_REQUEST_DEF, TRAIN_MODEL_RESPONSE_DEF,
    START_INFERENCE_REQUEST_DEF, START_INFERENCE_RESPONSE_DEF,
    GET_ACTION_CHUNK_REQUEST_DEF, GET_ACTION_CHUNK_RESPONSE_DEF,
    STOP_TRAINING_REQUEST_DEF, STOP_TRAINING_RESPONSE_DEF,
    TRAINING_STATUS_REQUEST_DEF, TRAINING_STATUS_RESPONSE_DEF,
    TRAINING_PROGRESS_DEF,
)

logger = logging.getLogger("robot_service_server")


@dataclass
class TrainingProgress:
    """Training progress data for monitoring."""
    step: int = 0
    total_steps: int = 0
    loss: float = 0.0
    learning_rate: float = 0.0
    gradient_norm: float = 0.0
    epoch: float = 0.0
    samples_per_second: float = 0.0
    elapsed_seconds: float = 0.0
    eta_seconds: float = 0.0
    state: str = ""
    message: str = ""


class _RequestProxy:
    """Proxy object for service request with dict-like attribute access.

    Lookup order:
      1. Raw message fields (actual service definition fields)
      2. extra_params dict (parsed from extra_params_json)
      3. Raise AttributeError (so getattr(proxy, 'x', default) works correctly)
    """
    def __init__(self, raw_msg, extra_params: Optional[dict] = None):
        self._raw = raw_msg
        self.extra_params = extra_params or {}

    def __getattr__(self, name):
        if name.startswith("_") or name == "extra_params":
            return super().__getattribute__(name)
        # 1. Check raw message (only if attribute actually exists)
        raw = self._raw
        if hasattr(raw, name):
            val = getattr(raw, name)
            # Return raw value if non-empty; for strings, skip empty ones
            # to allow extra_params fallback
            if not (isinstance(val, str) and val == ""):
                return val
        # 2. Check extra_params dict
        extra = super().__getattribute__("extra_params")
        if name in extra:
            return extra[name]
        # 3. Check raw message (return even if empty, if field exists)
        if hasattr(raw, name):
            return getattr(raw, name)
        # 4. Raise AttributeError for proper getattr(obj, name, default) behavior
        raise AttributeError(f"Request has no attribute '{name}'")


class RobotServiceServer:
    """Decorator-based service framework for training/inference executors.

    Services (5):
        /{name}/train             - Start training (async, background thread)
        /{name}/infer             - Start inference (load policy)
        /{name}/get_action_chunk  - Get action chunk (on-demand inference)
        /{name}/stop              - Stop training/inference
        /{name}/status            - Get state and progress (built-in)
    """

    def __init__(
        self,
        name: str,
        router_ip: str = "127.0.0.1",
        router_port: int = 7447,
        domain_id: Optional[int] = None,
        node_name: Optional[str] = None,
        namespace: str = "/",
    ):
        self._name = name
        self._router_ip = router_ip
        self._router_port = router_port
        self._domain_id = domain_id
        self._node_name = node_name or f"{name}_executor"
        self._namespace = namespace

        # State management (thread-safe)
        self._state = "idle"
        self._state_lock = threading.Lock()
        self._progress = TrainingProgress()
        self._stop_event = threading.Event()
        self._current_job_id: Optional[str] = None

        # Handlers registered via decorators
        self._handlers: dict[str, Callable] = {}

        # Runtime (populated in _setup_services)
        self._services: list = []
        self._progress_publisher: Optional[ROS2Publisher] = None
        self._progress_thread: Optional[threading.Thread] = None
        self._training_thread: Optional[threading.Thread] = None
        self._running = False

        # NaN-aware progress change detection
        self._last_published_step: int = -1
        self._last_published_loss: float = float("nan")

    # ------------------------------------------------------------------ #
    # Properties
    # ------------------------------------------------------------------ #

    @property
    def name(self) -> str:
        return self._name

    @property
    def state(self) -> str:
        with self._state_lock:
            return self._state

    @state.setter
    def state(self, value: str) -> None:
        with self._state_lock:
            self._state = value

    @property
    def stop_requested(self) -> threading.Event:
        return self._stop_event

    @property
    def progress(self) -> TrainingProgress:
        return self._progress

    # ------------------------------------------------------------------ #
    # Decorator API
    # ------------------------------------------------------------------ #

    def on_train(self, func: Callable) -> Callable:
        """Register handler for /{name}/train service."""
        self._handlers["train"] = func
        return func

    def on_load_policy(self, func: Callable) -> Callable:
        """Register handler for /{name}/infer service."""
        self._handlers["load_policy"] = func
        return func

    def on_get_action(self, func: Callable) -> Callable:
        """Register handler for /{name}/get_action_chunk service."""
        self._handlers["get_action"] = func
        return func

    def on_stop(self, func: Callable) -> Callable:
        """Register handler for /{name}/stop service."""
        self._handlers["stop"] = func
        return func

    # ------------------------------------------------------------------ #
    # Progress reporting
    # ------------------------------------------------------------------ #

    def report_progress(self, **kwargs):
        """Update training progress fields.

        Example: server.report_progress(step=100, loss=0.5, learning_rate=1e-4)
        """
        for k, v in kwargs.items():
            if hasattr(self._progress, k):
                setattr(self._progress, k, v)

    # ------------------------------------------------------------------ #
    # Service handlers (internal)
    # ------------------------------------------------------------------ #

    def _handle_train(self, request_msg):
        """Handle TrainModel - launch training in background thread."""
        handler = self._handlers.get("train")
        if not handler:
            return self._make_response(
                TRAIN_MODEL_RESPONSE_DEF,
                "physical_ai_interfaces/srv/TrainModel_Response",
                success=False, message="No train handler registered", job_id="",
            )

        if self.state in ("training", "inference"):
            return self._make_response(
                TRAIN_MODEL_RESPONSE_DEF,
                "physical_ai_interfaces/srv/TrainModel_Response",
                success=False,
                message=f"Already {self.state}. Stop current task first.",
                job_id="",
            )

        proxy = self._parse_request(request_msg)
        job_id = f"{self._name}_train_{int(time.time())}"
        self._current_job_id = job_id
        self._stop_event.clear()
        self._progress = TrainingProgress(state="training")
        self.state = "training"

        def _train_worker():
            try:
                handler(proxy)
                if self._stop_event.is_set():
                    logger.info("Training stopped by user")
                else:
                    logger.info("Training completed")
            except Exception as e:
                logger.error(f"Training failed: {e}", exc_info=True)
                self.state = "error"
                self._progress.state = "error"
                self._progress.message = str(e)
            finally:
                if self.state == "training":
                    self.state = "idle"
                self._progress.state = self.state
                self._publish_progress()
                self._current_job_id = None

        self._training_thread = threading.Thread(target=_train_worker, daemon=True)
        self._training_thread.start()

        return self._make_response(
            TRAIN_MODEL_RESPONSE_DEF,
            "physical_ai_interfaces/srv/TrainModel_Response",
            success=True, message="Training started", job_id=job_id,
        )

    def _handle_load_policy(self, request_msg):
        """Handle StartInference service request."""
        handler = self._handlers.get("load_policy")
        if not handler:
            return self._make_response(
                START_INFERENCE_RESPONSE_DEF,
                "physical_ai_interfaces/srv/StartInference_Response",
                success=False, message="No load_policy handler registered",
                action_keys=[],
            )

        proxy = self._parse_request(request_msg)
        self.state = "inference"

        try:
            result = handler(proxy)
            if isinstance(result, dict):
                success = result.get("success", True)
                message = result.get("message", "")
                action_keys = result.get("action_keys", [])
                if not success:
                    self.state = "error"
            else:
                success, message, action_keys = True, "", []
            return self._make_response(
                START_INFERENCE_RESPONSE_DEF,
                "physical_ai_interfaces/srv/StartInference_Response",
                success=success, message=message, action_keys=action_keys,
            )
        except Exception as e:
            self.state = "error"
            logger.error(f"Load policy error: {e}", exc_info=True)
            return self._make_response(
                START_INFERENCE_RESPONSE_DEF,
                "physical_ai_interfaces/srv/StartInference_Response",
                success=False, message=str(e), action_keys=[],
            )

    def _handle_get_action(self, request_msg):
        """Handle GetActionChunk service request."""
        handler = self._handlers.get("get_action")
        if not handler:
            return self._make_response(
                GET_ACTION_CHUNK_RESPONSE_DEF,
                "physical_ai_interfaces/srv/GetActionChunk_Response",
                success=False, message="No get_action handler registered",
                action_chunk=[], chunk_size=0, action_dim=0,
            )

        proxy = _RequestProxy(request_msg)

        try:
            result = handler(proxy)
            if isinstance(result, dict):
                chunk = result.get("action_chunk")
                chunk_size = result.get("chunk_size", 0)
                action_dim = result.get("action_dim", 0)
                success = result.get("success", True)
                message = result.get("message", "")

                if isinstance(chunk, np.ndarray):
                    if len(chunk.shape) > 1:
                        chunk_size = chunk_size or chunk.shape[0]
                        action_dim = action_dim or chunk.shape[-1]
                    chunk_flat = chunk.flatten().tolist()
                elif chunk is not None:
                    chunk_flat = list(chunk)
                else:
                    chunk_flat = []

                return self._make_response(
                    GET_ACTION_CHUNK_RESPONSE_DEF,
                    "physical_ai_interfaces/srv/GetActionChunk_Response",
                    success=success, message=message,
                    action_chunk=chunk_flat, chunk_size=chunk_size, action_dim=action_dim,
                )
            return self._make_response(
                GET_ACTION_CHUNK_RESPONSE_DEF,
                "physical_ai_interfaces/srv/GetActionChunk_Response",
                success=False, message="Invalid handler return",
                action_chunk=[], chunk_size=0, action_dim=0,
            )
        except Exception as e:
            logger.error(f"Get action error: {e}", exc_info=True)
            return self._make_response(
                GET_ACTION_CHUNK_RESPONSE_DEF,
                "physical_ai_interfaces/srv/GetActionChunk_Response",
                success=False, message=str(e),
                action_chunk=[], chunk_size=0, action_dim=0,
            )

    def _handle_stop(self, request_msg):
        """Handle StopTraining service request."""
        handler = self._handlers.get("stop")

        if self.state == "idle":
            return self._make_response(
                STOP_TRAINING_RESPONSE_DEF,
                "physical_ai_interfaces/srv/StopTraining_Response",
                success=False, message="No task is running",
            )

        prev_state = self.state
        self._stop_event.set()
        self.state = "idle"

        if handler:
            try:
                handler()
            except Exception as e:
                logger.error(f"Stop handler error: {e}", exc_info=True)

        # Wait for training thread to finish
        if self._training_thread and self._training_thread.is_alive():
            self._training_thread.join(timeout=10.0)

        return self._make_response(
            STOP_TRAINING_RESPONSE_DEF,
            "physical_ai_interfaces/srv/StopTraining_Response",
            success=True, message=f"Stopped (was {prev_state})",
        )

    def _handle_status(self, request_msg):
        """Handle TrainingStatus - built-in, returns current state and progress."""
        return self._make_response(
            TRAINING_STATUS_RESPONSE_DEF,
            "physical_ai_interfaces/srv/TrainingStatus_Response",
            state=self.state,
            step=self._progress.step,
            total_steps=self._progress.total_steps,
            loss=float(self._progress.loss),
            learning_rate=float(self._progress.learning_rate),
            gradient_norm=float(self._progress.gradient_norm),
            elapsed_seconds=float(self._progress.elapsed_seconds),
            eta_seconds=float(self._progress.eta_seconds),
            job_id=self._current_job_id or "",
            message=self._progress.message,
        )

    # ------------------------------------------------------------------ #
    # Request parsing
    # ------------------------------------------------------------------ #

    def _parse_request(self, msg) -> _RequestProxy:
        extra = {}
        extra_json = getattr(msg, "extra_params_json", "")
        if extra_json:
            try:
                extra = json.loads(extra_json)
            except json.JSONDecodeError:
                logger.warning(f"Invalid extra_params_json: {extra_json}")
        return _RequestProxy(msg, extra_params=extra)

    # ------------------------------------------------------------------ #
    # Response builder
    # ------------------------------------------------------------------ #

    def _make_response(self, definition, type_name, **kwargs):
        """Create a response object from definition and kwargs."""
        from zenoh_ros2_sdk.session import ZenohSession
        session = ZenohSession.get_instance(self._router_ip, self._router_port)
        ResponseClass = session.register_message_type(definition, type_name)
        # Filter kwargs to only fields that exist in the response class
        if hasattr(ResponseClass, "__dataclass_fields__"):
            fields = ResponseClass.__dataclass_fields__.keys()
            filtered = {k: v for k, v in kwargs.items() if k in fields}
        else:
            filtered = kwargs
        # Convert plain lists to numpy arrays for CDR serialization (float64[] etc.)
        for k, v in filtered.items():
            if isinstance(v, list) and (not v or isinstance(v[0], (int, float))):
                filtered[k] = np.array(v, dtype=np.float64)
        return ResponseClass(**filtered)

    # ------------------------------------------------------------------ #
    # Progress publishing
    # ------------------------------------------------------------------ #

    def _progress_loop(self):
        """Background loop: publish progress when step or loss changes."""
        while self._running:
            try:
                if self.state == "training" and self._progress_publisher:
                    current_step = self._progress.step
                    current_loss = self._progress.loss

                    step_changed = current_step != self._last_published_step

                    # NaN-aware comparison
                    curr_nan = isinstance(current_loss, float) and math.isnan(current_loss)
                    last_nan = isinstance(self._last_published_loss, float) and math.isnan(self._last_published_loss)
                    if curr_nan and last_nan:
                        loss_changed = False
                    elif curr_nan or last_nan:
                        loss_changed = True
                    else:
                        loss_changed = current_loss != self._last_published_loss

                    if step_changed or loss_changed:
                        self._publish_progress()
                        self._last_published_step = current_step
                        self._last_published_loss = current_loss

                time.sleep(1.0)
            except Exception as e:
                logger.debug(f"Progress loop error: {e}")

    def _publish_progress(self):
        if self._progress_publisher is None:
            return
        try:
            self._progress_publisher.publish(
                step=self._progress.step,
                total_steps=self._progress.total_steps,
                epoch=self._progress.epoch,
                loss=self._progress.loss,
                learning_rate=self._progress.learning_rate,
                gradient_norm=self._progress.gradient_norm,
                samples_per_second=self._progress.samples_per_second,
                elapsed_seconds=self._progress.elapsed_seconds,
                eta_seconds=self._progress.eta_seconds,
                state=self.state,
            )
        except Exception as e:
            logger.debug(f"Failed to publish progress: {e}")

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def _setup_services(self):
        """Register all services and start progress publisher.

        Closes any stale ZenohSession first to ensure clean discovery state.
        This prevents stale queryables from previous executor runs from
        interfering with service calls.
        """
        prefix = f"/{self._name}"
        common_kwargs = {
            "node_name": self._node_name,
            "namespace": self._namespace,
            "router_ip": self._router_ip,
            "router_port": self._router_port,
        }
        if self._domain_id is not None:
            common_kwargs["domain_id"] = self._domain_id

        # Core services (always registered)
        service_defs = [
            (f"{prefix}/train", "physical_ai_interfaces/srv/TrainModel",
             TRAIN_MODEL_REQUEST_DEF, TRAIN_MODEL_RESPONSE_DEF,
             self._handle_train),
            (f"{prefix}/infer", "physical_ai_interfaces/srv/StartInference",
             START_INFERENCE_REQUEST_DEF, START_INFERENCE_RESPONSE_DEF,
             self._handle_load_policy),
            (f"{prefix}/stop", "physical_ai_interfaces/srv/StopTraining",
             STOP_TRAINING_REQUEST_DEF, STOP_TRAINING_RESPONSE_DEF,
             self._handle_stop),
            (f"{prefix}/status", "physical_ai_interfaces/srv/TrainingStatus",
             TRAINING_STATUS_REQUEST_DEF, TRAINING_STATUS_RESPONSE_DEF,
             self._handle_status),
        ]

        # Conditionally register get_action_chunk if handler registered
        if "get_action" in self._handlers:
            service_defs.append(
                (f"{prefix}/get_action_chunk", "physical_ai_interfaces/srv/GetActionChunk",
                 GET_ACTION_CHUNK_REQUEST_DEF, GET_ACTION_CHUNK_RESPONSE_DEF,
                 self._handle_get_action)
            )

        for svc_name, srv_type, req_def, resp_def, handler in service_defs:
            try:
                svc = ROS2ServiceServer(
                    service_name=svc_name,
                    srv_type=srv_type,
                    callback=handler,
                    request_definition=req_def,
                    response_definition=resp_def,
                    mode="callback",
                    **common_kwargs,
                )
                self._services.append(svc)
                logger.info(f"Service registered: {svc_name}")
            except Exception as e:
                logger.error(f"Failed to register service {svc_name}: {e}", exc_info=True)

        # Progress publisher
        try:
            self._progress_publisher = ROS2Publisher(
                topic=f"{prefix}/training_progress",
                msg_type="physical_ai_interfaces/msg/TrainingProgress",
                msg_definition=TRAINING_PROGRESS_DEF,
                **common_kwargs,
            )
        except Exception as e:
            logger.warning(f"Failed to create progress publisher: {e}")

        # Start progress background thread
        self._running = True
        self._progress_thread = threading.Thread(target=self._progress_loop, daemon=True)
        self._progress_thread.start()

        self.state = "idle"
        logger.info(f"RobotServiceServer '{self._name}' ready with {len(self._services)} services")

    def _cleanup(self):
        """Stop all services and cleanup."""
        self._running = False

        if self._progress_thread:
            self._progress_thread.join(timeout=2.0)

        # Stop training if running
        self._stop_event.set()
        if self._training_thread and self._training_thread.is_alive():
            self._training_thread.join(timeout=5.0)

        for svc in self._services:
            try:
                svc.close()
            except Exception:
                pass
        self._services.clear()

        if self._progress_publisher:
            try:
                self._progress_publisher.close()
            except Exception:
                pass
            self._progress_publisher = None

        # Close ZenohSession to undeclare all liveliness tokens
        try:
            from zenoh_ros2_sdk.session import ZenohSession
            session = ZenohSession.get_instance(self._router_ip, self._router_port)
            session.close()
        except Exception as e:
            logger.debug(f"Error closing ZenohSession: {e}")

        logger.info(f"RobotServiceServer '{self._name}' stopped")

    def spin(self):
        """Setup services and block until SIGINT/SIGTERM.

        This is the main entry point. Equivalent to ROS2's rclpy.spin().
        Calls _setup_services() internally, then waits for shutdown signal.
        """
        self._setup_services()

        def _signal_handler(signum, frame):
            logger.info(f"Received signal {signum}, shutting down...")
            self._cleanup()
            sys.exit(0)

        signal.signal(signal.SIGINT, _signal_handler)
        signal.signal(signal.SIGTERM, _signal_handler)

        logger.info(f"RobotServiceServer '{self._name}' spinning. Press Ctrl+C to stop.")

        try:
            while self._running:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
        finally:
            self._cleanup()
