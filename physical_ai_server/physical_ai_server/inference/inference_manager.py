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
InferenceManager - Action chunk based async inference manager.

Generic manager that works with any inference container (GR00T, LeRobot, etc.)
by parameterizing the service prefix.

Manages action buffer, background chunk requests via ContainerServiceClient,
L2-based chunk alignment, and action-to-JointTrajectory conversion.

Architecture:
    100Hz timer (physical_ai_server)
        -> pop_action() from buffer
        -> convert to JointTrajectory messages
        -> communicator.publish_action()

    Background thread:
        -> /{prefix}/get_action_chunk service call (15Hz waypoints)
        -> linear interpolation to 100Hz
        -> align & enqueue into buffer
"""

import collections
import logging
import threading
from typing import Optional

import numpy as np
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist

from physical_ai_server.communication.container_service_client import ContainerServiceClient

logger = logging.getLogger(__name__)


class InferenceManager:
    """Action chunk based async inference manager.

    Works with any inference container (GR00T, LeRobot, etc.)
    via the service_prefix parameter.
    """

    DEFAULT_INFERENCE_HZ = 15.0   # Default model output rate (overridable per call)
    BLEND_DURATION_S = 0.2  # Blend duration in seconds at chunk boundaries
    REFILL_MARGIN_S = 0.1  # Fixed time margin added to adaptive refill threshold
    EMA_ALPHA = 0.3        # EMA smoothing factor for chunk fetch time

    def __init__(
        self,
        node: Node,
        joint_topic_types: dict,
        joint_order: dict,
        service_prefix: str = "/groot",
        on_chunk_received: callable = None,
        control_hz: float = 100.0,
        inference_hz: float = 15.0,
        chunk_align_window_s: float = 0.3,
        client_cb_group=None,
    ):
        """
        Args:
            node: ROS2 node for creating service clients.
            joint_topic_types: {group_name: msg_type} from communicator.
            joint_order: {group_name: [joint_names]} from robot config.
                Keys are like "joint_order.leader_arm_left".
            service_prefix: Service prefix for the inference container
                (e.g., "/groot", "/lerobot").
            on_chunk_received: Optional callback(chunk_msg: JointTrajectory)
                called when a new action chunk arrives, for visualization.
            control_hz: Robot command rate from UI (used for interpolation).
            client_cb_group: Callback group for service clients. Must differ
                from the service server callback group to avoid deadlock
                with MultiThreadedExecutor.
        """
        self._node = node
        self._joint_topic_types = joint_topic_types
        self._joint_order = joint_order
        self._service_prefix = service_prefix
        self._on_chunk_received = on_chunk_received
        self._action_joint_map: dict = {}
        self._client_cb_group = client_cb_group

        # Interpolation and buffer parameters derived from control_hz.
        # inference_hz = rate of model output waypoints (must match training
        # data rate); control_hz = rate of interpolated commands to the robot.
        self._control_hz = float(control_hz)
        self._inference_hz = float(inference_hz) if inference_hz else self.DEFAULT_INFERENCE_HZ
        # L2 alignment is restricted to the first few raw waypoints of a new
        # chunk (chunk_align_window_s × inference_hz). This prevents jumping
        # to a "closer" later waypoint when the trajectory revisits points
        # (e.g. A→B→C→B→A would otherwise get stuck at the second B).
        self._chunk_align_window_s = max(0.0, float(chunk_align_window_s))
        self._blend_steps = max(1, int(self.BLEND_DURATION_S * self._control_hz))

        # Adaptive refill: threshold updated based on measured chunk fetch time
        self._chunk_fetch_ema: Optional[float] = None  # EMA of chunk fetch duration
        self._buffer_refill_threshold = max(1, int(self.REFILL_MARGIN_S * self._control_hz))

        # Action buffer (deque of 1D np arrays, each length = total action DOF)
        self._action_buffer: collections.deque = collections.deque()
        self._buffer_lock = threading.Lock()

        self._client: Optional[ContainerServiceClient] = None

        # Background threads
        self._inference_thread: Optional[threading.Thread] = None
        self._load_thread: Optional[threading.Thread] = None
        self._running = False
        self._requesting = False  # Prevent duplicate requests

        # Async load state
        self._loading = False
        self._load_error: Optional[str] = None
        self._paused = False

        # Chunk alignment
        self._last_action: Optional[np.ndarray] = None

        # Task instruction for language-conditioned policies
        self._task_instruction: str = ""

    @property
    def is_loading(self) -> bool:
        return self._loading

    @property
    def is_ready(self) -> bool:
        return self._running and not self._loading

    @property
    def load_error(self) -> Optional[str]:
        return self._load_error

    @property
    def is_paused(self) -> bool:
        """Return True if inference is paused (model loaded but not running)."""
        return self._paused and self._running

    def start(
        self,
        model_path: str,
        embodiment_tag: str,
        robot_type: str,
        task_instruction: str,
    ):
        """Start inference asynchronously. Returns immediately.

        Connects to the service client and launches a background thread
        to load the policy. Use is_loading / is_ready / load_error to
        monitor progress.
        """
        self._task_instruction = task_instruction
        self._loading = True
        self._load_error = None

        self._client = ContainerServiceClient(
            node=self._node,
            service_prefix=self._service_prefix,
            callback_group=self._client_cb_group,
        )
        self._client.connect()

        self._load_thread = threading.Thread(
            target=self._load_policy_thread,
            args=(model_path, embodiment_tag, robot_type, task_instruction),
            daemon=True,
        )
        self._load_thread.start()
        logger.info(f"Policy load started in background ({self._service_prefix})")

    def _load_policy_thread(
        self,
        model_path: str,
        embodiment_tag: str,
        robot_type: str,
        task_instruction: str,
    ):
        """Background thread: call start_inference service and transition to ready."""
        try:
            response = self._client.start_inference(
                model_path=model_path,
                embodiment_tag=embodiment_tag,
                robot_type=robot_type,
                task_instruction=task_instruction,
            )
            if not response.success:
                self._load_error = (
                    f"Inference setup failed ({self._service_prefix}): "
                    f"{response.message}"
                )
                logger.error(self._load_error)
                return

            action_keys = response.data.get("action_keys", [])
            self._build_action_joint_map(action_keys)

            self._running = True
            logger.info(
                f"InferenceManager ready ({self._service_prefix}), "
                f"action_keys={action_keys}, requesting first chunk..."
            )
            self._request_chunk_async()

        except Exception as e:
            self._load_error = str(e)
            logger.error(f"Policy load failed: {e}")
        finally:
            self._loading = False

    def pop_action(self) -> Optional[dict]:
        """Pop one action from buffer and convert to joint messages.

        Called by 100Hz timer in physical_ai_server.
        Returns dict of {group_name: JointTrajectory msg} or None if buffer empty.
        When buffer is empty, holds the last commanded action to maintain
        consistent 100Hz publish rate.
        """
        if self._paused:
            return None
        with self._buffer_lock:
            if not self._action_buffer:
                # Buffer empty — request new chunk if not already requesting
                if not self._requesting:
                    self._request_chunk_async()
                # Hold last action to maintain 100Hz rate
                if self._last_action is not None:
                    return self._action_to_joint_msgs(self._last_action)
                return None
            action = self._action_buffer.popleft()
            remaining = len(self._action_buffer)

        # Request new chunk if buffer running low
        if remaining < self._buffer_refill_threshold and not self._requesting:
            self._request_chunk_async()

        return self._action_to_joint_msgs(action)

    def _request_chunk_async(self):
        """Start background thread to fetch next action chunk."""
        if not self._running or self._paused:
            return
        self._requesting = True
        self._inference_thread = threading.Thread(
            target=self._fetch_chunk, daemon=True
        )
        self._inference_thread.start()

    def _update_refill_threshold(self, fetch_duration: float):
        """Update adaptive refill threshold based on measured chunk fetch time."""
        if self._chunk_fetch_ema is None:
            self._chunk_fetch_ema = fetch_duration
        else:
            self._chunk_fetch_ema = (
                self.EMA_ALPHA * fetch_duration
                + (1 - self.EMA_ALPHA) * self._chunk_fetch_ema
            )
        refill_s = self._chunk_fetch_ema + self.REFILL_MARGIN_S
        self._buffer_refill_threshold = max(1, int(refill_s * self._control_hz))

    def _fetch_chunk(self):
        """Blocking service call to get action chunk, then enqueue.

        Retries up to MAX_RETRIES times with a delay if observations
        are not yet available (e.g., during startup).
        """
        import time
        MAX_RETRIES = 10
        RETRY_DELAY = 1.0  # seconds

        try:
            if self._client is None:
                return

            for attempt in range(MAX_RETRIES):
                if not self._running or self._paused:
                    return

                t0 = time.perf_counter()
                response = self._client.get_action_chunk(
                    task_instruction=self._task_instruction
                )
                fetch_duration = time.perf_counter() - t0

                if response.success:
                    # Double-check we weren't paused during the service call
                    if self._paused:
                        logger.debug("Chunk arrived but paused, discarding")
                        return

                    chunk_data = response.data.get("action_chunk", [])
                    chunk_size = response.data.get("chunk_size", 0)
                    action_dim = response.data.get("action_dim", 0)

                    if chunk_size > 0 and action_dim > 0 and len(chunk_data) > 0:
                        self._update_refill_threshold(fetch_duration)
                        chunk = np.array(chunk_data).reshape(chunk_size, action_dim)
                        # Discard chunk if paused — it's based on pre-pause
                        # observations and would cause sudden movement on resume.
                        if self._paused:
                            return
                        self._align_and_enqueue(chunk)
                        self._publish_chunk_preview(chunk)
                        logger.info(
                            f"Chunk received: T={chunk_size}, D={action_dim}, "
                            f"buffer={len(self._action_buffer)}, "
                            f"fetch={fetch_duration:.3f}s, "
                            f"refill_th={self._buffer_refill_threshold}"
                        )
                        return  # Success
                    else:
                        logger.warning(f"Empty chunk response: {response.message}")
                        return
                else:
                    # Retry on transient failures (missing observations, etc.)
                    if attempt < MAX_RETRIES - 1:
                        logger.warning(
                            f"Chunk request failed (attempt {attempt + 1}/{MAX_RETRIES}): "
                            f"{response.message}, retrying in {RETRY_DELAY}s..."
                        )
                        time.sleep(RETRY_DELAY)
                    else:
                        logger.error(
                            f"Chunk request failed after {MAX_RETRIES} attempts: "
                            f"{response.message}"
                        )
        except Exception as e:
            logger.error(f"Error fetching action chunk: {e}")
        finally:
            self._requesting = False

    def _interpolate_chunk(self, chunk: np.ndarray) -> np.ndarray:
        """Linear interpolation from inference_hz to control_hz.

        Args:
            chunk: shape (T, D), T waypoints at self._inference_hz
        Returns:
            shape (T_interp, D), interpolated waypoints at self._control_hz
        """
        T, D = chunk.shape
        if T < 2:
            return chunk

        t_original = np.arange(T) / self._inference_hz
        duration = (T - 1) / self._inference_hz
        n_interp = int(round(duration * self._control_hz)) + 1
        t_interp = np.linspace(0, duration, n_interp)

        # Per-joint linear interpolation using np.interp
        interpolated = np.empty((n_interp, D))
        for d in range(D):
            interpolated[:, d] = np.interp(t_interp, t_original, chunk[:, d])

        return interpolated

    def _align_and_enqueue(self, chunk: np.ndarray):
        """L2 alignment, linear interpolation, and blending.

        1. L2 alignment: find closest raw waypoint to last_action, skip past ones
        2. Interpolate remaining raw waypoints to CONTROL_HZ
        3. Linear blend at chunk boundary from last_action
        """
        with self._buffer_lock:
            # L2 alignment on raw 15Hz waypoints, restricted to the first
            # chunk_align_window_s worth of waypoints so loop trajectories
            # (e.g. A→B→C→B→A) don't jump ahead to a later "closer" point.
            if self._last_action is not None and len(chunk) > 1:
                search_n = int(round(self._chunk_align_window_s * self._inference_hz))
                search_n = max(1, min(search_n, len(chunk)))
                distances = np.linalg.norm(chunk[:search_n] - self._last_action, axis=1)
                best_idx = int(np.argmin(distances))
                logger.info(
                    f"L2 align: search={search_n}/{len(chunk)}, "
                    f"best_idx={best_idx}, min_dist={distances[best_idx]:.4f}, "
                    f"window={self._chunk_align_window_s:.2f}s"
                )
                start_idx = best_idx + 1
                if start_idx < len(chunk):
                    chunk = chunk[start_idx:]

            if len(chunk) == 0:
                return

            # Interpolate 15Hz -> 100Hz
            chunk = self._interpolate_chunk(chunk)

            # Linear blend at chunk boundary
            if self._last_action is not None and len(chunk) > 0:
                n_blend = min(self._blend_steps, len(chunk))
                for i in range(n_blend):
                    alpha = (i + 1) / (n_blend + 1)
                    chunk[i] = (1 - alpha) * self._last_action + alpha * chunk[i]

            for action in chunk:
                self._action_buffer.append(action)

            if len(chunk) > 0:
                self._last_action = chunk[-1].copy()

    def _publish_chunk_preview(self, chunk: np.ndarray):
        """Publish full action chunk as a single JointTrajectory for 3D preview."""
        if self._on_chunk_received is None:
            return

        try:
            all_joint_names = []
            for modality_key, leader_group in self._action_joint_map.items():
                joint_names = self._joint_order.get(leader_group, [])
                all_joint_names.extend(joint_names)

            if not all_joint_names:
                return

            msg = JointTrajectory()
            msg.joint_names = list(all_joint_names)

            for t in range(len(chunk)):
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in chunk[t]]
                msg.points.append(point)

            self._on_chunk_received(msg)
        except Exception as e:
            logger.debug(f"Failed to publish chunk preview: {e}")

    def _build_action_joint_map(self, action_keys: list):
        """Build action_joint_map from model's action_keys and joint_order.

        Maps modality keys (e.g. "arm_left") to leader group keys in
        joint_order (e.g. "joint_order.leader_arm_left").
        """
        self._action_joint_map = {}
        for key in action_keys:
            leader_key = f"joint_order.leader_{key}"
            if leader_key in self._joint_order:
                self._action_joint_map[key] = leader_key
            else:
                logger.warning(
                    f"No leader group found for action key '{key}' "
                    f"(tried '{leader_key}')"
                )
        logger.info(f"Action joint map: {self._action_joint_map}")

    def _action_to_joint_msgs(self, action: np.ndarray) -> dict:
        """Convert flat action array to per-group JointTrajectory messages.

        Action is ordered by action_joint_map keys (same order as model output).
        e.g. [arm_l_j1..j7, grip_l, arm_r_j1..j7, grip_r] (16 dims)
        """
        joint_msg_datas = {}
        offset = 0

        for modality_key, leader_group in self._action_joint_map.items():
            joint_names = self._joint_order.get(leader_group, [])
            n_joints = len(joint_names)

            if n_joints == 0:
                continue

            values = action[offset:offset + n_joints]
            offset += n_joints

            # Strip "joint_order." prefix to match communicator publisher keys
            publisher_key = leader_group.removeprefix("joint_order.")

            if 'mobile' in publisher_key.lower():
                # Mobile action → Twist on /cmd_vel (publisher is created as
                # Twist in communicator.init_publishers).
                msg = Twist()
                msg.linear.x = float(values[0]) if len(values) > 0 else 0.0
                msg.linear.y = float(values[1]) if len(values) > 1 else 0.0
                msg.angular.z = float(values[2]) if len(values) > 2 else 0.0
            else:
                msg = JointTrajectory()
                msg.joint_names = list(joint_names)
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in values]
                msg.points = [point]

            joint_msg_datas[publisher_key] = msg

        return joint_msg_datas

    def stop(self):
        """Stop inference and cleanup all state."""
        self._running = False
        self._loading = False
        self._paused = False
        self._requesting = False

        # Send stop to the container FIRST, before cancelling in-flight calls.
        # This ensures the stop request is sent while the client/middleware is
        # in a clean state, not blocked by cancelled futures.
        if self._client:
            try:
                self._client.stop_inference()
            except Exception as e:
                logger.error(f"Error stopping inference: {e}")

        # Signal in-progress service calls to abort via Event
        if self._client:
            self._client._cancelled.set()

        # Wait for background threads to exit
        if self._load_thread and self._load_thread.is_alive():
            self._load_thread.join(timeout=5.0)
        if self._inference_thread and self._inference_thread.is_alive():
            self._inference_thread.join(timeout=5.0)

        if self._client:
            self._client._cancelled.clear()
            try:
                self._client.disconnect()
            except Exception as e:
                logger.debug(f"Error disconnecting client: {e}")
            self._client = None

        with self._buffer_lock:
            self._action_buffer.clear()

        self._last_action = None
        self._action_joint_map = {}
        self._chunk_fetch_ema = None
        self._load_error = None
        logger.info(f"InferenceManager stopped ({self._service_prefix})")

    def pause(self):
        """Pause inference loop.

        Model stays loaded, stops requesting new chunks.
        Clears action buffer to prevent stale actions causing
        sudden movement on resume.
        """
        self._paused = True
        with self._buffer_lock:
            self._action_buffer.clear()
        self._last_action = None
        logger.info(
            "InferenceManager paused (%s)", self._service_prefix
        )

    def resume(self, task_instruction: str = ""):
        """Resume inference loop and start requesting chunks again.

        Clears buffer and last_action to discard any stale chunks
        that arrived during pause (based on pre-pause observations).
        """
        if task_instruction:
            self._task_instruction = task_instruction

        # Wait for any in-flight fetch thread to notice _paused and exit
        # (skip if called from the fetch thread itself, e.g. via executor callback)
        if (
            self._inference_thread
            and self._inference_thread.is_alive()
            and self._inference_thread is not threading.current_thread()
        ):
            self._inference_thread.join(timeout=2.0)

        with self._buffer_lock:
            self._action_buffer.clear()
        self._last_action = None
        self._requesting = False  # Reset flag so resume always starts a fresh request
        self._paused = False
        self._request_chunk_async()
        logger.info(
            "InferenceManager resumed (%s)", self._service_prefix
        )

