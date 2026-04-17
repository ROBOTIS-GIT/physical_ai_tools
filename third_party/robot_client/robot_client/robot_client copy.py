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
RobotClient - High-level abstraction for robot sensor data and control.

Provides simple Python API over zenoh_ros2_sdk, hiding all Zenoh/ROS2 details.
Users only need to specify robot type to get automatic topic subscription.
"""
import os
import sys
import time
import threading
import logging
from pathlib import Path
from typing import Optional, Union

import numpy as np
import cv2
import yaml

# Add zenoh_ros2_sdk to path if not already available
_SDK_PATH = os.environ.get("ZENOH_SDK_PATH", "")
if _SDK_PATH and _SDK_PATH not in sys.path:
    sys.path.insert(0, _SDK_PATH)

from zenoh_ros2_sdk import ROS2Subscriber, ROS2Publisher

logger = logging.getLogger("robot_client")


def _find_config(robot_type: str) -> Path:
    """Find robot config YAML file."""
    # 1) Check in package config directory
    pkg_config = Path(__file__).parent / "config" / f"{robot_type}.yaml"
    if pkg_config.exists():
        return pkg_config
    # 2) Check environment variable
    config_dir = os.environ.get("ROBOT_CLIENT_CONFIG_DIR", "")
    if config_dir:
        ext_config = Path(config_dir) / f"{robot_type}.yaml"
        if ext_config.exists():
            return ext_config
    raise FileNotFoundError(
        f"Robot config not found for '{robot_type}'. "
        f"Searched: {pkg_config}"
    )


class RobotClient:
    """High-level robot interface over zenoh_ros2_sdk.

    Usage:
        robot = RobotClient("ffw_sg2_rev1")
        robot.wait_for_ready(timeout=10.0)
        images = robot.get_images()
        joints = robot.get_joint_positions()
    """

    def __init__(
        self,
        robot_type: str,
        sync_check: bool = False,
        sync_threshold_ms: float = 33.0,
        router_ip: str = "127.0.0.1",
        router_port: int = 7447,
        domain_id: Optional[int] = None,
    ):
        config_path = _find_config(robot_type)
        with open(config_path) as f:
            self._config = yaml.safe_load(f)

        self._robot_type = robot_type
        self._sync_check = sync_check
        self._sync_threshold_ms = sync_threshold_ms
        self._router_ip = router_ip
        self._router_port = router_port

        # Thread-safe data stores
        self._lock = threading.Lock()
        self._images: dict[str, np.ndarray] = {}
        self._image_timestamps: dict[str, float] = {}
        self._joint_positions: dict[str, np.ndarray] = {}
        self._joint_velocities: dict[str, np.ndarray] = {}
        self._joint_efforts: dict[str, np.ndarray] = {}
        self._joint_timestamps: dict[str, float] = {}
        self._sensors: dict[str, dict] = {}
        self._sensor_timestamps: dict[str, float] = {}
        self._task_instruction: str = ""

        # Subscribers and publishers
        self._subscribers: list = []
        self._publishers: dict[str, ROS2Publisher] = {}

        self._closed = False

        # Initialize
        self._init_subscriptions()
        self._init_publishers()
        logger.info(f"RobotClient initialized: {robot_type} "
                     f"({len(self._config.get('cameras', {}))} cameras, "
                     f"{len(self._config.get('joint_groups', {}))} joint groups)")

    # ------------------------------------------------------------------ #
    # Initialization
    # ------------------------------------------------------------------ #

    def _init_subscriptions(self):
        """Subscribe to all configured topics."""
        # Cameras
        for cam_name, cam_cfg in self._config.get("cameras", {}).items():
            sub = ROS2Subscriber(
                topic=cam_cfg["topic"],
                msg_type=cam_cfg["msg_type"],
                callback=lambda msg, name=cam_name: self._update_image(name, msg),
                # router_ip=self._router_ip,
                # router_port=self._router_port,
                # domain_id=self._domain_id,
            )
            self._subscribers.append(sub)
            logger.debug(f"Subscribed camera: {cam_name} -> {cam_cfg['topic']}")

        # Joint groups
        for group_name, group_cfg in self._config.get("joint_groups", {}).items():
            sub = ROS2Subscriber(
                topic=group_cfg["topic"],
                msg_type=group_cfg["msg_type"],
                callback=lambda msg, name=group_name: self._update_joint(name, msg),
                # router_ip=self._router_ip,
                # router_port=self._router_port,
                # domain_id=self._domain_id,
            )
            self._subscribers.append(sub)
            logger.debug(f"Subscribed joint: {group_name} -> {group_cfg['topic']}")

        # Additional sensors
        for sensor_name, sensor_cfg in self._config.get("sensors", {}).items():
            sub = ROS2Subscriber(
                topic=sensor_cfg["topic"],
                msg_type=sensor_cfg["msg_type"],
                callback=lambda msg, name=sensor_name: self._update_sensor(name, msg),
                # router_ip=self._router_ip,
                # router_port=self._router_port,
                # domain_id=self._domain_id,
            )
            self._subscribers.append(sub)
            logger.debug(f"Subscribed sensor: {sensor_name} -> {sensor_cfg['topic']}")

    def _init_publishers(self):
        """Create publishers for leader joint groups only."""
        for group_name, group_cfg in self._config.get("joint_groups", {}).items():
            if group_cfg.get("role") == "leader":
                self._publishers[group_name] = ROS2Publisher(
                    topic=group_cfg["topic"],
                    msg_type="sensor_msgs/msg/JointState",
                    # router_ip=self._router_ip,
                    # router_port=self._router_port,
                    # domain_id=self._domain_id,
                )
                logger.debug(f"Publisher created: {group_name} -> {group_cfg['topic']}")

        # Velocity publisher (cmd_vel)
        sensors = self._config.get("sensors", {})
        if "cmd_vel" in sensors:
            self._publishers["cmd_vel"] = ROS2Publisher(
                topic=sensors["cmd_vel"]["topic"],
                msg_type="geometry_msgs/msg/Twist",
                # router_ip=self._router_ip,
                # router_port=self._router_port,
                # domain_id=self._domain_id,
            )

    # ------------------------------------------------------------------ #
    # Callback handlers
    # ------------------------------------------------------------------ #

    def _update_image(self, cam_name: str, msg):
        """CompressedImage -> BGR numpy array."""
        try:
            data = msg.data
            if isinstance(data, (list, tuple)):
                data = bytes(data)
            buf = np.frombuffer(data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if img is not None:
                with self._lock:
                    self._images[cam_name] = img
                    self._image_timestamps[cam_name] = time.time()
        except Exception as e:
            logger.warning(f"Failed to decode image from {cam_name}: {e}")

    def _update_joint(self, group_name: str, msg):
        """JointState -> np.ndarray(float32)."""
        try:
            position = list(msg.position) if hasattr(msg.position, '__iter__') else []
            velocity = list(msg.velocity) if hasattr(msg.velocity, '__iter__') else []
            effort = list(msg.effort) if hasattr(msg.effort, '__iter__') else []
            with self._lock:
                if position:
                    self._joint_positions[group_name] = np.array(position, dtype=np.float32)
                if velocity:
                    self._joint_velocities[group_name] = np.array(velocity, dtype=np.float32)
                if effort:
                    self._joint_efforts[group_name] = np.array(effort, dtype=np.float32)
                self._joint_timestamps[group_name] = time.time()
        except Exception as e:
            logger.warning(f"Failed to parse joint from {group_name}: {e}")

    def _update_sensor(self, sensor_name: str, msg):
        """Parse sensor messages (Odometry, Twist, etc.)."""
        try:
            data = {}
            if sensor_name == "odom":
                pos = msg.pose.pose.position
                ori = msg.pose.pose.orientation
                lin = msg.twist.twist.linear
                ang = msg.twist.twist.angular
                data = {
                    "position": np.array([pos.x, pos.y, pos.z], dtype=np.float32),
                    "orientation": np.array([ori.x, ori.y, ori.z, ori.w], dtype=np.float32),
                    "linear_velocity": np.array([lin.x, lin.y, lin.z], dtype=np.float32),
                    "angular_velocity": np.array([ang.x, ang.y, ang.z], dtype=np.float32),
                }
            elif sensor_name == "cmd_vel":
                data = {
                    "linear": np.array([msg.linear.x, msg.linear.y, msg.linear.z], dtype=np.float32),
                    "angular": np.array([msg.angular.x, msg.angular.y, msg.angular.z], dtype=np.float32),
                }
            else:
                data = {"raw": str(msg)}

            with self._lock:
                self._sensors[sensor_name] = data
                self._sensor_timestamps[sensor_name] = time.time()
        except Exception as e:
            logger.warning(f"Failed to parse sensor {sensor_name}: {e}")

    # ------------------------------------------------------------------ #
    # Image API
    # ------------------------------------------------------------------ #

    @property
    def camera_names(self) -> list[str]:
        return list(self._config.get("cameras", {}).keys())

    def get_images(
        self,
        resize: Optional[tuple[int, int]] = None,
        format: str = "bgr",
    ) -> dict[str, np.ndarray]:
        """Get all camera images.

        Args:
            resize: Optional (width, height) tuple. None = original size.
            format: "bgr" (default) or "rgb".
        """
        with self._lock:
            result = {k: v.copy() for k, v in self._images.items()}
        if resize:
            result = {k: cv2.resize(v, resize) for k, v in result.items()}
        if format == "rgb":
            result = {k: cv2.cvtColor(v, cv2.COLOR_BGR2RGB) for k, v in result.items()}
        return result

    def get_image(
        self,
        camera_name: str,
        resize: Optional[tuple[int, int]] = None,
        format: str = "bgr",
    ) -> Optional[np.ndarray]:
        """Get single camera image."""
        with self._lock:
            img = self._images.get(camera_name)
            if img is None:
                return None
            img = img.copy()
        if resize:
            img = cv2.resize(img, resize)
        if format == "rgb":
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    def is_image_ready(self, camera_name: str) -> bool:
        with self._lock:
            return camera_name in self._images

    def get_image_timestamp(self, camera_name: str) -> Optional[float]:
        with self._lock:
            return self._image_timestamps.get(camera_name)

    # ------------------------------------------------------------------ #
    # Joint API
    # ------------------------------------------------------------------ #

    @property
    def joint_group_names(self) -> list[str]:
        return list(self._config.get("joint_groups", {}).keys())

    @property
    def total_dof(self) -> int:
        return self._config.get("total_dof", 0)

    def get_joint_names(self, group_name: str) -> list[str]:
        cfg = self._config.get("joint_groups", {}).get(group_name, {})
        return cfg.get("joint_names", [])

    def get_dof(self, group_name: str) -> int:
        cfg = self._config.get("joint_groups", {}).get(group_name, {})
        return cfg.get("dof", 0)

    def get_joint_positions(
        self, group: Optional[str] = None
    ) -> Union[dict[str, np.ndarray], np.ndarray]:
        """Get joint positions. Returns dict if no group, or np.ndarray for specific group."""
        with self._lock:
            if group:
                arr = self._joint_positions.get(group)
                return arr.copy() if arr is not None else np.array([], dtype=np.float32)
            return {k: v.copy() for k, v in self._joint_positions.items()}

    def get_joint_velocities(
        self, group: Optional[str] = None
    ) -> Union[dict[str, np.ndarray], np.ndarray]:
        with self._lock:
            if group:
                arr = self._joint_velocities.get(group)
                return arr.copy() if arr is not None else np.array([], dtype=np.float32)
            return {k: v.copy() for k, v in self._joint_velocities.items()}

    def get_joint_efforts(
        self, group: Optional[str] = None
    ) -> Union[dict[str, np.ndarray], np.ndarray]:
        with self._lock:
            if group:
                arr = self._joint_efforts.get(group)
                return arr.copy() if arr is not None else np.array([], dtype=np.float32)
            return {k: v.copy() for k, v in self._joint_efforts.items()}

    def is_joint_ready(self, group_name: str) -> bool:
        with self._lock:
            return group_name in self._joint_positions

    def get_joint_timestamp(self, group_name: str) -> Optional[float]:
        with self._lock:
            return self._joint_timestamps.get(group_name)

    # ------------------------------------------------------------------ #
    # Sensor API
    # ------------------------------------------------------------------ #

    def get_odom(self) -> Optional[dict]:
        with self._lock:
            return self._sensors.get("odom")

    def is_sensor_ready(self, sensor_name: str) -> bool:
        with self._lock:
            return sensor_name in self._sensors

    # ------------------------------------------------------------------ #
    # Task instruction
    # ------------------------------------------------------------------ #

    def set_task_instruction(self, instruction: str):
        self._task_instruction = instruction

    @property
    def task_instruction(self) -> str:
        return self._task_instruction

    # ------------------------------------------------------------------ #
    # Action output
    # ------------------------------------------------------------------ #

    def set_joint_positions(
        self,
        group_or_dict: Union[str, dict],
        positions: Optional[list] = None,
    ):
        """Publish joint positions to leader group(s).

        Args:
            group_or_dict: Group name (str) or dict of {group: positions}.
            positions: Position values (list/ndarray) when group_or_dict is str.
        """
        if isinstance(group_or_dict, dict):
            for group, pos in group_or_dict.items():
                self.set_joint_positions(group, pos)
            return

        group = group_or_dict
        cfg = self._config.get("joint_groups", {}).get(group, {})
        if cfg.get("role") == "follower":
            logger.warning(f"Cannot set positions for follower group: {group}")
            return

        pub = self._publishers.get(group)
        if pub is None:
            logger.warning(f"No publisher for group: {group}")
            return

        joint_names = cfg.get("joint_names", [])
        pos_list = list(positions) if positions is not None else []

        pub.publish(
            name=joint_names,
            position=pos_list,
            velocity=[],
            effort=[],
        )

    def set_velocity(
        self,
        linear_x: float = 0.0,
        linear_y: float = 0.0,
        angular_z: float = 0.0,
    ):
        """Publish velocity command (Twist message)."""
        pub = self._publishers.get("cmd_vel")
        if pub is None:
            logger.warning("No cmd_vel publisher available")
            return

        from zenoh_ros2_sdk import get_message_class
        Vector3 = get_message_class("geometry_msgs/msg/Vector3")
        pub.publish(
            linear=Vector3(x=linear_x, y=linear_y, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=angular_z),
        )

    def execute_action_chunk(
        self,
        action_chunk: np.ndarray,
        action_keys: list[str],
        frequency: float = 10.0,
    ):
        """Execute action chunk synchronously (blocking).

        Iterates through the chunk, publishing one action per tick at the
        given frequency. This is for standalone/demo scripts only.
        For async inference, use the service method (physical_ai_server
        InferenceManager handles L2 alignment + 10Hz pop).

        Args:
            action_chunk: (T, D) array of actions.
            action_keys: Joint group names corresponding to action dims.
            frequency: Publishing frequency in Hz.
        """
        interval = 1.0 / frequency
        for action in action_chunk:
            t0 = time.time()
            # Distribute action dims across groups
            offset = 0
            for key in action_keys:
                dof = self.get_dof(key)
                if dof > 0:
                    pos = action[offset:offset + dof]
                    self.set_joint_positions(key, pos.tolist())
                    offset += dof
            # Sleep for remaining interval
            elapsed = time.time() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ------------------------------------------------------------------ #
    # Observation
    # ------------------------------------------------------------------ #

    def get_observation(
        self,
        resize: Optional[tuple[int, int]] = None,
        format: str = "bgr",
    ) -> Optional[dict]:
        """Get full observation for inference.

        Returns:
            Dict with images, joint_positions, task_instruction.
            None if sync_check is enabled and data is out of sync.
        """
        if self._sync_check and not self._check_sync():
            return None
        return {
            "images": self.get_images(resize=resize, format=format),
            "joint_positions": self.get_joint_positions(),
            "task_instruction": self._task_instruction,
        }

    def _check_sync(self) -> bool:
        """Check if image and joint timestamps are within threshold."""
        threshold_s = self._sync_threshold_ms / 1000.0
        with self._lock:
            if not self._image_timestamps or not self._joint_timestamps:
                return False
            img_times = list(self._image_timestamps.values())
            jnt_times = list(self._joint_timestamps.values())

        latest_img = max(img_times) if img_times else 0
        latest_jnt = max(jnt_times) if jnt_times else 0
        return abs(latest_img - latest_jnt) < threshold_s

    # ------------------------------------------------------------------ #
    # Readiness / waiting
    # ------------------------------------------------------------------ #

    def wait_for_ready(self, timeout: float = 10.0) -> bool:
        """Wait until at least one frame from all sensors is received."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._all_ready():
                logger.info("All sensors ready")
                return True
            time.sleep(0.1)
        # Log what's missing
        missing = self._get_missing()
        logger.warning(f"Timeout waiting for sensors. Missing: {missing}")
        return False

    def wait_for_image(self, camera_name: str, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_image_ready(camera_name):
                return True
            time.sleep(0.1)
        return False

    def wait_for_joint(self, group_name: str, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_joint_ready(group_name):
                return True
            time.sleep(0.1)
        return False

    def _all_ready(self) -> bool:
        with self._lock:
            for cam in self._config.get("cameras", {}):
                if cam not in self._images:
                    return False
            for group in self._config.get("joint_groups", {}):
                if group not in self._joint_positions:
                    return False
            return True

    def _get_missing(self) -> list[str]:
        missing = []
        with self._lock:
            for cam in self._config.get("cameras", {}):
                if cam not in self._images:
                    missing.append(f"camera:{cam}")
            for group in self._config.get("joint_groups", {}):
                if group not in self._joint_positions:
                    missing.append(f"joint:{group}")
        return missing

    # ------------------------------------------------------------------ #
    # Info / diagnostics
    # ------------------------------------------------------------------ #

    def get_status(self) -> dict:
        """Get current status of all subscriptions."""
        with self._lock:
            return {
                "robot_type": self._robot_type,
                "cameras": {
                    name: {
                        "ready": name in self._images,
                        "shape": self._images[name].shape if name in self._images else None,
                        "timestamp": self._image_timestamps.get(name),
                    }
                    for name in self._config.get("cameras", {})
                },
                "joint_groups": {
                    name: {
                        "ready": name in self._joint_positions,
                        "dof": len(self._joint_positions[name]) if name in self._joint_positions else 0,
                        "timestamp": self._joint_timestamps.get(name),
                    }
                    for name in self._config.get("joint_groups", {})
                },
                "sensors": {
                    name: {
                        "ready": name in self._sensors,
                        "timestamp": self._sensor_timestamps.get(name),
                    }
                    for name in self._config.get("sensors", {})
                },
            }

    # ------------------------------------------------------------------ #
    # Cleanup
    # ------------------------------------------------------------------ #

    def close(self):
        """Close all subscriptions and publishers."""
        if hasattr(self, '_closed') and self._closed:
            return
        self._closed = True
        for sub in self._subscribers:
            try:
                sub.close()
            except Exception as e:
                logger.debug(f"Error closing subscriber: {e}")
        self._subscribers.clear()
        for pub in self._publishers.values():
            try:
                pub.close()
            except Exception as e:
                logger.debug(f"Error closing publisher: {e}")
        self._publishers.clear()
        logger.info("RobotClient closed")

    def __del__(self):
        self.close()
