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

"""GR00T N1.6 inference engine."""
import logging
import os
import time
from typing import Optional

import cv2
import numpy as np
import torch

import sys

import gr00t.model  # noqa: F401 - register custom models
from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.policy.gr00t_policy import Gr00tPolicy
from robot_client import RobotClient

# Add GR00T root to path for deployment script imports
_GROOT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.path.isdir(os.path.join(_GROOT_ROOT, "scripts", "deployment")):
    _groot_path = _GROOT_ROOT
elif os.path.isdir("/gr00t/scripts/deployment"):
    _groot_path = "/gr00t"
else:
    _groot_path = None

if _groot_path and _groot_path not in sys.path:
    sys.path.insert(0, _groot_path)

# TensorRT DiT acceleration - reuse existing GR00T deployment code
from scripts.deployment.standalone_inference_script import (  # noqa: E402
    replace_dit_with_tensorrt,
)
from scripts.deployment.export_onnx_n1d6 import DiTInputCapture, export_dit_to_onnx  # noqa: E402


def build_trt_engine(policy: Gr00tPolicy, observation: dict, engine_path: str):
    """Export DiT to ONNX and build TensorRT engine automatically.

    Uses DiTInputCapture and export_dit_to_onnx from GR00T deployment scripts,
    then builds the TRT engine via build_tensorrt_engine.build_engine().
    Takes ~3-5 min on Orin.
    """
    from scripts.deployment.build_tensorrt_engine import build_engine

    logger = logging.getLogger("groot_inference")
    onnx_path = engine_path.replace(".trt", ".onnx")

    # Step 1: Capture DiT input shapes via hook
    logger.info("Capturing DiT input shapes...")
    capture = DiTInputCapture()
    hook = policy.model.action_head.model.register_forward_pre_hook(
        capture.hook_fn, with_kwargs=True
    )
    with torch.inference_mode():
        policy.get_action(observation)
    hook.remove()

    if not capture.captured:
        raise RuntimeError("Failed to capture DiT inputs")

    # Step 2: Export DiT to ONNX
    # Patch torch.onnx.export to force dynamo=False (avoids torch.export failures
    # with DiT's dynamic shapes, without modifying the upstream GR00T code)
    _orig_export = torch.onnx.export
    def _patched_export(*args, **kwargs):
        kwargs.setdefault("dynamo", False)
        return _orig_export(*args, **kwargs)
    torch.onnx.export = _patched_export
    try:
        export_dit_to_onnx(
            policy=policy,
            captured_inputs=capture,
            output_path=onnx_path,
            use_bf16=True,
        )
    finally:
        torch.onnx.export = _orig_export

    logger.info("ONNX exported: %s", onnx_path)

    # Step 3: Build TensorRT engine
    min_shapes = {
        "sa_embs": (1, 1, 1536),
        "vl_embs": (1, 1, 2048),
        "timestep": (1,),
        "image_mask": (1, 1),
        "backbone_attention_mask": (1, 1),
    }
    opt_shapes = {
        "sa_embs": (1, 51, 1536),
        "vl_embs": (1, 122, 2048),
        "timestep": (1,),
        "image_mask": (1, 122),
        "backbone_attention_mask": (1, 122),
    }
    max_shapes = {
        "sa_embs": (1, 256, 1536),
        "vl_embs": (1, 512, 2048),
        "timestep": (1,),
        "image_mask": (1, 512),
        "backbone_attention_mask": (1, 512),
    }

    build_engine(
        onnx_path=onnx_path,
        engine_path=engine_path,
        precision="bf16",
        workspace_mb=8192,
        min_shapes=min_shapes,
        opt_shapes=opt_shapes,
        max_shapes=max_shapes,
    )

    # Clean up ONNX and external data files (no longer needed after TRT build)
    onnx_dir = os.path.dirname(onnx_path)
    onnx_basename = os.path.splitext(os.path.basename(onnx_path))[0]
    keep_exts = (".trt", ".json", ".safetensors", ".py", ".txt", ".model", ".md", ".bin")
    for f in os.listdir(onnx_dir):
        fpath = os.path.join(onnx_dir, f)
        if not os.path.isfile(fpath):
            continue
        # Remove the ONNX file itself
        if f.endswith(".onnx"):
            os.remove(fpath)
            continue
        # Remove external data files (created by ONNX export, no standard extension)
        if not f.endswith(keep_exts):
            os.remove(fpath)
    logger.info("Cleaned up ONNX export files from: %s", onnx_dir)


class GR00TInference:
    """Encapsulates GR00T policy loading, observation building, and inference."""

    logger = logging.getLogger("groot_inference")
    IMAGE_SIZE = (256, 256)
    DEFAULT_EMBODIMENT_TAG = "new_embodiment"
    ROTATE_MAP = {
        90: cv2.ROTATE_90_CLOCKWISE,
        180: cv2.ROTATE_180,
        270: cv2.ROTATE_90_COUNTERCLOCKWISE,
    }

    def __init__(self):
        self.policy: Optional[Gr00tPolicy] = None
        self.robot: Optional[RobotClient] = None
        self._loaded_model_path: Optional[str] = None  # track cached policy path
        self.policy_info: dict = {
            "video": [],       # e.g. ["cam_left_head", "cam_left_wrist", ...]
            "state": [],       # e.g. ["arm_left", "arm_right"]
            "action": [],      # e.g. ["arm_left", "arm_right"]
            "language": [],    # e.g. ["annotation.human.task_description"]
        }
        self.robot_info: dict = {
            "cameras": [],         # active camera names matched with model
            "joints": {},          # modality_key -> yaml_group mapping
            "camera_rotations": {},  # camera_name -> rotation_deg
        }

    @property
    def is_ready(self) -> bool:
        return self.policy is not None and self.robot is not None

    def load_policy(self, request) -> dict:
        """Load GR00T policy and create RobotClient for sensor data."""
        model_path = request.model_path
        robot_type = request.robot_type

        try:
            # Reuse cached policy if the same model is already loaded.
            # Only reload robot client (subscribers) on restart.
            if self.policy is not None and self._loaded_model_path == model_path:
                self.logger.info("Reusing cached policy: %s", model_path)
                if self.robot is not None:
                    self.robot.close()
                    self.robot = None
                self.init_policy_info()
                self.init_robot_info(robot_type)
                self.robot.wait_for_ready(timeout=10.0)
                return {
                    "success": True,
                    "message": "GR00T inference restarted (policy cached)",
                    "action_keys": list(self.policy_info["action"]),
                }

            self.logger.info("Loading GR00T policy from: %s", model_path)

            self.policy = Gr00tPolicy(
                embodiment_tag=EmbodimentTag.NEW_EMBODIMENT,
                model_path=model_path,
                device="cuda",
            )
            self._loaded_model_path = model_path

            self.init_policy_info()
            self.init_robot_info(robot_type)
            self.robot.wait_for_ready(timeout=10.0)

            # TensorRT acceleration for DiT (Action Head).
            # Engine is model-specific (weights baked in) and GPU-specific,
            # so it lives next to the checkpoint. Auto-build on first use.
            trt_path = os.path.join(model_path, "dit_model_bf16.trt")
            try:
                if not os.path.exists(trt_path):
                    self.logger.info("No TRT engine found, building automatically...")
                    dummy_obs = self._build_dummy_observation(request.task_instruction)
                    build_trt_engine(self.policy, dummy_obs, trt_path)

                replace_dit_with_tensorrt(self.policy, trt_path)
                self.logger.info("DiT accelerated with TensorRT: %s", trt_path)
            except Exception as e:
                self.logger.warning(
                    "TensorRT acceleration unavailable, using PyTorch Eager: %s", e
                )

            return {
                "success": True,
                "message": "GR00T inference started",
                "action_keys": list(self.policy_info["action"]),
            }
        except Exception as e:
            self.logger.error("Failed to start inference: %s", e, exc_info=True)
            return self.fail(str(e))

    def _build_dummy_observation(self, task_instruction: str = "") -> dict:
        """Build a real observation from robot sensors for TRT engine building."""
        images = self.robot.get_images(format="rgb")
        joints = self.robot.get_joint_positions()
        task = task_instruction or "dummy task"
        return self.preprocess(images, joints, task)

    def init_policy_info(self) -> None:
        """Read video/state/action/language keys from the loaded policy."""
        mc = getattr(self.policy, "modality_configs", None)
        if mc is None:
            mc = self.policy.processor.get_modality_configs().get(
                self.DEFAULT_EMBODIMENT_TAG, {}
            )

        for modality in ("video", "state", "action", "language"):
            if modality not in mc:
                self.policy_info[modality] = []
                continue
            entry = mc[modality]
            self.policy_info[modality] = getattr(
                entry, "modality_keys",
                entry.get("modality_keys", []) if isinstance(entry, dict) else [],
            )

        self.logger.info("Policy info: %s", self.policy_info)

    def init_robot_info(self, robot_type: str) -> None:
        """Create RobotClient and resolve active cameras/joints from YAML."""
        self.robot = RobotClient(robot_type)
        cam_config = self.robot._config.get("cameras", {})

        self.robot_info["cameras"] = [
            k for k in self.robot.camera_names if k in self.policy_info["video"]
        ]
        self.robot_info["camera_rotations"] = {
            name: cfg.get("rotation_deg", 0)
            for name, cfg in cam_config.items()
            if cfg.get("rotation_deg", 0) != 0
        }

        joints = {}
        for group in self.robot.joint_group_names:
            if "follower" not in group:
                continue
            modality_key = group.removeprefix("follower_")
            if modality_key in self.policy_info["state"]:
                joints[modality_key] = group
        self.robot_info["joints"] = joints

        # Sensor-backed state modalities. The training pipeline stores mobile
        # as a joint-like 3-dim modality, but robot_client keeps odom/cmd_vel
        # as sensors (semantically correct — they aren't joints). Bridge here.
        sensor_states = {}
        sensors_cfg = self.robot._config.get("sensors", {})
        if "mobile" in self.policy_info["state"] and "odom" in sensors_cfg:
            sensor_states["mobile"] = "odom"
        self.robot_info["sensor_states"] = sensor_states

        self.logger.info("Robot info: %s", self.robot_info)

    def get_action_chunk(self, request) -> dict:
        """Build observation from RobotClient, run inference, return action chunk."""
        if not self.is_ready:
            return self.fail("Not in inference mode")

        try:
            images = self.robot.get_images(format="rgb")
            joints = self.robot.get_joint_positions()
            task = request.task_instruction

            observation = self.preprocess(images, joints, task)
            if "success" in observation:
                return observation

            action, info = self.policy.get_action(observation)
            return self.postprocess_action(action)

        except Exception as e:
            self.logger.error("Inference failed: %s", e, exc_info=True)
            return self.fail(str(e))

    def preprocess(self, images: dict, joints: dict, task: str) -> dict:
        """Build observation dict from raw sensor data. Returns fail dict on error."""
        if not images or not joints:
            return self.fail("No recent observations from sensors")

        video_obs = {}
        for cam_key in self.robot_info["cameras"]:
            img = images.get(cam_key)
            if img is None:
                return self.fail(f"Missing camera: {cam_key}")
            rotation = self.robot_info["camera_rotations"].get(cam_key)
            if rotation and rotation in self.ROTATE_MAP:
                img = cv2.rotate(img, self.ROTATE_MAP[rotation])
            video_obs[cam_key] = img[np.newaxis, np.newaxis, ...]  # (1,1,H,W,C)

        state_obs = {}
        for modality_key, yaml_group in self.robot_info["joints"].items():
            positions = joints.get(yaml_group)
            if positions is None or len(positions) == 0:
                return self.fail(f"Missing joint group: {modality_key}")
            state_obs[modality_key] = positions[np.newaxis, np.newaxis, :]  # (1,1,D)

        # Sensor-backed state modalities (e.g. mobile ← odom).
        for modality_key, sensor_name in self.robot_info.get("sensor_states", {}).items():
            if sensor_name == "odom":
                odom = self.robot.get_odom()
                if odom is None:
                    return self.fail(f"Missing sensor: {sensor_name}")
                vec = np.array([
                    float(odom["linear_velocity"][0]),
                    float(odom["linear_velocity"][1]),
                    float(odom["angular_velocity"][2]),
                ], dtype=np.float32)
                state_obs[modality_key] = vec[np.newaxis, np.newaxis, :]  # (1,1,3)
            else:
                return self.fail(f"Unsupported sensor modality: {sensor_name}")

        language_obs = {key: [[task]] for key in self.policy_info["language"]}

        return {
            "video": video_obs,
            "state": state_obs,
            "language": language_obs,
        }

    def postprocess_action(self, action: dict) -> dict:
        chunks = [
            action[key][0]  # remove batch dim
            for key in self.policy_info["action"]
            if key in action and isinstance(action[key], np.ndarray)
        ]
        if not chunks:
            return self.fail("No action output from policy")

        chunk = np.concatenate(chunks, axis=1)  # (T, D_total)
        T, D = chunk.shape
        self.logger.info("Action chunk: T=%d, D=%d", T, D)
        return {
            "success": True,
            "action_chunk": np.asarray(chunk.flatten(), dtype=np.float64).tolist(),
            "chunk_size": T,
            "action_dim": D,
        }

    def cleanup(self) -> None:
        """Release robot resources. Policy is kept cached for fast restart."""
        if self.robot is not None:
            self.robot.close()
            self.robot = None

        self.policy_info = {k: [] for k in self.policy_info}
        self.robot_info = {
            "cameras": [],
            "joints": {},
            "camera_rotations": {},
        }

    @staticmethod
    def fail(message: str) -> dict:
        return {"success": False, "message": message}

