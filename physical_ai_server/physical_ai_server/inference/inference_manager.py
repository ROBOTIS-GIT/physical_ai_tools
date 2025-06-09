import logging
import os
import time
from dataclasses import asdict
from pprint import pformat
import numpy as np
import torch

import sys
dev_lerobot_path = '/root/ros2_ws/src/physical_ai_tools/lerobot'
if dev_lerobot_path not in sys.path:
    sys.path.insert(0, dev_lerobot_path)
from lerobot.common.policies.pretrained import PreTrainedPolicy


class InferenceManager:
    """Manager for inference-related tasks, such as loading policies and datasets."""

    def __init__(
            self,
            policy_type: str,
            policy_path: str,
            device: str = "cuda"):

        self.policy = self._load_policy(policy_type, policy_path)
        self.device = device

    def _load_policy(self, policy_type: str, policy_path: str):
        policy_cls = self._get_policy_class(policy_type)
        policy = policy_cls.from_pretrained(policy_path)
        return policy

    def get_policy_config(self):
        return self.policy.config

    def predict(
            self,
            images: dict[str, np.ndarray],
            state: list[float],
            task_instruction: str = None) -> list:

        observation = self._preprocess(images, state, task_instruction)
        with torch.inference_mode():
            action = self.policy.select_action(observation)
            action = action.squeeze(0).to("cpu").numpy()

        return action

    def _preprocess(
            self,
            images: dict[str, np.ndarray],
            state: list,
            task_instruction: str = None) -> dict:

        observation = self._convert_images2tensors(images)
        observation['observation.state'] = self._convert_np2tensors(state)
        for key in observation.keys():
            observation[key] = observation[key].to(self.device)

        if task_instruction is not None:
            observation['task'] = [task_instruction]

        return observation

    def _convert_images2tensors(
            self,
            images: dict[str, np.ndarray]) -> dict[str, torch.Tensor]:

        processed_images = {}
        for key, value in images.items():
            image = torch.from_numpy(value)
            image = image.to(torch.float32) / 255
            image = image.permute(2, 0, 1)
            image = image.to(self.device, non_blocking=True)
            image = image.unsqueeze(0)
            processed_images['observation.images.' + key] = image

        return processed_images

    def _convert_np2tensors(
            self,
            data: dict[str, np.ndarray]) -> dict[str, torch.Tensor]:

        tensor_data = torch.from_numpy(data)
        tensor_data = tensor_data.to(torch.float32)
        tensor_data = tensor_data.to(self.device, non_blocking=True)
        tensor_data = tensor_data.unsqueeze(0)

        return tensor_data

    def _get_policy_class(self, name: str) -> PreTrainedPolicy:
        """Get the policy's class and config class given a name (matching the policy class' `name` attribute)."""
        if name == "tdmpc":
            from lerobot.common.policies.tdmpc.modeling_tdmpc import TDMPCPolicy

            return TDMPCPolicy
        elif name == "diffusion":
            from lerobot.common.policies.diffusion.modeling_diffusion import DiffusionPolicy

            return DiffusionPolicy
        elif name == "act":
            from lerobot.common.policies.act.modeling_act import ACTPolicy

            return ACTPolicy
        elif name == "vqbet":
            from lerobot.common.policies.vqbet.modeling_vqbet import VQBeTPolicy

            return VQBeTPolicy
        elif name == "pi0":
            from lerobot.common.policies.pi0.modeling_pi0 import PI0Policy

            return PI0Policy
        elif name == "pi0fast":
            from lerobot.common.policies.pi0fast.modeling_pi0fast import PI0FASTPolicy
            return PI0FASTPolicy
        # elif name == "groot-n1":
        #     from Isaac.groot_n1.policies.groot_n1 import GrootN1Policy
        #     return GrootN1Policy
        else:
            raise NotImplementedError(f"Policy with name {name} is not implemented.")
