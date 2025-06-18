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

from lerobot.common.policies.pretrained import PreTrainedPolicy
import numpy as np
import torch


class InferenceManager:

    def __init__(
            self,
            policy_type: str,
            policy_path: str,
            device: str = 'cuda'):

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
            action = action.squeeze(0).to('cpu').numpy()

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
        if name == 'tdmpc':
            from lerobot.common.policies.tdmpc.modeling_tdmpc import TDMPCPolicy

            return TDMPCPolicy
        elif name == 'diffusion':
            from lerobot.common.policies.diffusion.modeling_diffusion import DiffusionPolicy

            return DiffusionPolicy
        elif name == 'act':
            from lerobot.common.policies.act.modeling_act import ACTPolicy

            return ACTPolicy
        elif name == 'vqbet':
            from lerobot.common.policies.vqbet.modeling_vqbet import VQBeTPolicy

            return VQBeTPolicy
        elif name == 'pi0':
            from lerobot.common.policies.pi0.modeling_pi0 import PI0Policy

            return PI0Policy
        elif name == 'pi0fast':
            from lerobot.common.policies.pi0fast.modeling_pi0fast import PI0FASTPolicy
            return PI0FASTPolicy
        # TODO: Uncomment when GrootN1Policy is implemented
        # elif name == 'groot-n1':
        #     from Isaac.groot_n1.policies.groot_n1 import GrootN1Policy
        #     return GrootN1Policy
        else:
            raise NotImplementedError(f'Policy with name {name} is not implemented.')
