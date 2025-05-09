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

from abc import ABC, abstractmethod
from typing import Any


class BasePolicyWrapper(ABC):
    """
    Base class for policy wrappers.
    This class is used to wrap a policy and provide a common interface for all policies.
    """

    def __init__(self, policy):
        self.policy = policy
        self.config = None
        self.input_images = []
        self.input_instructions = ""
    
    @abstractmethod
    def load_dataset(self, dataset_path) -> Any:
        """
        Load the dataset from the given path.
        """
        raise NotImplementedError
    
    @abstractmethod
    def load_model(self, model_path) -> Any:
        """
        Load the model from the given path.
        """
        raise NotImplementedError
    
    @abstractmethod
    def load_model_config(self, config_path) -> Any:
        """
        Load the model config from the given path.
        """
        raise NotImplementedError
    
    @abstractmethod
    def train(self, dataset):
        """
        Train the policy using the provided dataset.
        """
        raise NotImplementedError
    
    def resume_train(self, dataset, checkpoint_path):
        """
        Resume training the policy using the provided dataset and checkpoint path.
        """
        self.load_model(checkpoint_path)
        self.train(dataset)
    
    @abstractmethod
    def inference(self, images, instruction):
        """
        Perform inference using the provided images and instruction.
        """
        raise NotImplementedError

    def evaluate(self, dataset):
        """
        Evaluate the policy using the provided dataset.
        """
    
    def visualize(self, images):
        """
        Visualize the policy using the provided images.
        """

    @abstractmethod
    def transform(self):
        raise NotImplementedError

    @abstractmethod
    def get_action(self, observations) -> bool:
        images = observations["images"]
        instruction = observations["instruction"]

        raise NotImplementedError

    @abstractmethod
    def get_modality_config(self):
        """
        Get modality config from the policy.
        """
        pass