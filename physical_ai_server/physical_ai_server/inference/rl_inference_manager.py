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
# Author: Dongyun Kim, Taehyeong Kim

import os

import numpy as np
from physical_ai_server.utils.read_file import read_yaml_file
import torch


import argparse
import math

import numpy as np
import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial.transform import Rotation

import sys
import os

class RLInferenceManager:

    def __init__(
            self,
            device: str = 'cuda',
            inference_fps: int = 1000):

        self.device = device
        self.inference_fps = inference_fps
        self.policy_type = None
        self.policy_path = None
        self.policy = None
        self.policy_environment = None
        self.iteration = 0
        self.target_command = np.zeros(7)  # [x, y, z, qw, qx, qy, qz]

    def validate_policy(self, policy_path: str) -> bool:
        result_message = ''
        # For RL policies, we assume the path is valid if it exists
        if not os.path.exists(policy_path) or not os.path.isdir(policy_path):
            return False, f'Policy path {policy_path} does not exist or is not a directory.'
        
        policy_environment_path = os.path.join(policy_path, 'env.yaml')
        if not os.path.exists(policy_environment_path):
            result_message = f'Environment configuration file env.yaml does not exist in {policy_environment_path}.'
            return False, result_message

        policy_environment = read_yaml_file(policy_environment_path)
        if policy_environment is None:
            result_message = f'Failed to read environment configuration from {policy_environment_path}.'
            return False, result_message

        self.policy_path = policy_path
        self.policy_environment = policy_environment
        self._robot_environment_setup()

        return True, f'Policy path {policy_path} is valid.'

    def load_policy(self):
        policy_path = os.path.join(self.policy_path, 'policy.pt')
        try:
            self.policy = torch.jit.load(policy_path, map_location=self.device)
            self.policy.to(self.device)
            return True
        except Exception as e:
            print(f'Failed to load RL policy from {policy_path}: {e}')
            return False

    def clear_policy(self):
        if hasattr(self, 'policy'):
            del self.policy
            self.policy = None
        else:
            print('No policy to clear.')

    def get_policy_config(self):
        return None
        
    def predict(
            self,
            images: dict[str, np.ndarray],
            state: list[float],
            task_instruction: str = None,
            additional_params: dict = None) -> tuple[np.ndarray, list[str]]:

        observation = self._preprocess(images, state, task_instruction, additional_params)
        action, action_joint_names = self._action_from_policy(observation, task_instruction)

        return action, action_joint_names

    def _preprocess(
            self,
            images: dict[str, np.ndarray],
            state: list[float],
            task_instruction: str = None,
            additional_params: dict = None) -> np.ndarray:

        if additional_params is None:
            raise ValueError("additional_params is required for preprocessing.")

        if task_instruction == "omy_reach":
            return self._preprocess_omy_reach(state, additional_params)

        else:
            raise NotImplementedError(f'Preprocessing for task {task_instruction} is not implemented.')

    def _action_from_policy(self, observation: np.ndarray, task_instruction: str) -> np.ndarray:
        """
        Get action from the policy based on the observation.
        This method should be overridden in subclasses if needed.
        """
        if self.policy is None:
            raise ValueError("Policy is not loaded. Call `load_policy()` first.")

        action = []
        with torch.inference_mode():
            obs_tensor = torch.from_numpy(observation).view(1, -1).float().to(self.device)
            raw_action = self.policy(obs_tensor).detach().view(-1).cpu().numpy()

        if task_instruction == "omy_reach":

            self.previous_action = raw_action.copy()

            for i, name in enumerate(self.arm_action_joint_names):
                default_value = self.default_joint_positions.get(name)
                if default_value is None:
                    raise ValueError(f"Default joint position for '{name}' not found in YAML.")
                action.append(default_value + raw_action[i] * self.arm_action_scale)

            return action, self.arm_action_joint_names
        else:
            raise NotImplementedError(f'Action extraction for task {task_instruction} is not implemented.')

    def _preprocess_omy_reach(self, state: list[float], additional_params: dict) -> np.ndarray:
        """
        Preprocess observation for omy_reach task.
        Includes joint offsets, velocities, command pose, and previous action.
        """
        input_joint_names = additional_params.get('joint_names')
        input_joint_velocity = additional_params.get('joint_velocity')
        if input_joint_names is None or input_joint_velocity is None:
            raise ValueError("'joint_names' and 'joint_velocity' must be provided in additional_params.")

        # Build joint position/velocity mapping
        joint_position = dict(zip(input_joint_names, state))
        joint_velocity = dict(zip(input_joint_names, input_joint_velocity))

        # Extract arrays in observation order
        joint_position_array = np.array(
            [joint_position.get(name, 0.0) for name in self.observation_joint_names],
            dtype=np.float32
        )
        joint_default_position_array = np.array(
            [self.default_joint_positions.get(name, 0.0) for name in self.observation_joint_names],
            dtype=np.float32
        )
        joint_velocity_array = np.array(
            [joint_velocity.get(name, 0.0) for name in self.observation_joint_names],
            dtype=np.float32
        )

        command_interval = int(3.0 * self.inference_fps)
        phase = self.iteration % command_interval
        self.iteration += 1

        if phase == 0:
            self.target_command = self._sample_reach_random_pose()
            self.get_logger().info(f"New target command: {np.round(self.target_command, 4)}")

        # Ensure previous_action exists
        if not hasattr(self, "previous_action"):
            self.previous_action = np.zeros(len(self.arm_action_joint_names), dtype=np.float32)

        # Compute position offset
        joint_pos_offset = joint_position_array - joint_default_position_array

        return np.concatenate([
            joint_pos_offset,
            joint_velocity_array,
            self.target_command,
            self.previous_action
        ], dtype=np.float32)

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
            data):
        if isinstance(data, list):
            data = np.array(data)
        tensor_data = torch.from_numpy(data)
        tensor_data = tensor_data.to(torch.float32)
        tensor_data = tensor_data.to(self.device, non_blocking=True)
        tensor_data = tensor_data.unsqueeze(0)

        return tensor_data

    def _get_policy_class(self, name: str):
        pass

    @staticmethod
    def get_available_policies() -> list[str]:
        return [
            'rsl_rl',
        ]

    @staticmethod
    def get_saved_policies():
        pass

    def _robot_environment_setup(self):
        """Set up the RL environment from the loaded YAML configuration."""
        if self.policy_environment is None:
            raise ValueError("YAML not loaded. Call `load_policy_yaml()` first.")

        # Example: Load specific parameters from the YAML
        self.arm_action_scale = self._get_arm_action_scale()
        self.gripper_action_scale = self._get_gripper_action_scale()
        self.observation_joint_names = self._get_observation_joint_names()
        self.arm_action_joint_names = self._get_arm_action_joint_names()
        self.gripper_action_joint_names = self._get_gripper_action_joint_names()
        self.default_joint_positions = self._get_default_joint_positions()

    def _get_policy_environment_data(self, key_path: str, default=None):
        if self.policy_environment is None:
            raise ValueError("YAML not loaded. Call `load_policy_yaml()` first.")
        keys = key_path.split('.')
        val = self.policy_environment
        try:
            for key in keys:
                val = val[key]
            return val
        except (KeyError, TypeError):
            return default

    def _get_arm_action_scale(self) -> float:
        """Get the action scale from the YAML configuration."""
        if self.policy_environment is None:
            raise ValueError("YAML not loaded. Call `load_policy_yaml()` first.")
        return self._get_policy_environment_data("actions.arm_action.scale", 0.5)

    def _get_gripper_action_scale(self) -> float:
        """Get the gripper action scale from the YAML configuration."""
        if self.policy_environment is None:
            raise ValueError("YAML not loaded. Call `load_policy_yaml()` first.")
        return self._get_policy_environment_data("actions.gripper_action.scale", 0.5)

    def _get_observation_joint_names(self) -> list[str]:
        """Get the joint names from the YAML configuration."""
        if self.policy_environment is None:
            raise ValueError("YAML not loaded. Call `load_policy_yaml()` first.")
        return self._get_policy_environment_data("observations.policy.joint_pos.params.asset_cfg.joint_names", [])

    def _get_arm_action_joint_names(self) -> list[str]:
        """Get the action joint names from the YAML configuration."""
        if self.policy_environment is None:
            raise ValueError("YAML not loaded. Call `load_policy_yaml()` first.")
        return self._get_policy_environment_data("actions.arm_action.joint_names", [])
    
    def _get_gripper_action_joint_names(self) -> list[str]:
        """Get the gripper action joint names from the YAML configuration."""
        if self.policy_environment is None:
            raise ValueError("YAML not loaded. Call `load_policy_yaml()` first.")
        return self._get_policy_environment_data("actions.gripper_action.joint_names", [])

    def _get_default_joint_positions(self) -> dict[str, float]:
        """Get the default joint positions from the YAML configuration for given joint names."""
        if self.policy_environment is None:
            raise ValueError("YAML not loaded. Call `load_policy_yaml()` first.")

        default_joint = self._get_policy_environment_data("scene.robot.init_state.joint_pos", None)
        if default_joint is None:
            raise ValueError("Default joint positions not found in YAML.")

        return {str(k): float(v) for k, v in default_joint.items()} 

    def _sample_reach_random_pose(self) -> np.ndarray:
        pos = np.random.uniform([0.25, -0.2, 0.3], [0.45, 0.2, 0.45])
        roll = np.random.uniform(-math.pi / 4, math.pi / 4)
        pitch = 0.0
        yaw = np.random.uniform(math.pi / 4, math.pi * 3 / 4)
        quat = Rotation.from_euler("zyx", [yaw, pitch, roll]).as_quat()  # [x, y, z, w]
        return np.concatenate([pos, [quat[3], quat[0], quat[1], quat[2]]])  # [x, y, z, qw, qx, qy, qz]

class RLControllerNode(Node, RLInferenceManager):
    def __init__(self, model_dir: str):
        Node.__init__(self, 'rl_controller_node')
        inference_fps = 1000  # Set the inference frequency

        RLInferenceManager.__init__(self, inference_fps=inference_fps)  # Init parent class

        # Validate and load policy
        valid, msg = self.validate_policy(model_dir)
        if not valid:
            self.get_logger().error(msg)
            rclpy.shutdown()
            return
        if not self.load_policy():
            self.get_logger().error('Failed to load policy.')
            rclpy.shutdown()
            return

        # ROS interfaces
        self.joint_state_subscriber = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )

        step_size = 1.0 / inference_fps  # 100Hz
        self.trajectory_time_from_start = 0.1
        self.joint_command_timer = self.create_timer(step_size, self.timer_callback)

        self.has_joint_data = False
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'rh_r1_joint']

        self.get_logger().info("RL Controller Node initialized.")

    def joint_state_callback(self, msg: JointState):
        """Store all joint states from the message as dictionaries."""
        self.current_joint_positions = []
        self.current_joint_velocities = []

        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0.0
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            self.current_joint_positions.append(pos)
            self.current_joint_velocities.append(vel)

        self.has_joint_data = True

    def create_trajectory_command(self, joint_positions: list, joint_names: list[str]) -> JointTrajectory:
        """
        Creates a JointTrajectory message from a list of joint positions.

        Args:
            joint_positions (list): List of joint positions
        
        Returns:
            JointTrajectory: ROS2 JointTrajectory message
        """
        point = JointTrajectoryPoint()

        if len(joint_positions) != len(joint_names):
            raise ValueError(f"Expected {len(joint_names)} joint positions, got {len(joint_positions)}.")

        point.positions = joint_positions
        
        point.time_from_start = Duration(
            sec=0,
            nanosec=int(self.trajectory_time_from_start * 1e9)
        )

        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = joint_names
        joint_trajectory.points.append(point)
        return joint_trajectory

    def timer_callback(self):
        if not self.has_joint_data:
            return

        # Use predict() from RLInferenceManager
        additional_params = {}
        additional_params['joint_velocity'] = self.current_joint_velocities
        additional_params['joint_names'] = self.joint_names
        action, action_joint_names = self.predict(
            images={},  # No image input for this task
            state=self.current_joint_positions,
            task_instruction="omy_reach",
            additional_params=additional_params
        )

        joint_trajectory_msg = self.create_trajectory_command(action, action_joint_names)
        self.joint_trajectory_publisher.publish(joint_trajectory_msg)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_dir", type=str, required=True)
    parsed_args, remaining_args = parser.parse_known_args(args)

    rclpy.init(args=remaining_args)
    node = RLControllerNode(model_dir=parsed_args.model_dir)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
