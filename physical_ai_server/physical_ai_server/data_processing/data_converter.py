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

from typing import Any, Dict, List

import cv2
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import CompressedImage, JointState
import torch
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class DataConverter:

    def __init__(self):
        self._image_converter = CvBridge()  # Image converter using CVBridge
        self._joint_converter = None  # Joint data converter

    def compressed_image2cvmat(
            self,
            msg: CompressedImage,
            desired_encoding: str = 'passthrough') -> np.ndarray:
        try:
            cv_image = self._image_converter.compressed_imgmsg_to_cv2(
                    msg,
                    desired_encoding=desired_encoding)
            if cv_image is None:
                raise RuntimeError('cv_bridge returned None')
            if cv_image.dtype == np.uint16:
                cv_image = cv2.normalize(
                        cv_image,
                        None,
                        0,
                        255,
                        cv2.NORM_MINMAX,
                        dtype=cv2.CV_8U)
            return cv_image
        except Exception as e:
            raise RuntimeError(f'Failed to convert compressed image: {str(e)}')

    def joint_trajectory2tensor_array(
            self,
            msg: JointTrajectory,
            joint_order: List[str],
            target_format: str = 'numpy') -> Any:

        try:
            joint_pos_map = dict(zip(
                msg.joint_names,
                msg.points[0].positions
            ))

            ordered_positions = [
                joint_pos_map[name]
                for name in joint_order
            ]
            if target_format == 'numpy':
                return np.array(ordered_positions, dtype=np.float32)
            elif target_format == 'torch':
                return torch.tensor(ordered_positions, dtype=torch.float32)
            else:
                raise ValueError(f'Unsupported target format: {target_format}')
        except Exception as e:
            raise RuntimeError(f'Failed to convert joint trajectory: {str(e)}')

    def joint_state2tensor_array(
            self,
            msg: JointState,
            joint_order: List[str],
            target_format: str = 'numpy') -> Any:

        try:
            joint_pos_map = dict(zip(
                msg.name,
                msg.position
            ))
            ordered_positions = [
                joint_pos_map[name]
                for name in joint_order
            ]
            if target_format == 'numpy':
                return np.array(ordered_positions, dtype=np.float32)
            elif target_format == 'torch':
                return torch.tensor(ordered_positions, dtype=torch.float32)
            else:
                raise ValueError(f'Unsupported target format: {target_format}')
        except Exception as e:
            raise RuntimeError(f'Failed to convert joint state: {str(e)}')

    def tensor_array2joint_trajectory(
            self,
            action,
            joint_order: Dict[str, Any]):

        mapped_action = {}
        follower_joint_list = [value for key, value in joint_order.items() if 'follower' in key][0]

        for order, name in enumerate(follower_joint_list):
            mapped_action[name] = action[order]

        joint_trajectory_msgs = {}
        for key, value in joint_order.items():
            if 'leader' in key:
                key = key.split('.')[-1]
                reorder_action = [mapped_action[joint] for joint in value]
                joint_msg = JointTrajectory()
                joint_msg.joint_names = value
                joint_msg.points.append(JointTrajectoryPoint(positions=reorder_action))
                joint_trajectory_msgs[key] = joint_msg

        return joint_trajectory_msgs
