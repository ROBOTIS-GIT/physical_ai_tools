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

from lerobot.lerobot.common.datasets.lerobot_dataset import LeRobotDataset


class DataSaver(LeRobotDataset):

    def __init__(self, robot, config):
        # Assume config is of type RecordControlConfig
        self.robot = robot
        self.config = config
        # TODO: Implement init function
        pass

    def record(self):
        # TODO: Implement record function
        pass
