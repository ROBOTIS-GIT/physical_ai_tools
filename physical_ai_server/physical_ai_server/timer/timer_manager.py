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

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup


class TimerManager:

    def __init__(self, node: Node):
        self._node = node
        self._timer = {}
        self._timer_frequency = {}
        self._timer_callback = {}
        self._callback_groups = {}

    def start(self, timer_name):
        if self._timer[timer_name] is None:
            callback_group = self._callback_groups.get(timer_name, None)
            self._timer[timer_name] = self._node.create_timer(
                1.0/self._timer_frequency[timer_name],
                self._timer_callback[timer_name],
                callback_group=callback_group)

    def stop(self, timer_name):
        if self._timer[timer_name] is not None:
            self._timer[timer_name].destroy()
            self._timer[timer_name] = None

    def stop_all(self):
        for timer_name in self._timer:
            self.stop(timer_name)

    def set_timer(
            self,
            timer_name,
            timer_frequency,
            callback_function,
            use_separate_thread=False):
        self._timer[timer_name] = None
        self._timer_frequency[timer_name] = timer_frequency
        self._timer_callback[timer_name] = callback_function
        if use_separate_thread:
            self._callback_groups[timer_name] = ReentrantCallbackGroup()
        else:
            self._callback_groups[timer_name] = None
