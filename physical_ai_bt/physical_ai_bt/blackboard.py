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
# Author: Seongwoo Kim

"""Behavior Tree Blackboard for shared state."""


class Blackboard:
    """Singleton blackboard for sharing data between BT nodes."""

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._data = {}
        return cls._instance

    def set(self, key: str, value):
        """Set a value in the blackboard."""
        self._data[key] = value

    def get(self, key: str, default=None):
        """Get a value from the blackboard."""
        return self._data.get(key, default)

    def has(self, key: str) -> bool:
        """Check if a key exists in the blackboard."""
        return key in self._data

    def clear(self):
        """Clear all blackboard data."""
        self._data.clear()
