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


from collections import defaultdict
from typing import Callable, Optional, Set, Type

import rclpy
from rclpy.node import Node
import rclpy.qos


class MultiSubscriber:

    def __init__(self, node: Node, enabled_sources: Optional[Set[str]] = None):
        self._node = node
        self._subscribers = defaultdict(dict)
        self._enabled_sources = enabled_sources

    def is_source_enabled(self, category: str) -> bool:
        if self._enabled_sources is None:  # All sources enabled
            return True
        return category in self._enabled_sources

    def set_enabled_sources(self, enabled_sources: Optional[Set[str]]) -> None:
        self._enabled_sources = enabled_sources

    def add_subscriber(
            self,
            category: str,
            name: str,
            topic: str,
            msg_type: Type,
            callback: Optional[Callable] = None,
            qos_profile: Optional[rclpy.qos.QoSProfile] = None) -> None:

        # Skip if this source category is disabled
        if not self.is_source_enabled(category):
            self._node.get_logger().debug(
                f'Skipping subscriber {category}/{name} as category is disabled'
            )
            return

        if qos_profile is None:
            qos_profile = rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST
            )

        if category in self._subscribers and name in self._subscribers[category]:
            self._node.get_logger().warn(
                f'Subscriber {category}/{name} already exists. Overwriting.'
            )

        # Create subscriber with callback
        self._subscribers[category][name] = self._node.create_subscription(
            msg_type,
            topic,
            callback,
            qos_profile=qos_profile
        )

        self._node.get_logger().info(f'Subscribed to {topic} as {category}/{name}')

    def cleanup(self):
        self._node.get_logger().info(
            'Cleaning up MultiSubscriber resources...')

        for category, subscribers in self._subscribers.items():
            for name, subscriber in subscribers.items():
                self._node.destroy_subscription(subscriber)
                self._node.get_logger().debug(
                    f'Destroyed subscriber {category}/{name}')

        self._subscribers.clear()

        self._node.get_logger().info(
            'MultiSubscriber cleanup completed')
