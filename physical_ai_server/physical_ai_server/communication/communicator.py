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

from functools import partial
from typing import Any, Dict, Optional, Set, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from physical_ai_interfaces.msg import TaskStatus
from physical_ai_interfaces.srv import (
    GetImageTopicList
)
from physical_ai_server.communication.multi_subscriber import MultiSubscriber
from physical_ai_server.utils.parameter_utils import parse_topic_list_with_names
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, JointState
from trajectory_msgs.msg import JointTrajectory


class Communicator:

    # Define data source categories
    SOURCE_CAMERA = 'camera'
    SOURCE_FOLLOWER = 'follower'
    SOURCE_LEADER = 'leader'

    # Define operation modes
    MODE_COLLECTION = 'collection'  # Full data collection mode (images, follower, leader)
    MODE_INFERENCE = 'inference'    # Inference mode (images, follower only)

    PUB_QOS_SIZE = 100

    def __init__(
        self,
        node: Node,
        operation_mode: str,
        params: Dict[str, Any]
    ):
        self.node = node
        self.operation_mode = operation_mode
        self.params = params

        # Parse topic lists for more convenient access
        self.camera_topics = parse_topic_list_with_names(self.params['camera_topic_list'])
        self.joint_topics = parse_topic_list_with_names(self.params['joint_topic_list'])

        # Determine which sources to enable based on operation mode
        self.enabled_sources = self._get_enabled_sources_for_mode(self.operation_mode)

        # Initialize MultiSubscriber with enabled sources
        self.multi_subscriber = MultiSubscriber(self.node, self.enabled_sources)

        # Initialize joint publishers
        self.joint_publishers = {}

        # Log topic information
        node.get_logger().info(f'Parsed camera topics: {self.camera_topics}')
        node.get_logger().info(f'Parsed joint topics: {self.joint_topics}')

        self.camera_topic_msgs = {}
        self.follower_topic_msgs = {}
        self.leader_topic_msgs = {}

        self.init_subscribers()
        self.init_publishers()
        self.init_services()

    def _get_enabled_sources_for_mode(self, mode: str) -> Set[str]:
        enabled_sources = set()

        # Camera and follower are always needed
        enabled_sources.add(self.SOURCE_CAMERA)
        enabled_sources.add(self.SOURCE_FOLLOWER)

        # Leader is only needed in collection mode
        if mode == self.MODE_COLLECTION:
            enabled_sources.add(self.SOURCE_LEADER)

        self.node.get_logger().info(f'Enabled sources for {mode} mode: {enabled_sources}')
        return enabled_sources

    def init_subscribers(self):
        # Initialize camera subscribers if defined
        for name, topic in self.camera_topics.items():
            self.multi_subscriber.add_subscriber(
                category=self.SOURCE_CAMERA,
                name=name,
                topic=topic,
                msg_type=CompressedImage,
                callback=partial(self._camera_callback, name)
            )
            self.node.get_logger().info(f'Camera subscriber: {name} -> {topic}')

        # Initialize joint subscribers with appropriate message types and callbacks
        for name, topic in self.joint_topics.items():
            # Determine category and message type based on name patterns
            if 'follower' in name.lower():
                if 'mobile' in name.lower():
                    msg_type = Odometry
                else:
                    msg_type = JointState
                category = self.SOURCE_FOLLOWER
                callback = partial(self._follower_callback, name)
            elif 'leader' in name.lower():
                if 'mobile' in name.lower():
                    msg_type = Twist
                else:
                    msg_type = JointTrajectory
                category = self.SOURCE_LEADER
                callback = partial(self._leader_callback, name)
            else:
                # Log an error message if the topic name does not include 'follower' or 'leader'
                self.node.get_logger().error(
                    '[Error] Please include follower or leader in the topic name.'
                )
                continue  # Move to the next topic

            self.multi_subscriber.add_subscriber(
                category=category,
                name=name,
                topic=topic,
                msg_type=msg_type,
                callback=callback
            )
            self.node.get_logger().info(
                f'Joint subscriber: {name} -> {topic} ({msg_type.__name__})')

    def init_publishers(self):
        self.node.get_logger().info('Initializing joint publishers...')
        for name, topic_name in self.joint_topics.items():
            if 'leader' in name.lower():
                if 'mobile' in name.lower():
                    self.joint_publishers[name] = self.node.create_publisher(
                        Twist,
                        topic_name,
                        self.PUB_QOS_SIZE
                    )
                else:
                    self.joint_publishers[name] = self.node.create_publisher(
                        JointTrajectory,
                        topic_name,
                        self.PUB_QOS_SIZE
                    )
        self.node.get_logger().info('Initializing joint publishers... done')

        self.status_publisher = self.node.create_publisher(
            TaskStatus,
            '/task/status',
            self.PUB_QOS_SIZE
        )

    def init_services(self):
        self.image_topic_list_service = self.node.create_service(
            GetImageTopicList,
            '/image/get_available_list',
            self.get_image_topic_list_callback
        )

    def _camera_callback(self, name: str, msg: CompressedImage) -> None:
        self.camera_topic_msgs[name] = msg

    def _follower_callback(self, name: str, msg: JointState) -> None:
        self.follower_topic_msgs[name] = msg

    def _leader_callback(self, name: str, msg: JointTrajectory) -> None:
        self.leader_topic_msgs[name] = msg

    def get_latest_data(self) -> Optional[Tuple[Dict, Dict, Dict]]:
        if not (self.camera_topic_msgs or self.follower_topic_msgs or self.leader_topic_msgs):
            return None, None, None
        if self.operation_mode == self.MODE_COLLECTION:
            return self.camera_topic_msgs, self.follower_topic_msgs, self.leader_topic_msgs
        elif self.operation_mode == self.MODE_INFERENCE:
            return self.camera_topic_msgs, self.follower_topic_msgs, None

    def publish_action(self, joint_msg_datas: Dict[str, Any]):
        for name, joint_msg in joint_msg_datas.items():
            self.joint_publishers[name].publish(joint_msg)

    def publish_status(self, status: TaskStatus):
        self.status_publisher.publish(status)

    def get_image_topic_list_callback(self, request, response):
        camera_topic_list = []
        for topic_name in self.camera_topics.values():
            topic = topic_name
            if topic.endswith('/compressed'):
                topic = topic[:-11]
            camera_topic_list.append(topic)

        if len(camera_topic_list) == 0:
            self.node.get_logger().error('No image topics found')
            response.image_topic_list = []
            response.success = False
            response.message = 'Please check image topics in your robot configuration.'
            return response

        response.image_topic_list = camera_topic_list
        response.success = True
        response.message = 'Image topic list retrieved successfully'
        return response

    def get_publisher_msg_types(self):
        msg_types = {}
        for publisher_name, publisher in self.joint_publishers.items():
            msg_types[publisher_name] = publisher.msg_type
        return msg_types

    def cleanup(self):
        self.node.get_logger().info(
            'Cleaning up Communicator resources...')

        if hasattr(self, 'status_publisher'):
            self.node.destroy_publisher(self.status_publisher)
            self.status_publisher = None

        for _, publisher in self.joint_publishers.items():
            self.node.destroy_publisher(publisher)
        self.joint_publishers.clear()

        if hasattr(self, 'multi_subscriber'):
            self.multi_subscriber.cleanup()
            self.multi_subscriber = None

        if hasattr(self, 'image_topic_list_service'):
            self.node.destroy_service(self.image_topic_list_service)
            self.image_topic_list_service = None

        self.camera_topic_msgs.clear()
        self.follower_topic_msgs.clear()
        self.leader_topic_msgs.clear()

        self.node.get_logger().info('Communicator cleanup completed')
