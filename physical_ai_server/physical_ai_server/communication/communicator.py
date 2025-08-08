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
# Author: Dongyun Kim, Seongwoo Kim, Kiwoong Park

from functools import partial
from typing import Any, Dict, Optional, Set, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from physical_ai_interfaces.msg import TaskStatus, TrainingStatus, BrowserItem
from physical_ai_interfaces.srv import (
    BrowseFile,
    GetImageTopicList
)
from physical_ai_server.communication.multi_subscriber import MultiSubscriber
from physical_ai_server.utils.parameter_utils import parse_topic_list_with_names
from physical_ai_server.utils.file_browse_utils import FileBrowseUtils
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy
)
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Empty
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
        self.file_browse_utils = FileBrowseUtils(
            max_workers=8,
            logger=self.node.get_logger())

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

        self.heartbeat_qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

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
            self.camera_topic_msgs[name] = None
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
                self.follower_topic_msgs[name] = None
            elif 'leader' in name.lower():
                if 'mobile' in name.lower():
                    msg_type = Twist
                else:
                    msg_type = JointTrajectory
                category = self.SOURCE_LEADER
                callback = partial(self._leader_callback, name)
                self.leader_topic_msgs[name] = None
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

        self.training_status_publisher = self.node.create_publisher(
            TrainingStatus,
            '/training/status',
            self.PUB_QOS_SIZE
        )

        self.heartbeat_publisher = self.node.create_publisher(
            Empty,
            'heartbeat',
            self.heartbeat_qos_profile)

    def init_services(self):
        self.image_topic_list_service = self.node.create_service(
            GetImageTopicList,
            '/image/get_available_list',
            self.get_image_topic_list_callback
        )

        self.file_browser_service = self.node.create_service(
            BrowseFile,
            '/browse_file',
            self.browse_file_callback
        )

    def _camera_callback(self, name: str, msg: CompressedImage) -> None:
        self.camera_topic_msgs[name] = msg

    def _follower_callback(self, name: str, msg: JointState) -> None:
        self.follower_topic_msgs[name] = msg

    def _leader_callback(self, name: str, msg: JointTrajectory) -> None:
        self.leader_topic_msgs[name] = msg

    def get_latest_data(self) -> Optional[Tuple[Dict, Dict, Dict]]:
        if any(msg is None for msg in self.camera_topic_msgs.values()):
            return None, None, None

        if any(msg is None for msg in self.follower_topic_msgs.values()):
            return self.camera_topic_msgs, None, None

        if self.operation_mode == self.MODE_COLLECTION:
            if any(msg is None for msg in self.leader_topic_msgs.values()):
                return self.camera_topic_msgs, self.follower_topic_msgs, None
            return self.camera_topic_msgs, self.follower_topic_msgs, self.leader_topic_msgs
        elif self.operation_mode == self.MODE_INFERENCE:
            return self.camera_topic_msgs, self.follower_topic_msgs, None
        else:
            raise NotImplementedError(
                f'Operation mode {self.operation_mode} is not supported')

    def clear_latest_data(self):
        for key in self.camera_topic_msgs.keys():
            self.camera_topic_msgs[key] = None
        for key in self.follower_topic_msgs.keys():
            self.follower_topic_msgs[key] = None
        for key in self.leader_topic_msgs.keys():
            self.leader_topic_msgs[key] = None
        self.node.get_logger().info('Cleared latest data from communicator')

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

    def browse_file_callback(self, request, response):
        try:
            if request.action == "get_path":
                result = self.file_browse_utils.handle_get_path_action(
                    request.current_path)
            elif request.action == "go_parent":
                # Check if target_files are provided for parallel file checking
                target_files = None
                if hasattr(request, 'target_files') and request.target_files:
                    target_files = set(request.target_files)

                if target_files:
                    # Use parallel file checking for go_parent
                    result = self.file_browse_utils.handle_go_parent_with_file_check(
                        request.current_path, target_files)
                else:
                    # Use standard go_parent (no target files or empty list)
                    result = self.file_browse_utils.handle_go_parent_action(
                        request.current_path)
            elif request.action == "browse":
                # Check if target_files are provided for parallel file checking
                target_files = None
                if hasattr(request, 'target_files') and request.target_files:
                    target_files = set(request.target_files)

                if target_files:
                    # Use parallel file checking
                    result = self.file_browse_utils.handle_browse_with_file_check(
                        request.current_path, request.target_name, target_files)
                else:
                    # Use standard browsing (no target files or empty list)
                    result = self.file_browse_utils.handle_browse_action(
                        request.current_path, request.target_name)
            else:
                result = {
                    'success': False,
                    'message': f"Unknown action: {request.action}",
                    'current_path': "",
                    'parent_path': "",
                    'selected_path': "",
                    'items': []
                }

            # Convert result dict to response object
            response.success = result['success']
            response.message = result['message']
            response.current_path = result['current_path']
            response.parent_path = result['parent_path']
            response.selected_path = result['selected_path']

            # Convert item dicts to BrowserItem objects
            response.items = []
            for item_dict in result['items']:
                item = BrowserItem()
                item.name = item_dict['name']
                item.full_path = item_dict['full_path']
                item.is_directory = item_dict['is_directory']
                item.size = item_dict['size']
                item.modified_time = item_dict['modified_time']
                # Set has_target_file field (default False for files)
                item.has_target_file = item_dict.get('has_target_file', False)
                response.items.append(item)

        except Exception as e:
            self.node.get_logger().error(f"Error in browse file handler: {str(e)}")
            response.success = False
            response.message = f"Error: {str(e)}"
            response.current_path = ""
            response.parent_path = ""
            response.selected_path = ""
            response.items = []

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

    def heartbeat_timer_callback(self):
        heartbeat_msg = Empty()
        self.heartbeat_publisher.publish(heartbeat_msg)

    def publish_training_status(self, status: TrainingStatus):
        self.training_status_publisher.publish(status)
