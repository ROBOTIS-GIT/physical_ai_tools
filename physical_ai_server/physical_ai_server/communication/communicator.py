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
from typing import Any, Dict, List, Optional, Set, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from physical_ai_interfaces.msg import (
    BrowserItem,
    DatasetInfo,
    TaskStatus
)
from physical_ai_interfaces.srv import (
    BrowseFile,
    EditDataset,
    GetDatasetInfo,
    GetImageTopicList
)
from physical_ai_server.communication.multi_subscriber import MultiSubscriber
from physical_ai_server.data_processing.data_editor import DataEditor
from physical_ai_server.utils.file_browse_utils import FileBrowseUtils
from physical_ai_server.utils.parameter_utils import (
    parse_topic_list,
    parse_topic_list_with_names,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy
)
from rosbag_recorder.srv import SendCommand
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Empty, String
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
        self.rosbag_extra_topics = parse_topic_list(
            self.params['rosbag_extra_topic_list']
        )

        # Determine which sources to enable based on operation mode
        self.enabled_sources = self._get_enabled_sources_for_mode(self.operation_mode)

        # Initialize MultiSubscriber with enabled sources
        self.multi_subscriber = MultiSubscriber(self.node, self.enabled_sources)

        # Initialize DataEditor for dataset editing
        self.data_editor = DataEditor()

        # Initialize joint publishers
        self.joint_publishers = {}

        # Log topic information
        node.get_logger().info(f'Parsed camera topics: {self.camera_topics}')
        node.get_logger().info(f'Parsed joint topics: {self.joint_topics}')
        node.get_logger().info(f'Parsed rosbag extra topics: {self.rosbag_extra_topics}')

        self.camera_topic_msgs = {}
        self.follower_topic_msgs = {}
        self.leader_topic_msgs = {}

        self.heartbeat_qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.rosbag_service_available = False

        self.init_subscribers()
        self.init_publishers()
        self.init_services()

        self.joystick_state = {
            'updated': False,
            'mode': None
        }

    def get_all_topics(self):
        result = []
        for name, topic in self.camera_topics.items():
            result.append(topic)
        for name, topic in self.joint_topics.items():
            result.append(topic)
        result.extend(self.rosbag_extra_topics)
        return result

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

        self.joystick_trigger_subscriber = self.node.create_subscription(
            String,
            '/leader/joystick_controller/tact_trigger',
            self.joystick_trigger_callback,
            10
        )

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

        self.data_editor_service = self.node.create_service(
            EditDataset,
            '/dataset/edit',
            self.dataset_edit_callback
        )

        self.get_dataset_info_service = self.node.create_service(
            GetDatasetInfo,
            '/dataset/get_info',
            self.get_dataset_info_callback
        )

        self._rosbag_send_command_client = self.node.create_client(
            SendCommand,
            'rosbag_recorder/send_command')

        if self._check_rosbag_services_available():
            self.rosbag_service_available = True
            self.node.get_logger().info('Rosbag service is available')
        else:
            self.node.get_logger().error('Failed to connect to rosbag service')
            self.rosbag_service_available = False

    def _check_rosbag_services_available(self):
        return self._rosbag_send_command_client.wait_for_service(timeout_sec=3.0)

    def prepare_rosbag(self, topics: List[str]):
        self._send_rosbag_command(
            command=SendCommand.Request.PREPARE,
            topics=topics
        )

    def start_rosbag(self, rosbag_uri: str):
        self._send_rosbag_command(
            command=SendCommand.Request.START,
            uri=rosbag_uri
        )

    def stop_rosbag(self):
        self._send_rosbag_command(
            command=SendCommand.Request.STOP
        )

    def stop_and_delete_rosbag(self):
        self._send_rosbag_command(
            command=SendCommand.Request.STOP_AND_DELETE
        )

    def finish_rosbag(self):
        self._send_rosbag_command(
            command=SendCommand.Request.FINISH
        )

    def _send_rosbag_command(self,
                             command: int,
                             topics: List[str] = None,
                             uri: str = None):

        if not self.rosbag_service_available:
            self.node.get_logger().error('Rosbag service is not available')
            raise RuntimeError('Rosbag service is not available')

        req = SendCommand.Request()
        req.command = command
        req.topics = topics if topics is not None else []
        req.uri = uri if uri is not None else ''

        # Asynchronous service call - fire and forget
        future = self._rosbag_send_command_client.call_async(req)
        future.add_done_callback(
            lambda f: self.node.get_logger().info(
                f'Sent rosbag record command: {command} {f.result().message}'
                if f.done() and f.result().success
                else 'Failed to send command: '
                     f'{command} {f.result().message if f.done() else "timeout"}'
            )
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
            if request.action == 'get_path':
                result = self.file_browse_utils.handle_get_path_action(
                    request.current_path)
            elif request.action == 'go_parent':
                # Check if target_files or target_folders are provided
                target_files = None
                target_folders = None

                if hasattr(request, 'target_files') and request.target_files:
                    target_files = set(request.target_files)
                if hasattr(request, 'target_folders') and request.target_folders:
                    target_folders = set(request.target_folders)

                if target_files or target_folders:
                    # Use parallel target checking for go_parent
                    result = self.file_browse_utils.handle_go_parent_with_target_check(
                        request.current_path,
                        target_files,
                        target_folders)
                else:
                    # Use standard go_parent (no targets specified)
                    result = self.file_browse_utils.handle_go_parent_action(
                        request.current_path)
            elif request.action == 'browse':
                # Check if target_files or target_folders are provided
                target_files = None
                target_folders = None

                if hasattr(request, 'target_files') and request.target_files:
                    target_files = set(request.target_files)
                if hasattr(request, 'target_folders') and request.target_folders:
                    target_folders = set(request.target_folders)

                if target_files or target_folders:
                    # Use parallel target checking
                    result = self.file_browse_utils.handle_browse_with_target_check(
                        request.current_path,
                        request.target_name,
                        target_files,
                        target_folders)
                else:
                    # Use standard browsing (no targets specified)
                    result = self.file_browse_utils.handle_browse_action(
                        request.current_path, request.target_name)
            else:
                result = {
                    'success': False,
                    'message': f'Unknown action: {request.action}',
                    'current_path': '',
                    'parent_path': '',
                    'selected_path': '',
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
            self.node.get_logger().error(f'Error in browse file handler: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
            response.current_path = ''
            response.parent_path = ''
            response.selected_path = ''
            response.items = []

        return response

    def dataset_edit_callback(self, request, response):
        try:
            if request.mode == EditDataset.Request.MERGE:
                merge_dataset_list = request.merge_dataset_list
                output_path = request.output_path
                # TODO: Implement HuggingFace upload functionality if needed
                # upload_huggingface = request.upload_huggingface
                self.data_editor.merge_datasets(
                    merge_dataset_list, output_path)

            elif request.mode == EditDataset.Request.DELETE:
                delete_dataset_path = request.delete_dataset_path
                delete_episode_num = list(request.delete_episode_num)
                # TODO: Implement HuggingFace upload functionality if needed
                # upload_huggingface = request.upload_huggingface

                # Use batch delete for better performance
                if len(delete_episode_num) > 1:
                    self.data_editor.delete_episodes_batch(
                        delete_dataset_path, delete_episode_num
                    )
                else:
                    # Single episode deletion
                    self.data_editor.delete_episode(
                        delete_dataset_path, delete_episode_num[0]
                    )
            else:
                response.success = False
                response.message = f'Unknown edit mode: {request.mode}'
                return response

            response.success = True
            response.message = f'Successfully processed edit mode: {request.mode}'
            return response

        except Exception as e:
            self.node.get_logger().error(f'Error in dataset_edit_callback: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'

        return response

    def get_dataset_info_callback(self, request, response):
        try:
            dataset_path = request.dataset_path
            dataset_info = self.data_editor.get_dataset_info(dataset_path)

            info = DatasetInfo()
            info.codebase_version = dataset_info.get('codebase_version', 'unknown') if isinstance(
                dataset_info.get('codebase_version'), str) else 'unknown'
            info.robot_type = dataset_info.get('robot_type', 'unknown') if isinstance(
                dataset_info.get('robot_type'), str) else 'unknown'
            info.total_episodes = dataset_info.get('total_episodes', 0) if isinstance(
                dataset_info.get('total_episodes'), int) else 0
            info.total_tasks = dataset_info.get('total_tasks', 0) if isinstance(
                dataset_info.get('total_tasks'), int) else 0
            info.fps = dataset_info.get('fps', 0) if isinstance(
                dataset_info.get('fps'), int) else 0

            response.dataset_info = info
            response.success = True
            response.message = 'Dataset info retrieved successfully'
            return response

        except Exception as e:
            self.node.get_logger().error(f'Error in get_dataset_info_callback: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
            response.dataset_info = DatasetInfo()
            return response

    def get_publisher_msg_types(self):
        msg_types = {}
        for publisher_name, publisher in self.joint_publishers.items():
            msg_types[publisher_name] = publisher.msg_type
        return msg_types

    def _destroy_service_if_exists(self, service_attr_name: str):
        if hasattr(self, service_attr_name):
            service = getattr(self, service_attr_name)
            if service is not None:
                self.node.destroy_service(service)
                setattr(self, service_attr_name, None)

    def _destroy_client_if_exists(self, client_attr_name: str):
        if hasattr(self, client_attr_name):
            client = getattr(self, client_attr_name)
            if client is not None:
                self.node.destroy_client(client)
                setattr(self, client_attr_name, None)

    def _destroy_publisher_if_exists(self, publisher_attr_name: str):
        if hasattr(self, publisher_attr_name):
            publisher = getattr(self, publisher_attr_name)
            if publisher is not None:
                self.node.destroy_publisher(publisher)
                setattr(self, publisher_attr_name, None)

    def cleanup(self):
        self.node.get_logger().info('Cleaning up Communicator resources...')

        self._cleanup_publishers()
        self._cleanup_subscribers()
        self._cleanup_services()

        # Clear message containers
        self.camera_topic_msgs.clear()
        self.follower_topic_msgs.clear()
        self.leader_topic_msgs.clear()

        self.node.get_logger().info('Communicator cleanup completed')

    def _cleanup_publishers(self):
        publisher_names = [
            'status_publisher',
            'heartbeat_publisher'
        ]
        for publisher_name in publisher_names:
            self._destroy_publisher_if_exists(publisher_name)

        # Clean up joint publishers
        for _, publisher in self.joint_publishers.items():
            self.node.destroy_publisher(publisher)
        self.joint_publishers.clear()

    def _cleanup_subscribers(self):
        # Clean up multi subscriber
        if hasattr(self, 'multi_subscriber') and self.multi_subscriber is not None:
            self.multi_subscriber.cleanup()
            self.multi_subscriber = None

        # Clean up joystick trigger subscriber
        if hasattr(self, 'joystick_trigger_subscriber') and \
           self.joystick_trigger_subscriber is not None:
            self.node.destroy_subscription(self.joystick_trigger_subscriber)
            self.joystick_trigger_subscriber = None

    def _cleanup_services(self):
        service_names = [
            'image_topic_list_service',
            'file_browser_service',
            'data_editor_service',
            'get_dataset_info_service'
        ]
        for service_name in service_names:
            self._destroy_service_if_exists(service_name)

    def _cleanup_clients(self):
        client_names = [
            '_rosbag_send_command_client'
        ]
        for client_name in client_names:
            self._destroy_client_if_exists(client_name)

    def heartbeat_timer_callback(self):
        heartbeat_msg = Empty()
        self.heartbeat_publisher.publish(heartbeat_msg)

    def joystick_trigger_callback(self, msg: String):
        self.node.get_logger().info(f'Received joystick trigger: {msg.data}')
        self.joystick_state['updated'] = True
        self.joystick_state['mode'] = msg.data
