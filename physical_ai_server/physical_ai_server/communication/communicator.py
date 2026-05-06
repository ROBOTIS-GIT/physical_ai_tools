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

from typing import Any, Callable, Dict, List, Optional
import threading

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from physical_ai_interfaces.msg import (
    BrowserItem,
    DatasetInfo,
    TaskStatus
)
from physical_ai_interfaces.srv import (
    BrowseFile,
    EditDataset,
    GetDatasetInfo,
    GetImageTopicList,
)
from physical_ai_server.data_processing.data_editor import DataEditor
from physical_ai_server.utils.file_browse_utils import FileBrowseUtils
from physical_ai_server.utils.parameter_utils import (
    parse_topic_list,
    parse_topic_list_with_names,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy
)
from rosbag_recorder.srv import SendCommand
from std_msgs.msg import Empty, String
from trajectory_msgs.msg import JointTrajectory


class Communicator:
    """
    Communicator class for rosbag2-only data acquisition.

    This class manages:
    - Rosbag2 recording (start/stop/save)
    - Status publishing
    - Joystick trigger for control
    - Services for file browsing and dataset editing

    Note: Camera/Joint data subscription removed - rosbag2 records directly from topics.
    """

    # Define operation modes
    MODE_COLLECTION = 'collection'
    MODE_INFERENCE = 'inference'

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

        # Parse topic lists for rosbag recording
        self.camera_topics = parse_topic_list_with_names(self.params['camera_topic_list'])
        self.joint_topics = parse_topic_list_with_names(self.params['joint_topic_list'])
        self.rosbag_extra_topics = parse_topic_list(
            self.params['rosbag_extra_topic_list']
        )

        # Parse command topics for inference action publishing
        self.command_topics = parse_topic_list_with_names(
            self.params.get('command_topic_list', [])
        )

        # Initialize DataEditor for dataset editing
        self.data_editor = DataEditor()

        # Initialize joint publishers (for inference mode)
        self.joint_publishers = {}
        # Mirror JointState publishers on leader joint_states topics so that
        # rosbag can record the model's commands during inference (when the
        # human-leader hardware isn't publishing those topics).
        self.state_mirror_publishers = {}

        # Log topic information
        node.get_logger().info(f'Parsed camera topics: {self.camera_topics}')
        node.get_logger().info(f'Parsed joint topics: {self.joint_topics}')
        node.get_logger().info(f'Parsed rosbag extra topics: {self.rosbag_extra_topics}')

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

        # Joystick handler callback for immediate processing
        self._joystick_handler: Optional[Callable[[str], None]] = None

    def get_all_topics(self):
        """Get all topics for rosbag recording."""
        result = []
        for name, topic in self.camera_topics.items():
            result.append(topic)
        for name, topic in self.joint_topics.items():
            result.append(topic)
        result.extend(self.rosbag_extra_topics)
        return result

    def init_subscribers(self):
        """Initialize only joystick trigger subscriber."""
        # Joystick trigger for control (keep this)
        self.joystick_trigger_subscriber = self.node.create_subscription(
            String,
            '/leader/joystick_controller/tact_trigger',
            self.joystick_trigger_callback,
            10
        )
        self.node.get_logger().info('Joystick trigger subscriber initialized')

    def init_publishers(self):
        """Initialize publishers."""
        self.node.get_logger().info('Initializing publishers...')

        # Command publishers for inference mode (from command_topic_list)
        for name, topic_name in self.command_topics.items():
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

        # Mirror JointState publishers — same positions, broadcast on the
        # leader joint_states topic so rosbag (which already subscribes
        # there) captures the model's commanded state during inference.
        # For 'mobile', the command topic and state topic are the same
        # (/cmd_vel), so no mirror is needed — skip it.
        for name, state_topic in self.joint_topics.items():
            if 'mobile' in name.lower():
                continue
            if name not in self.joint_publishers:
                continue  # no command publisher → nothing to mirror
            self.state_mirror_publishers[name] = self.node.create_publisher(
                JointState,
                state_topic,
                self.PUB_QOS_SIZE
            )

        # Status publisher
        self.status_publisher = self.node.create_publisher(
            TaskStatus,
            '/task/status',
            self.PUB_QOS_SIZE
        )

        # Action event publisher (for voice feedback: start/finish/cancel)
        self.action_event_publisher = self.node.create_publisher(
            String,
            '/task/action_event',
            self.PUB_QOS_SIZE
        )

        # Action chunk preview publisher (full chunk for 3D visualization)
        self.action_chunk_publisher = self.node.create_publisher(
            JointTrajectory,
            '/inference/trajectory_preview',
            self.PUB_QOS_SIZE
        )

        # Heartbeat publisher
        self.heartbeat_publisher = self.node.create_publisher(
            Empty,
            'heartbeat',
            self.heartbeat_qos_profile)

        self.node.get_logger().info('Publishers initialized')

    def init_services(self):
        """Initialize services."""
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
            'rosbag_recorder/send_command',
            callback_group=ReentrantCallbackGroup())

        if self._check_rosbag_services_available():
            self.rosbag_service_available = True
            self.node.get_logger().info('Rosbag service is available')
        else:
            self.node.get_logger().error('Failed to connect to rosbag service')
            self.rosbag_service_available = False

    def _check_rosbag_services_available(self):
        return self._rosbag_send_command_client.wait_for_service(timeout_sec=3.0)

    # ========== Rosbag Management ==========

    def prepare_rosbag(self, topics: List[str]):
        self._send_rosbag_command(
            command=SendCommand.Request.PREPARE,
            topics=topics
        )

    def start_rosbag(self, rosbag_uri: str):
        self._send_rosbag_command(
            command=SendCommand.Request.START,
            uri=rosbag_uri,
            wait_for_response=True,
            timeout_sec=10.0,
        )

    def stop_rosbag(self):
        self._send_rosbag_command(
            command=SendCommand.Request.STOP,
            wait_for_response=True,
            timeout_sec=10.0,
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
                             uri: str = None,
                             wait_for_response: bool = False,
                             timeout_sec: float = 5.0):

        if not self.rosbag_service_available:
            self.node.get_logger().error('Rosbag service is not available')
            raise RuntimeError('Rosbag service is not available')

        req = SendCommand.Request()
        req.command = command
        req.topics = topics if topics is not None else []
        req.uri = uri if uri is not None else ''

        future = self._rosbag_send_command_client.call_async(req)

        if wait_for_response:
            done_event = threading.Event()
            future.add_done_callback(lambda _: done_event.set())

            if not done_event.wait(timeout=timeout_sec):
                raise TimeoutError(
                    f'Rosbag command timeout: command={command}, timeout={timeout_sec}s')

            result = future.result()
            if result is None:
                raise RuntimeError(f'Rosbag command returned no response: command={command}')
            if not result.success:
                raise RuntimeError(
                    f'Rosbag command failed: command={command}, message={result.message}')

            self.node.get_logger().info(
                f'Rosbag command completed: command={command}, message={result.message}')
            return

        # Asynchronous service call - fire and forget
        future.add_done_callback(
            lambda f: self.node.get_logger().info(
                f'Sent rosbag record command: {command} {f.result().message}'
                if f.done() and f.result().success
                else 'Failed to send command: '
                     f'{command} {f.result().message if f.done() else "timeout"}'
            )
        )

    # ========== Publishers ==========

    def publish_action(self, joint_msg_datas: Dict[str, Any]):
        """Publish joint commands (for inference mode).

        Also mirrors JointTrajectory positions as a JointState on the
        corresponding leader joint_states topic so rosbag captures the
        model's commands while recording during inference.
        """
        now = self.node.get_clock().now().to_msg()
        for name, joint_msg in joint_msg_datas.items():
            self.joint_publishers[name].publish(joint_msg)

            # Mirror only JointTrajectory (Twist on /cmd_vel is already on
            # the recorded topic, no mirror needed).
            mirror_pub = self.state_mirror_publishers.get(name)
            if mirror_pub is None or not isinstance(joint_msg, JointTrajectory):
                continue
            state_msg = JointState()
            state_msg.header.stamp = now
            state_msg.name = list(joint_msg.joint_names)
            if joint_msg.points:
                state_msg.position = [float(p) for p in joint_msg.points[0].positions]
            mirror_pub.publish(state_msg)

    def publish_action_chunk(self, msg: JointTrajectory):
        """Publish full action chunk for 3D trajectory visualization."""
        self.action_chunk_publisher.publish(msg)

    def publish_status(self, status: TaskStatus):
        """Publish task status."""
        self.status_publisher.publish(status)

    def publish_action_event(self, event: str):
        """Publish action event for voice feedback (start/finish/cancel)."""
        msg = String()
        msg.data = event
        self.action_event_publisher.publish(msg)

    def get_publisher_msg_types(self):
        """Get message types for joint publishers."""
        msg_types = {}
        for publisher_name, publisher in self.joint_publishers.items():
            msg_types[publisher_name] = publisher.msg_type
        return msg_types

    # ========== Joystick Handler ==========

    def register_joystick_handler(self, handler: Callable[[str], None]):
        """
        Register a handler for joystick triggers.

        This allows immediate processing of joystick events without
        waiting for the timer callback.

        Args:
            handler: Callback function that takes joystick mode string
        """
        self._joystick_handler = handler
        self.node.get_logger().info('Joystick handler registered')

    # ========== Callbacks ==========

    def joystick_trigger_callback(self, msg: String):
        """Handle joystick trigger for recording control."""
        self.node.get_logger().info(f'Received joystick trigger: {msg.data}')
        self.joystick_state['updated'] = True
        self.joystick_state['mode'] = msg.data

        # Call registered handler immediately if available
        if self._joystick_handler is not None:
            self._joystick_handler(msg.data)
            # Mark as processed to prevent duplicate handling in timer callback
            self.joystick_state['updated'] = False

    def heartbeat_timer_callback(self):
        """Publish heartbeat."""
        heartbeat_msg = Empty()
        self.heartbeat_publisher.publish(heartbeat_msg)

    # ========== Service Callbacks ==========

    def get_image_topic_list_callback(self, request, response):
        camera_topic_list = []
        for topic_name in self.camera_topics.values():
            # Return full topic path (e.g. .../image_raw/compressed) so the viewer uses the correct topic
            camera_topic_list.append(topic_name)

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
                target_files = None
                target_folders = None

                if hasattr(request, 'target_files') and request.target_files:
                    target_files = set(request.target_files)
                if hasattr(request, 'target_folders') and request.target_folders:
                    target_folders = set(request.target_folders)

                if target_files or target_folders:
                    result = self.file_browse_utils.handle_go_parent_with_target_check(
                        request.current_path,
                        target_files,
                        target_folders)
                else:
                    result = self.file_browse_utils.handle_go_parent_action(
                        request.current_path)
            elif request.action == 'browse':
                target_files = None
                target_folders = None

                if hasattr(request, 'target_files') and request.target_files:
                    target_files = set(request.target_files)
                if hasattr(request, 'target_folders') and request.target_folders:
                    target_folders = set(request.target_folders)

                if target_files or target_folders:
                    result = self.file_browse_utils.handle_browse_with_target_check(
                        request.current_path,
                        request.target_name,
                        target_files,
                        target_folders)
                else:
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

            response.success = result['success']
            response.message = result['message']
            response.current_path = result['current_path']
            response.parent_path = result['parent_path']
            response.selected_path = result['selected_path']

            response.items = []
            for item_dict in result['items']:
                item = BrowserItem()
                item.name = item_dict['name']
                item.full_path = item_dict['full_path']
                item.is_directory = item_dict['is_directory']
                item.size = item_dict['size']
                item.modified_time = item_dict['modified_time']
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
        from pathlib import Path
        try:
            if request.mode == EditDataset.Request.MERGE:
                source_dirs = [Path(p) for p in request.merge_source_task_dirs]
                output_dir = Path(request.merge_output_task_dir)
                result = self.data_editor.merge_rosbag_task_folders(
                    source_dirs,
                    output_dir,
                    move=bool(request.merge_move_sources),
                )
                response.affected_count = int(result.total_episodes)
                response.message = (
                    f'Merged {result.total_episodes} episodes into '
                    f'{result.output_dir} (mode='
                    f'{"move" if result.moved else "copy"})'
                )

            elif request.mode == EditDataset.Request.DELETE:
                task_dir = Path(request.delete_task_dir)
                indices = [int(i) for i in request.delete_episode_num]
                result = self.data_editor.delete_rosbag_episodes(
                    task_dir,
                    indices,
                    compact=bool(request.delete_compact),
                )
                response.affected_count = int(result.deleted_count)
                response.message = (
                    f'Deleted {result.deleted_count} episodes from '
                    f'{result.task_dir} (compact={result.compacted}, '
                    f'remaining={result.remaining_count})'
                )

            else:
                response.success = False
                response.affected_count = 0
                response.message = f'Unknown edit mode: {request.mode}'
                return response

            response.success = True
            return response

        except Exception as e:
            self.node.get_logger().error(f'Error in dataset_edit_callback: {str(e)}')
            response.success = False
            response.affected_count = 0
            response.message = f'Error: {str(e)}'
            return response

    def get_dataset_info_callback(self, request, response):
        from pathlib import Path
        try:
            task_dir = Path(request.dataset_path)
            task_info = self.data_editor.get_rosbag_task_info(task_dir)

            info = DatasetInfo()
            info.robot_type = task_info.robot_type
            info.task_instruction = task_info.task_instruction
            info.episode_count = int(task_info.episode_count)
            info.total_duration_s = float(task_info.total_duration_s)
            info.fps = int(task_info.fps)

            response.dataset_info = info
            response.success = True
            response.message = 'Task info retrieved successfully'
            return response

        except Exception as e:
            self.node.get_logger().error(f'Error in get_dataset_info_callback: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
            response.dataset_info = DatasetInfo()
            return response

    # ========== Cleanup ==========

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

        self.node.get_logger().info('Communicator cleanup completed')

    def _cleanup_publishers(self):
        publisher_names = [
            'status_publisher',
            'action_event_publisher',
            'heartbeat_publisher'
        ]
        for publisher_name in publisher_names:
            self._destroy_publisher_if_exists(publisher_name)

        for _, publisher in self.joint_publishers.items():
            self.node.destroy_publisher(publisher)
        self.joint_publishers.clear()

    def _cleanup_subscribers(self):
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
