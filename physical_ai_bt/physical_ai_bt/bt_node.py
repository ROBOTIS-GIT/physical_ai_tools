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

import os
import rclpy
import time
from ament_index_python.packages import get_package_share_directory
from physical_ai_interfaces.msg import TaskStatus
from physical_ai_interfaces.srv import SendCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy
from physical_ai_bt.actions.base_action import BTNode, NodeStatus
from physical_ai_bt.bt_nodes_loader import XMLTreeLoader
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node




class BehaviorTreeNode(Node):
    """Generic ROS2 node for Behavior Tree execution."""

    def __init__(self):
        super().__init__('physical_ai_bt_node')

        # --- Inference trigger state ---
        self.inference_detected = False
        self.inference_start_time = None
        self.waiting_for_inference = True
        self.latest_status = None
        self._last_status_stamp = None
        self._last_status_checked_time = time.time()
        self._prev_inference_detected = False

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        # Subscribe to /task/status for monitoring AI Server inference state
        self.status_sub = self.create_subscription(
            TaskStatus,
            '/task/status',
            self._status_callback,
            qos_profile
        )

        # Service server to receive commands from Physical AI Manager
        self.command_service = self.create_service(
            SendCommand,
            '/task/command',
            self._command_callback
        )

        # Service client to relay commands to Physical AI Server
        self.ai_server_client = self.create_client(
            SendCommand,
            '/ai_server/task/command'
        )
        self.get_logger().info('Waiting for Physical AI Server...')
        while not self.ai_server_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Physical AI Server not available, waiting...')
        self.get_logger().info('Physical AI Server is ready')

        self.declare_parameter('robot_type', 'ffw_sg2_rev1')
        self.declare_parameter('tree_xml', 'ffw_test.xml')
        self.declare_parameter('tick_rate', 30.0)

        robot_type = self.get_parameter('robot_type').value
        tree_xml = self.get_parameter('tree_xml').value
        tick_rate = self.get_parameter('tick_rate').value

        self.robot_type = robot_type
        self.joint_names = self._load_joint_order(robot_type)
        self.topic_config = self._load_topic_config(robot_type)

        pkg_share = get_package_share_directory('physical_ai_bt')
        xml_path = os.path.join(pkg_share, 'trees', tree_xml)

        if not os.path.exists(xml_path):
            # Try relative path
            xml_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                'trees',
                tree_xml
            )

        # Load behavior tree from XML
        self.xml_loader = XMLTreeLoader(
            self,
            joint_names=self.joint_names,
            topic_config=self.topic_config
        )
        self.root = self.xml_loader.load_tree_from_file(xml_path)

        # Create timer for BT tick
        self.timer = self.create_timer(1.0 / tick_rate, self.tick_callback)

        self.get_logger().info('Behavior Tree Node initialized')
        self.get_logger().info(f'  Robot type: {robot_type}')
        self.get_logger().info(f'  Joint names: {self.joint_names}')
        self.get_logger().info(f'  Tree XML: {tree_xml}')
        self.get_logger().info(f'  Tree: {self.root.name}')
        self.get_logger().info(f'  Tick rate: {tick_rate} Hz')

    def _status_callback(self, msg):
        """
        Subscribe to /task/status for monitoring purposes.

        Note: This callback no longer triggers BT execution.
        BT is now triggered via /task/command service from Physical AI Manager.
        """
        self.latest_status = msg
        # Save last status timestamp for monitoring
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            try:
                self._last_status_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            except Exception:
                self._last_status_stamp = time.time()
        else:
            self._last_status_stamp = time.time()
        self._last_status_checked_time = time.time()

        # OLD TRIGGER LOGIC REMOVED:
        # BT is now triggered via _command_callback, not by /task/status messages

    def _load_joint_order(self, robot_type: str) -> list:
        """Load joint order from ROS2 parameters (config file)."""
        # Load from bt_node namespace (from bt_node_params.yaml)
        self.declare_parameter(f'{robot_type}.joint_list', [''])
        joint_list_param = self.get_parameter(f'{robot_type}.joint_list').value

        if not joint_list_param or joint_list_param == ['']:
            self.get_logger().warn(
                f'No joint_list found in config for {robot_type}, using default'
            )
            return []

        # Collect joint orders from all joint groups
        all_joint_order = []
        for joint_name in joint_list_param:
            # Load joint_order from bt_node namespace
            param_name = f'{robot_type}.joint_order.{joint_name}'
            self.declare_parameter(param_name, [''])
            joint_order = self.get_parameter(param_name).value

            if joint_order and joint_order != ['']:
                all_joint_order.extend(joint_order)
                self.get_logger().info(f'Loaded {len(joint_order)} joints from {joint_name}')

        if not all_joint_order:
            self.get_logger().error(f'No joint_order found for any joint group')
            return []

        self.get_logger().info(f'Total joints loaded: {len(all_joint_order)}')
        return all_joint_order

    def _load_topic_config(self, robot_type: str) -> dict:
        """
        Load topic configuration for multi-publisher support.

        Returns:
            dict: {
                'joint_list': ['leader_left', 'leader_right', ...],
                'joint_topic_list': ['leader_left:/topic1', 'leader_right:/topic2', ...],
                'topic_map': {
                    'leader_left': '/topic1',
                    'leader_right': '/topic2',
                    ...
                },
                'joint_order': {
                    'leader_left': ['arm_l_joint1', ...],
                    'leader_right': ['arm_r_joint1', ...],
                    ...
                }
            }
        """
        # joint_list is already declared in _load_joint_order, just get it
        joint_list = self.get_parameter(f'{robot_type}.joint_list').value

        # Load joint_topic_list
        self.declare_parameter(f'{robot_type}.joint_topic_list', [''])
        joint_topic_list = self.get_parameter(f'{robot_type}.joint_topic_list').value

        # Build topic_map: parse "joint_group:/topic" format
        topic_map = {}
        for topic_entry in joint_topic_list:
            if ':' in topic_entry:
                joint_group, topic = topic_entry.split(':', 1)
                topic_map[joint_group] = topic

        # Load joint_order for each joint group (already declared in _load_joint_order)
        joint_order = {}
        for joint_name in joint_list:
            param_name = f'{robot_type}.joint_order.{joint_name}'
            order = self.get_parameter(param_name).value
            if order and order != ['']:
                joint_order[joint_name] = order

        config = {
            'joint_list': joint_list,
            'joint_topic_list': joint_topic_list,
            'topic_map': topic_map,
            'joint_order': joint_order
        }

        self.get_logger().info(f'Loaded topic config for {len(topic_map)} joint groups')
        return config

    def _command_callback(self, request, response):
        """
        Handle commands from Physical AI Manager.

        This callback receives commands (e.g., START_INFERENCE) from the manager,
        uses them as BT triggers, and relays them to the Physical AI Server.
        """
        try:
            self.get_logger().info(f'Received command: {request.command}')

            # Check if this is START_INFERENCE command
            if request.command == SendCommand.Request.START_INFERENCE:
                self.get_logger().info('START_INFERENCE received - activating BT trigger')

                # Activate BT trigger
                self.inference_detected = True
                self.inference_start_time = time.time()
                self.waiting_for_inference = False

                # Relay command to Physical AI Server
                self.get_logger().info('Relaying command to Physical AI Server...')
                relay_request = SendCommand.Request()
                relay_request.command = request.command
                relay_request.task_info = request.task_info

                # Call AI Server service
                future = self.ai_server_client.call_async(relay_request)

                # Wait for response (with timeout)
                timeout = 5.0
                start_time = time.time()
                while not future.done():
                    if time.time() - start_time > timeout:
                        self.get_logger().error('Timeout waiting for AI Server response')
                        response.success = False
                        response.message = 'Timeout waiting for AI Server response'
                        return response
                    rclpy.spin_once(self, timeout_sec=0.1)

                # Get result from AI Server
                try:
                    ai_server_response = future.result()
                    if ai_server_response.success:
                        self.get_logger().info('Command successfully relayed to AI Server')
                        response.success = True
                        response.message = 'BT triggered and command relayed to AI Server'
                    else:
                        self.get_logger().error(f'AI Server rejected command: {ai_server_response.message}')
                        response.success = False
                        response.message = f'AI Server error: {ai_server_response.message}'
                        # Reset BT trigger if AI Server failed
                        self.inference_detected = False
                        self.waiting_for_inference = True
                except Exception as e:
                    self.get_logger().error(f'Error getting AI Server response: {str(e)}')
                    response.success = False
                    response.message = f'Error communicating with AI Server: {str(e)}'
                    # Reset BT trigger if relay failed
                    self.inference_detected = False
                    self.waiting_for_inference = True

            else:
                # For other commands, just relay without triggering BT
                self.get_logger().info(f'Relaying non-START_INFERENCE command: {request.command}')
                relay_request = SendCommand.Request()
                relay_request.command = request.command
                relay_request.task_info = request.task_info

                future = self.ai_server_client.call_async(relay_request)
                timeout = 5.0
                start_time = time.time()
                while not future.done():
                    if time.time() - start_time > timeout:
                        response.success = False
                        response.message = 'Timeout waiting for AI Server response'
                        return response
                    rclpy.spin_once(self, timeout_sec=0.1)

                ai_server_response = future.result()
                response.success = ai_server_response.success
                response.message = ai_server_response.message

        except Exception as e:
            self.get_logger().error(f'Error in command callback: {str(e)}')
            response.success = False
            response.message = f'Error in BT command callback: {str(e)}'

        return response

    def tick_callback(self):
        """Timer callback for BT tick execution."""
        # Safety check: Reset BT if status messages stop coming
        # (indicates inference stopped on AI Server side)
        prev = self.inference_detected
        status_timeout = 1.0
        now = time.time()
        msg_time = self._last_status_stamp
        if self.latest_status is not None and msg_time is not None:
            msg_age = now - msg_time
            if msg_age > status_timeout:
                self.get_logger().warn('Status message timeout - resetting BT')
                self.inference_detected = False
        else:
            if self.latest_status is None:
                self.inference_detected = False

        # Reset tree if inference was stopped
        if prev and not self.inference_detected:
            self._reset_tree()
        self._prev_inference_detected = self.inference_detected

        # Wait for inference trigger from service callback
        if self.waiting_for_inference:
            if not self.inference_detected:
                return  # Do not tick tree until trigger
            self.waiting_for_inference = False

        # Step 2: Tick the root node
        status = self.root.tick()

        # Handle tree completion
        if status == NodeStatus.SUCCESS:
            self.get_logger().info('Behavior Tree cycle completed successfully!')
            self.get_logger().info('Resetting tree for next inference...')
            self._reset_tree()
            self.inference_detected = False
            self.inference_start_time = None
            self.waiting_for_inference = True
        elif status == NodeStatus.FAILURE:
            self.get_logger().error('Behavior Tree execution failed')
            self.get_logger().info('Resetting tree to retry...')
            self._reset_tree()
            self.inference_detected = False
            self.inference_start_time = None
            self.waiting_for_inference = True

    def _reset_tree(self):
        """Reset the behavior tree for next execution cycle."""
        # Get XML path again
        tree_xml = self.get_parameter('tree_xml').value
        pkg_share = get_package_share_directory('physical_ai_bt')
        xml_path = os.path.join(pkg_share, 'trees', tree_xml)

        if not os.path.exists(xml_path):
            xml_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                'trees',
                tree_xml
            )

        # Reload tree from XML
        self.root = self.xml_loader.load_tree_from_file(xml_path)


def main(args=None):
    """Main entry point for the BT node."""
    rclpy.init(args=args)

    try:
        bt_node = BehaviorTreeNode()
        executor = MultiThreadedExecutor()
        executor.add_node(bt_node)

        bt_node.get_logger().info('Behavior Tree Node is running')
        executor.spin()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in BT node: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
