#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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

"""ROS 2 node for executing behavior trees."""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from physical_ai_interfaces.srv import LoadAndRunTree
from physical_ai_interfaces.srv import SendCommand
from std_msgs.msg import String
from std_srvs.srv import SetBool

from physical_ai_bt.blackboard import Blackboard  # noqa: I100
from physical_ai_bt.bt_core import NodeStatus  # noqa: I100
from physical_ai_bt.bt_nodes_loader import TreeLoader  # noqa: I100


class BehaviorTreeNode(Node):
    """ROS 2 node that loads and executes behavior trees."""

    def __init__(self):
        """Initialize the behavior tree node."""
        super().__init__('physical_ai_bt_node')

        self.blackboard = Blackboard()

        self.tree_execution_mode = 'stopped'

        self.declare_parameter('robot_type', 'ffw_sg2_rev1')
        self.declare_parameter('tick_rate', 30.0)

        robot_type = self.get_parameter('robot_type').value
        tick_rate = self.get_parameter('tick_rate').value

        self.robot_type = robot_type
        self.joint_names = self._load_joint_order(robot_type)
        self.topic_config = self._load_topic_config(robot_type)

        self.tree_loader = TreeLoader(
            self,
            joint_names=self.joint_names,
            topic_config=self.topic_config
        )

        self.root = None

        self.timer = self.create_timer(1.0 / tick_rate, self.tick_callback)

        # Service: start/stop BT execution
        self.set_running_srv = self.create_service(
            SetBool, '/bt/set_running', self._set_running_callback
        )

        # Service: load tree from XML string and start execution
        self.load_and_run_srv = self.create_service(
            LoadAndRunTree, '/bt/load_and_run', self._load_and_run_callback
        )

        # Service client: cleanup inference on BT stop
        self._cleanup_client = self.create_client(
            SendCommand, '/task/command'
        )

        # Publisher: BT execution status
        self._status_pub = self.create_publisher(String, '/bt/status', 10)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        # Publisher: currently active node names
        self._active_nodes_pub = self.create_publisher(
            String, '/bt/active_nodes', 10
        )

        self.get_logger().info('Behavior Tree Node initialized')
        self.get_logger().info(f'Robot type: {robot_type}')
        self.get_logger().info(f'Tick rate: {tick_rate} Hz')
        self.get_logger().info(
            'Waiting for tree via /bt/load_and_run'
        )

    def _load_joint_order(self, robot_type: str) -> list:
        """Load joint order configuration for the robot type."""
        self.declare_parameter(f'{robot_type}.joint_list', [''])
        joint_list_param = self.get_parameter(
            f'{robot_type}.joint_list'
        ).value

        if not joint_list_param or joint_list_param == ['']:
            self.get_logger().warn(
                f'No joint_list found in config for {robot_type}, '
                'using default'
            )
            return []

        all_joint_order = []
        for joint_name in joint_list_param:
            param_name = f'{robot_type}.joint_order.{joint_name}'
            self.declare_parameter(param_name, [''])
            joint_order = self.get_parameter(param_name).value

            if joint_order and joint_order != ['']:
                all_joint_order.extend(joint_order)
                num_joints = len(joint_order)
                self.get_logger().info(
                    f'Loaded {num_joints} joints from {joint_name}'
                )

        if not all_joint_order:
            self.get_logger().error(
                'No joint_order found for any joint group'
            )
            return []

        self.get_logger().info(f'Total joints loaded: {len(all_joint_order)}')
        return all_joint_order

    def _load_topic_config(self, robot_type: str) -> dict:
        """Load topic configuration for the robot type."""
        joint_list = self.get_parameter(f'{robot_type}.joint_list').value

        self.declare_parameter(f'{robot_type}.joint_topic_list', [''])
        joint_topic_list = self.get_parameter(
            f'{robot_type}.joint_topic_list'
        ).value

        topic_map = {}
        for topic_entry in joint_topic_list:
            if ':' in topic_entry:
                joint_group, topic = topic_entry.split(':', 1)
                topic_map[joint_group] = topic

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

        num_groups = len(topic_map)
        self.get_logger().info(
            f'Loaded topic config for {num_groups} joint groups'
        )
        return config

    def tick_callback(self):
        """Execute one tick of the behavior tree."""
        if self.root is None:
            return

        if self.tree_execution_mode == 'stopping':
            return

        if self.tree_execution_mode != 'running':
            return

        status = self.root.tick()

        # Publish active node IDs
        if status == NodeStatus.RUNNING:
            active_names = self.root.get_active_node_ids()
            msg = String()
            msg.data = ','.join(active_names)
            self._active_nodes_pub.publish(msg)
        else:
            msg = String()
            msg.data = ''
            self._active_nodes_pub.publish(msg)

        if status in [NodeStatus.SUCCESS, NodeStatus.FAILURE]:
            if status == NodeStatus.SUCCESS:
                status_name = 'successfully'
            else:
                status_name = 'with failure'
            self.get_logger().info(
                f'Behavior Tree completed {status_name}'
            )
            self._handle_tree_completion(status)

    def _set_running_callback(self, request, response):
        """Handle /bt/set_running service call."""
        if request.data:
            if self.root is None:
                response.success = False
                response.message = 'No tree loaded'
                return response
            self.tree_execution_mode = 'running'
            response.success = True
            response.message = 'BT started'
            self.get_logger().info('BT execution started via service')
        else:
            if self.tree_execution_mode == 'running':
                self.tree_execution_mode = 'stopped'
                if self.root is not None:
                    self.root.reset()
                self._send_cleanup_command()
                self.get_logger().info('BT execution stopped via service')
            response.success = True
            response.message = 'BT stopped'
        self._publish_status()
        return response

    def _load_and_run_callback(self, request, response):
        """Handle /bt/load_and_run: load XML tree and start execution."""
        try:
            # Stop current tree if running
            if self.tree_execution_mode == 'running':
                self.tree_execution_mode = 'stopped'
                if self.root is not None:
                    self.root.reset()
                self._send_cleanup_command()

            # Load new tree from XML string
            self.root = self.tree_loader.load_tree_from_string(
                request.tree_xml
            )

            # Start execution
            self.tree_execution_mode = 'running'
            self._publish_status()

            response.success = True
            response.message = f'Tree loaded and started: {self.root.name}'
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Failed to load tree: {str(e)}'
            self.get_logger().error(response.message)
            self.root = None
            self.tree_execution_mode = 'stopped'
            self._publish_status()
        return response

    def _publish_status(self):
        """Publish current BT execution status."""
        msg = String()
        msg.data = self.tree_execution_mode
        self._status_pub.publish(msg)

    def _send_cleanup_command(self):
        """Send STOP_INFERENCE to pause inference (model stays loaded)."""
        try:
            if not self._cleanup_client.service_is_ready():
                self.get_logger().warn(
                    'Cleanup: /task/command service not available, skipping'
                )
                return
            req = SendCommand.Request()
            req.command = SendCommand.Request.STOP_INFERENCE
            future = self._cleanup_client.call_async(req)
            future.add_done_callback(self._cleanup_done_callback)
            self.get_logger().info('Cleanup: STOP_INFERENCE sent to server')
        except Exception as e:
            self.get_logger().error(f'Cleanup command failed: {e}')

    def _cleanup_done_callback(self, future):
        """Handle cleanup service response."""
        try:
            response = future.result()
            if response and response.success:
                self.get_logger().info('Cleanup: server confirmed stop')
            else:
                msg = response.message if response else 'No response'
                self.get_logger().warn(f'Cleanup: server response: {msg}')
        except Exception as e:
            self.get_logger().error(f'Cleanup response error: {e}')

    def _handle_tree_completion(self, status: NodeStatus):
        """Handle the completion of a behavior tree execution."""
        if self.root is not None:
            self.root.reset()

        self.tree_execution_mode = 'stopped'
        self._publish_status()
        self.get_logger().info('Behavior tree completed')


def main(args=None):
    """Run the behavior tree node."""
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
