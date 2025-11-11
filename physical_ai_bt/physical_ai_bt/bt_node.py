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
from ament_index_python.packages import get_package_share_directory
from physical_ai_bt.actions.base_action import BTNode, NodeStatus
from physical_ai_bt.xml_loader import XMLTreeLoader
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class BehaviorTreeNode(Node):
    """Generic ROS2 node for Behavior Tree execution."""

    def __init__(self):
        super().__init__('physical_ai_bt_node')

        # Declare parameters with use_sim_time to allow parameter server override
        self.declare_parameter('robot_type', 'omx_f')
        self.declare_parameter('tree_xml', 'robot_control_tree.xml')
        self.declare_parameter('tick_rate', 10.0)  # Hz

        # Declare BT behavior parameters
        self.declare_parameter('inference_timeout', 5.0)
        self.declare_parameter('rule_timeout', 15.0)
        self.declare_parameter('position_threshold', 0.1)
        self.declare_parameter('current_positions', [0.0, -0.378752, -0.151795, 1.541355, 0.006874, 0.73828])
        self.declare_parameter('target_positions', [-1.5, -0.378752, -0.151795, 1.541355, 0.006874, 0.73828])

        # Get parameters
        robot_type = self.get_parameter('robot_type').value
        tree_xml = self.get_parameter('tree_xml').value
        tick_rate = self.get_parameter('tick_rate').value

        # Try to get robot_type from global parameter server first
        try:
            global_robot_type = self.get_parameter_or(
                '/physical_ai_server/robot_type',
                rclpy.Parameter('robot_type', rclpy.Parameter.Type.STRING, robot_type)
            ).value
            if global_robot_type and global_robot_type != robot_type:
                self.get_logger().info(
                    f'Using global robot_type from parameter server: {global_robot_type}'
                )
                robot_type = global_robot_type
        except Exception as e:
            self.get_logger().debug(f'Could not read global robot_type: {e}')

        self.robot_type = robot_type

        # Load joint order from config
        self.joint_names = self._load_joint_order(robot_type)

        # Build runtime parameters for XML injection
        self.runtime_params = {
            'inference_timeout': self.get_parameter('inference_timeout').value,
            'rule_timeout': self.get_parameter('rule_timeout').value,
            'position_threshold': self.get_parameter('position_threshold').value,
            'current_positions': ','.join(map(str, self.get_parameter('current_positions').value)),
            'target_positions': ','.join(map(str, self.get_parameter('target_positions').value)),
        }

        # Get XML file path
        pkg_share = get_package_share_directory('physical_ai_bt')
        xml_path = os.path.join(pkg_share, 'trees', tree_xml)

        if not os.path.exists(xml_path):
            # Try relative path
            xml_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                'trees',
                tree_xml
            )

        # Load behavior tree from XML with runtime parameters
        self.xml_loader = XMLTreeLoader(
            self,
            joint_names=self.joint_names,
            runtime_params=self.runtime_params
        )
        self.root: BTNode = self.xml_loader.load_tree_from_file(xml_path)

        # Create timer for BT tick
        self.timer = self.create_timer(1.0 / tick_rate, self.tick_callback)

        self.get_logger().info('Behavior Tree Node initialized')
        self.get_logger().info(f'  Robot type: {robot_type}')
        self.get_logger().info(f'  Joint names: {self.joint_names}')
        self.get_logger().info(f'  Tree XML: {tree_xml}')
        self.get_logger().info(f'  Tree: {self.root.name}')
        self.get_logger().info(f'  Tick rate: {tick_rate} Hz')
        self.get_logger().info(f'  Runtime params: {self.runtime_params}')
        self.get_logger().info('  Mode: Continuous (restarts after each completion)')

    def _load_joint_order(self, robot_type: str) -> list:
        """Load joint order from ROS2 parameters (config file)."""
        # Declare joint_list parameter
        self.declare_parameter(f'{robot_type}.joint_list', [''])
        joint_list_param = self.get_parameter(f'{robot_type}.joint_list').value

        if not joint_list_param or joint_list_param == ['']:
            self.get_logger().warn(
                f'No joint_list found in config for {robot_type}, using default'
            )
            return ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_joint_1']

        # Get first joint name (e.g., 'leader')
        joint_name = joint_list_param[0]

        # Declare joint_order parameter
        param_name = f'{robot_type}.joint_order.{joint_name}'
        self.declare_parameter(param_name, [''])
        joint_order = self.get_parameter(param_name).value

        if not joint_order or joint_order == ['']:
            self.get_logger().error(f'No joint_order found for {joint_name}')
            return ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_joint_1']

        self.get_logger().info(f'Loaded joint order from config: {joint_order}')
        return joint_order

    def tick_callback(self):
        """Timer callback for BT tick execution."""
        # Tick the root node
        status = self.root.tick()

        # Handle tree completion
        if status == NodeStatus.SUCCESS:
            self.get_logger().info('Behavior Tree cycle completed successfully!')
            self.get_logger().info('Resetting tree for next inference...')
            self._reset_tree()
        elif status == NodeStatus.FAILURE:
            self.get_logger().error('Behavior Tree execution failed')
            self.get_logger().info('Resetting tree to retry...')
            self._reset_tree()
        # RUNNING: continue ticking

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
