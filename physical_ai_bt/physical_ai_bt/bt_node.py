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

import rclpy
from physical_ai_bt.actions.base_action import BTNode, NodeStatus
from physical_ai_bt.tree_builder import TreeBuilder
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class BehaviorTreeNode(Node):
    """Generic ROS2 node for Behavior Tree execution."""

    def __init__(self):
        super().__init__('physical_ai_bt_node')

        # Declare parameters
        self.declare_parameter('robot_type', 'omx_f')
        self.declare_parameter('inference_timeout', 5.0)
        self.declare_parameter('rule_timeout', 15.0)
        self.declare_parameter('tick_rate', 10.0)  # Hz
        self.declare_parameter('current_positions', [0.0, -0.378752, -0.151795, 1.541355, 0.006874, 0.73828])
        self.declare_parameter('target_positions', [-1.5, -0.378752, -0.151795, 1.541355, 0.006874, 0.73828])

        # Get parameters
        robot_type = self.get_parameter('robot_type').value
        inference_timeout = self.get_parameter('inference_timeout').value
        rule_timeout = self.get_parameter('rule_timeout').value
        tick_rate = self.get_parameter('tick_rate').value
        current_positions = self.get_parameter('current_positions').value
        target_positions = self.get_parameter('target_positions').value

        # Load joint order from config
        joint_names = self._load_joint_order(robot_type)

        # Build behavior tree
        self.root: BTNode = TreeBuilder.build_robot_control_tree(
            node=self,
            inference_timeout=inference_timeout,
            rule_timeout=rule_timeout,
            joint_names=joint_names,
            current_positions=current_positions,
            target_positions=target_positions
        )

        self.tree_completed = False

        # Create timer for BT tick
        self.timer = self.create_timer(1.0 / tick_rate, self.tick_callback)

        self.get_logger().info('Behavior Tree Node initialized')
        self.get_logger().info(f'  Robot type: {robot_type}')
        self.get_logger().info(f'  Joint names: {joint_names}')
        self.get_logger().info(f'  Tree: {self.root.name}')
        self.get_logger().info(f'  Tick rate: {tick_rate} Hz')

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
        if self.tree_completed:
            return

        # Tick the root node
        status = self.root.tick()

        # Handle tree completion
        if status == NodeStatus.SUCCESS:
            self.get_logger().info('Behavior Tree completed successfully!')
            self.tree_completed = True
            self.timer.cancel()
        elif status == NodeStatus.FAILURE:
            self.get_logger().error('Behavior Tree execution failed')
            self.tree_completed = True
            self.timer.cancel()
        # RUNNING: continue ticking


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
