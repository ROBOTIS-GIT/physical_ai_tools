#!/usr/bin/env python3
# Copyright 2025 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Author: Seongwoo Kim

"""Whole-body rule-based action for fixed position movement."""

import time
from typing import TYPE_CHECKING, List
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if TYPE_CHECKING:
    from rclpy.node import Node

class RuleWholeBody(BaseAction):
    """Whole-body action that moves robot to fixed target positions."""
    def __init__(
            self,
            node: 'Node',
            current_positions: List[float],
            target_positions: List[float],
            joint_names: List[str],
            position_threshold: float = 0.001,
            timeout: float = 15.0,
            topic_config: dict = None
        ):
        super().__init__(node, name="RuleWholeBody")
        self.current_positions = current_positions
        self.target_positions = target_positions
        self.joint_names = joint_names
        self.position_threshold = position_threshold
        self.timeout = timeout
        self.topic_config = topic_config or {}
        if not isinstance(self.topic_config, dict):
            self.topic_config = {}
        # QoS profile for publishers
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Create publishers based on topic_config
        self.publishers = {}
        if self.topic_config and 'topic_map' in self.topic_config:
            # Multi-topic mode: create publisher for each joint group
            for joint_group, topic in self.topic_config['topic_map'].items():
                if joint_group == 'leader_mobile':
                    # Mobile uses Twist message
                    self.publishers[joint_group] = self.node.create_publisher(
                        Twist,
                        topic,
                        qos_profile
                    )
                else:
                    # Joint groups use JointTrajectory
                    self.publishers[joint_group] = self.node.create_publisher(
                        JointTrajectory,
                        topic,
                        qos_profile
                    )
            self.log_info(f"Created {len(self.publishers)} publishers for multi-topic mode")
        else:
            # Single-topic mode: fallback for backward compatibility
            self.trajectory_pub = self.node.create_publisher(
                JointTrajectory,
                '/leader/joint_trajectory',
                qos_profile
            )
            self.log_info("Created single publisher (legacy mode)")

        # Subscriber for follower joint states (for monitoring only)
        # Try to create subscription, but continue if it fails
        try:
            subscriber_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            )
            self.joint_state_sub = self.node.create_subscription(
                JointState,
                '/joint_states',
                self._joint_state_callback,
                subscriber_qos
            )
            self.log_info("Subscribed to /joint_states for position monitoring")
        except Exception as e:
            self.log_warn(f"Could not subscribe to /joint_states: {e}")
            self.joint_state_sub = None

        # State variables
        self.command_sent = False
        self.start_time = None
        self.latest_joint_positions = None

    def _joint_state_callback(self, msg: JointState):
        """Callback for joint state messages."""
        # Reorder positions to match self.joint_names order
        if hasattr(msg, 'name') and msg.name:
            try:
                # Create mapping from received names to positions
                name_to_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
                # Reorder according to self.joint_names, skip mobile commands (linear_x, etc)
                self.latest_joint_positions = []
                for name in self.joint_names:
                    if name in name_to_pos:
                        self.latest_joint_positions.append(name_to_pos[name])
                    elif name in ['linear_x', 'linear_y', 'angular_z']:
                        # Mobile commands - use 0 as they're not in joint_states
                        self.latest_joint_positions.append(0.0)
                    else:
                        raise KeyError(name)
            except KeyError as e:
                if not hasattr(self, '_joint_error_logged'):
                    self._joint_error_logged = True
                    self.log_error(f"Joint name mismatch: {e}")
                self.latest_joint_positions = list(msg.position)
        else:
            self.latest_joint_positions = list(msg.position)

    def tick(self) -> NodeStatus:
        """Execute one tick of whole-body rule-based action."""
        current_time = time.time()

        # Step 1: Send trajectory command if not sent
        if not self.command_sent:
            self._send_trajectory_command()
            self.command_sent = True
            self.start_time = current_time
            return NodeStatus.RUNNING

        # Step 2: For mobile base, continue sending velocity commands
        elapsed = current_time - self.start_time
        if self._has_mobile_command() and elapsed < self.timeout:
            # Continue publishing mobile velocity commands every tick
            self._publish_mobile_command()

        # Step 3: Check if target reached or timeout
        # If joint state subscription failed, use time-based completion (2s trajectory time + 0.5s buffer)
        if self.joint_state_sub is None:
            if elapsed > self.timeout:
                # Stop mobile base before completing
                if self._has_mobile_command():
                    self._stop_mobile()
                self.log_info(f"Trajectory completed in {elapsed:.1f}s (no feedback monitoring)")
                return NodeStatus.SUCCESS
            return NodeStatus.RUNNING

        if elapsed > self.timeout:
            if self._has_mobile_command():
                self._stop_mobile()
            self.log_warn(f"Timeout ({self.timeout}s) reached")
            return NodeStatus.FAILURE

        if self._is_target_reached():
            if self._has_mobile_command():
                self._stop_mobile()
            self.log_info(f"Target reached in {elapsed:.1f}s")
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING

    def _send_trajectory_command(self):
        """Publish trajectory command with target positions, starting from latest joint_states if available."""
        # Use latest joint positions if available, else fallback to initial current_positions
        # Compose goal positions for all joint groups
        # Left arm
        left_arm_goal = [0.5954753044455301, -0.05522330830078125, -0.17947575197753907, 0.22714006458231562, 0.003069460332193454, 0.8517752421836835, 0.3084807633854421, -1.2033011704101564]
        # Right arm
        right_arm_goal = [0.5939405742794334, -0.05982525065917969, -0.20555342534179688, 0.49725257381533955, -0.15186409782714844, 0.7827123847093308, -0.5292233712158203, -1.1388739773925782]
        # Head (linear neck)
        head_goal = [-0.01793636702335067, -0.009748462565261374]
        # Lift
        lift_goal = [-0.020166984446496245]
        # Swerve (rotate in place)
        angular_z = (3.141592653589793 / 2) / self.timeout
        swerve_goal = [0.0, 0.0, angular_z]

        # Compose full target_positions in joint_list order
        joint_list = self.topic_config.get('joint_list', [])
        joint_order = self.topic_config.get('joint_order', {})
        full_goal = []
        for group in joint_list:
            if group == 'leader_left':
                full_goal.extend(left_arm_goal)
            elif group == 'leader_right':
                full_goal.extend(right_arm_goal)
            elif group == 'leader_head':
                full_goal.extend(head_goal)
            elif group == 'leader_lift':
                full_goal.extend(lift_goal)
            elif group == 'leader_mobile':
                full_goal.extend(swerve_goal)

        self.target_positions = full_goal
        start_positions = self.latest_joint_positions if self.latest_joint_positions is not None else self.current_positions
        if self.publishers:
            self._publish_multi_topic(start_positions)
        else:
            self._publish_single_topic(start_positions)

    def _publish_single_topic(self, start_positions):
        """Publish to single topic (legacy mode), using start_positions as initial pose."""
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp.sec = 0
        trajectory_msg.header.stamp.nanosec = 0
        trajectory_msg.joint_names = self.joint_names

        # Create trajectory points: start and target
        start_point = JointTrajectoryPoint()
        start_point.positions = start_positions
        start_point.time_from_start = Duration(sec=0, nanosec=0)

        target_point = JointTrajectoryPoint()
        target_point.positions = self.target_positions
        target_point.time_from_start = Duration(sec=0, nanosec=0)

        trajectory_msg.points = [start_point, target_point]

        self.trajectory_pub.publish(trajectory_msg)
        self.log_info("Published single trajectory command (start from latest pose)")

    def _publish_multi_topic(self, start_positions):
        """
        Publish to multiple topics based on joint groups.
        Splits target_positions and start_positions according to joint_order and publishes to corresponding topics.
        """
        joint_list = self.topic_config.get('joint_list', [])
        joint_order = self.topic_config.get('joint_order', {})

        # Split positions by joint groups
        position_offset = 0
        for joint_group in joint_list:
            if joint_group not in self.publishers:
                self.log_warn(f"No publisher for joint group: {joint_group}")
                continue

            group_joints = joint_order.get(joint_group, [])
            if not group_joints:
                self.log_warn(f"No joint_order for group: {joint_group}")
                continue
            group_size = len(group_joints)
            group_target_positions = self.target_positions[position_offset:position_offset + group_size]
            group_start_positions = start_positions[position_offset:position_offset + group_size]
            position_offset += group_size

            if joint_group == 'leader_mobile':
                # Mobile: publish Twist message (target only)
                twist_msg = Twist()
                twist_msg.linear.x = group_target_positions[0] if len(group_target_positions) > 0 else 0.0
                twist_msg.linear.y = group_target_positions[1] if len(group_target_positions) > 1 else 0.0
                twist_msg.angular.z = group_target_positions[2] if len(group_target_positions) > 2 else 0.0
                self.publishers[joint_group].publish(twist_msg)
                self.log_info(f"Published Twist to {joint_group}: angular_z={twist_msg.angular.z} (rotate 90deg in place)")
            else:
                # Joint group: publish JointTrajectory with only valid start point
                trajectory_msg = JointTrajectory()
                trajectory_msg.header.stamp.sec = 0
                trajectory_msg.header.stamp.nanosec = 0
                trajectory_msg.joint_names = group_joints
                points = []
                # Only add start point if valid
                if group_start_positions and len(group_start_positions) == len(group_joints):
                    start_point = JointTrajectoryPoint()
                    start_point.positions = group_start_positions
                    start_point.time_from_start = Duration(sec=0, nanosec=0)
                    points.append(start_point)
                # Always add goal point
                target_point = JointTrajectoryPoint()
                target_point.positions = group_target_positions
                target_point.time_from_start = Duration(sec=0, nanosec=0)
                points.append(target_point)
                trajectory_msg.points = points
                self.publishers[joint_group].publish(trajectory_msg)
                self.log_info(f"Published trajectory to {joint_group}: {len(group_joints)} joints (start/goal points: {len(points)})")

    def _has_mobile_command(self) -> bool:
        """Check if this action includes mobile base commands."""
        if not self.topic_config or 'joint_list' not in self.topic_config:
            return False
        return 'leader_mobile' in self.topic_config['joint_list']

    def _publish_mobile_command(self):
        """Publish mobile velocity command (called repeatedly during execution)."""
        if 'leader_mobile' not in self.publishers:
            return

        joint_list = self.topic_config.get('joint_list', [])
        joint_order = self.topic_config.get('joint_order', {})

        # Find mobile positions in target_positions
        position_offset = 0
        for joint_group in joint_list:
            group_joints = joint_order.get(joint_group, [])
            group_size = len(group_joints)

            if joint_group == 'leader_mobile':
                group_positions = self.target_positions[position_offset:position_offset + group_size]

                twist_msg = Twist()
                if len(group_positions) >= 3:
                    twist_msg.linear.x = group_positions[0]
                    twist_msg.linear.y = group_positions[1]
                    twist_msg.angular.z = group_positions[2]

                self.publishers[joint_group].publish(twist_msg)
                return

            position_offset += group_size

    def _stop_mobile(self):
        """Stop mobile base by publishing zero velocity."""
        if 'leader_mobile' not in self.publishers:
            return

        twist_msg = Twist()
        # All velocities are 0.0 by default
        self.publishers['leader_mobile'].publish(twist_msg)
        self.log_info("Mobile base stopped")

    def _is_target_reached(self) -> bool:
        """Check if robot reached target position."""
        if self.latest_joint_positions is None:
            return False

        if len(self.latest_joint_positions) != len(self.target_positions):
            if not hasattr(self, '_mismatch_logged'):
                self._mismatch_logged = True
                self.log_warn(
                    f"Joint count mismatch: got {len(self.latest_joint_positions)}, "
                    f"expected {len(self.target_positions)}"
                )
            return False

        # Check if all joints are within threshold
        for i, (current, target) in enumerate(zip(self.latest_joint_positions, self.target_positions)):
            diff = abs(current - target)
            if diff > self.position_threshold:
                return False

        return True

    def reset(self):
        """Reset action state for re-execution."""
        super().reset()
        self.command_sent = False
        self.start_time = None
        self.latest_joint_positions = None
