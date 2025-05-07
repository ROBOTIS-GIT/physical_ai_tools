#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
import numpy as np
import torch
from pathlib import Path
import time
from typing import Dict, Any

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.datasets.utils import DEFAULT_FEATURES

class JointStateRecorder(Node):
    def __init__(self):
        super().__init__('joint_state_recorder')

        # Parameters
        self.declare_parameter('repo_id', 'joint_state_dataset')
        self.declare_parameter('fps', 30)
        self.declare_parameter('root_dir', str(Path.home() / '.cache/huggingface/lerobot'))
        self.declare_parameter('task', 'joint_state_task')
        self.declare_parameter('joint_names', [
            'arm_r_joint1', 'r_rh_r1_joint', 'arm_r_joint3', 'arm_l_joint3',
            'l_rh_r1_joint', 'arm_l_joint6', 'neck_joint1', 'neck_joint2',
            'arm_l_joint7', 'arm_l_joint1', 'arm_l_joint2', 'arm_r_joint7',
            'arm_r_joint6', 'arm_r_joint5', 'arm_r_joint4', 'arm_l_joint5',
            'linear_joint', 'arm_r_joint2', 'arm_l_joint4'
        ])

        self.repo_id = self.get_parameter('repo_id').value
        self.fps = self.get_parameter('fps').value
        self.root_dir = Path(self.get_parameter('root_dir').value)
        self.task = self.get_parameter('task').value
        self.joint_names = self.get_parameter('joint_names').value
        self.num_joints = len(self.joint_names)

        self.get_logger().info(f"Initializing JointStateRecorder with parameters:")
        self.get_logger().info(f"  - repo_id: {self.repo_id}")
        self.get_logger().info(f"  - fps: {self.fps}")
        self.get_logger().info(f"  - root_dir: {self.root_dir}")
        self.get_logger().info(f"  - task: {self.task}")
        self.get_logger().info(f"  - num_joints: {self.num_joints}")
        self.get_logger().info(f"  - joint_names: {self.joint_names}")

        # Create dataset
        self.dataset = self._create_dataset()

        # Buffer for current episode
        self.episode_buffer = {}
        self.episode_start_time = None

        # Setup subscriber
        self._setup_subscriber()

        # Timer for recording at fixed fps
        self.timer = self.create_timer(1.0/self.fps, self._record_frame)

        # Debug counters
        self.frame_count = 0
        self.last_log_time = time.time()

    def _create_dataset(self) -> LeRobotDataset:
        """Create a new LeRobot dataset with appropriate features."""
        self.get_logger().info("Creating LeRobot dataset...")
        features = DEFAULT_FEATURES.copy()

        # Add state features
        features['observation.state'] = {
            'dtype': 'float32',
            'names': self.joint_names,
            'shape': (self.num_joints,)  # Number of joints
        }

        # Add action features - same as state
        features['action'] = {
            'dtype': 'float32',
            'names': self.joint_names,
            'shape': (self.num_joints,)  # Number of joints
        }

        # Add dummy image feature
        features['observation.images.camera'] = {
            'dtype': 'video',
            'names': ['channels', 'height', 'width'],
            'shape': (3, 480, 640)  # Standard camera resolution
        }

        dataset = LeRobotDataset.create(
            repo_id=self.repo_id,
            fps=self.fps,
            # root=self.root_dir,
            features=features,
            use_videos=True
        )
        self.get_logger().info("Dataset created successfully")
        return dataset

    def _setup_subscriber(self):
        """Setup ROS2 subscriber for joint states."""
        self.get_logger().info("Setting up joint states subscriber...")

        # State subscriber
        self.state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._state_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST
            )
        )
        self.get_logger().info("Subscribed to /joint_states")

        # Buffer for latest message
        self.latest_state_msg = None

    def _state_callback(self, msg):
        """Store latest state message."""
        self.latest_state_msg = msg
        # Log joint states periodically
        current_time = time.time()
        if current_time - self.last_log_time > 1.0:  # Log every second
            self.get_logger().info(f"Received joint states: {dict(zip(msg.name, msg.position))}")
            self.last_log_time = current_time

    def _record_frame(self):
        """Record a frame at fixed fps."""
        if self.latest_state_msg is None:
            self.get_logger().info("Skipping frame - missing joint states message")
            return

        # Convert ROS message to numpy array
        frame = {}

        # Convert state - ensure joint order matches dataset features
        state_positions = np.zeros(self.num_joints, dtype=np.float32)
        state_map = dict(zip(self.latest_state_msg.name, self.latest_state_msg.position))
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in state_map:
                state_positions[i] = state_map[joint_name]
            else:
                self.get_logger().warn(f"Joint {joint_name} not found in state message")
        frame['observation.state'] = state_positions

        frame['action'] = state_positions

        # Add dummy image
        frame['observation.images.camera'] = np.zeros((3, 480, 640), dtype=np.uint8)

        # Add task
        frame['task'] = self.task

        # Add frame to dataset
        try:
            self.dataset.add_frame(frame)
            self.frame_count += 1
            if self.frame_count % self.fps == 0:  # Log every second
                self.get_logger().info(f"Recorded {self.frame_count} frames")
        except Exception as e:
            self.get_logger().error(f"Failed to add frame: {str(e)}")

    def save_episode(self):
        """Save current episode to dataset."""
        try:
            if self.frame_count > 0:
                self.get_logger().info("Saving episode...")
                self.dataset.save_episode()
                self.frame_count = 0
                self.get_logger().info("Episode saved successfully")
            else:
                self.get_logger().info("No frames to save in current episode")
        except Exception as e:
            self.get_logger().error(f"Failed to save episode: {str(e)}")
            raise

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Received keyboard interrupt, saving episode...")
        try:
            node.save_episode()
        except Exception as e:
            node.get_logger().error(f"Failed to save episode on shutdown: {str(e)}")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            node.get_logger().error(f"Error during shutdown: {str(e)}")

if __name__ == '__main__':
    main()
