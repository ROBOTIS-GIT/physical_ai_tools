from typing import Optional

from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from trajectory_msgs.msg import JointTrajectory
import torch


class RobotDataSubscriber(Node):
    def __init__(self):
        super().__init__('robot_data_subscriber')

        self.declare_parameter(
            'joint_order_follower', [
                'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
                'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7',
                'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
                'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7',
                'neck_joint1', 'neck_joint2', 'linear_joint'
            ]
        )

        self.declare_parameter(
            'joint_order_leader_r_hand', [
                'right_thumb_1_joint', 'right_thumb_2_joint',
                'right_index_1_joint', 'right_middle_1_joint',
                'right_ring_1_joint', 'right_little_1_joint',
            ]
        )
        self.declare_parameter(
            'joint_order_leader_l_hand', [
                'left_thumb_1_joint', 'left_thumb_2_joint',
                'left_index_1_joint', 'left_middle_1_joint',
                'left_ring_1_joint', 'left_little_1_joint',
            ]
        )
        self.declare_parameter(
            'joint_order_leader_r_arm', [
                'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
                'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7',
            ]
        )
        self.declare_parameter(
            'joint_order_leader_l_arm', [
                'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
                'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7',
            ]
        )

        self.joint_order_follower_r_hand = self.get_parameter('joint_order_follower_r_hand').value
        self.joint_order_follower_l_hand = self.get_parameter('joint_order_follower_l_hand').value
        self.joint_order_follower = self.get_parameter('joint_order_follower').value

        self.joint_order_leader_r_hand = self.get_parameter('joint_order_leader_r_hand').value
        self.joint_order_leader_l_hand = self.get_parameter('joint_order_leader_l_hand').value
        self.joint_order_leader_r_arm = self.get_parameter('joint_order_leader_r_arm').value
        self.joint_order_leader_l_arm = self.get_parameter('joint_order_leader_l_arm').value

        self.follower_hand_right = Subscriber(self, Int32MultiArray, '/follower/right_hand_angles')
        self.follower_hand_left = Subscriber(self, Int32MultiArray, '/follower/left_hand_angles')
        self.follower = Subscriber(
            self,
            JointState,
            '/joint_states'
        )

        self.leader_hand_right = Subscriber(self, JointTrajectory, '/leader/joint_trajectory_right_hand/joint_trajectory')
        self.leader_hand_left = Subscriber(self, JointTrajectory, '/leader/joint_trajectory_left/joint_trajectory')
        self.leader_arm_right = Subscriber(
            self, JointTrajectory,
            '/leader/joint_trajectory_right/joint_trajectory'
        )
        self.leader_arm_left = Subscriber(
            self,
            JointTrajectory,
            '/leader/joint_trajectory_left/joint_trajectory'
        )
        self.leader_neck = Subscriber(self, JointTrajectory, '/leader/neck_controller/joint_trajectory')
        self.leader_linear = Subscriber(
            self,
            JointTrajectory,
            '/leader/body_controller/joint_trajectory'
        )

        self.sync = ApproximateTimeSynchronizer(
            [
                self.follower_hand_right,
                self.follower_hand_left,
                self.follower,
                self.leader_hand_right,
                self.leader_hand_left,
                self.leader_arm_right,
                self.leader_arm_left,
                self.leader_neck,
                self.leader_linear
            ],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self.synced_callback)
        
        self.latest_observation = None
        self.latest_action = None

    def synced_callback(
        self,
        follower_hand_right_msg: Int32MultiArray,
        follower_hand_left_msg: Int32MultiArray,
        follower_msg: JointState,
        leader_hand_right_msg: JointTrajectory,
        leader_hand_left_msg: JointTrajectory,
        leader_arm_right_msg: JointTrajectory,
        leader_arm_left_msg: JointTrajectory,
        leader_neck_msg: JointTrajectory,
        leader_linear_msg: JointTrajectory
    ):
        try:
            follower_pos_map = dict(zip(
                follower_msg.name,
                follower_msg.position
            ))
            ordered_follower = [
                follower_pos_map[name]
                for name in self.joint_order_follower
            ]
            obs_pos = (
                list(map(float, follower_hand_right_msg.data)) +
                list(map(float, follower_hand_left_msg.data)) +
                ordered_follower
            )
            obs_tensor = torch.tensor(obs_pos, dtype=torch.float32)

            r_hand_pos_map = dict(zip(leader_hand_right_msg.joint_names, leader_hand_right_msg.points[0].positions))
            ordered_leader_r_hand = [r_hand_pos_map[name] for name in self.joint_order_leader_r_hand]

            l_hand_pos_map = dict(zip(leader_hand_left_msg.joint_names, leader_hand_left_msg.points[0].positions))
            ordered_leader_l_hand = [l_hand_pos_map[name] for name in self.joint_order_leader_l_hand]

            r_arm_pos_map = dict(zip(leader_arm_right_msg.joint_names, leader_arm_right_msg.points[0].positions))
            ordered_leader_r_arm = [r_arm_pos_map[name] for name in self.joint_order_leader_r_arm]

            l_arm_pos_map = dict(zip(leader_arm_left_msg.joint_names, leader_arm_left_msg.points[0].positions))
            ordered_leader_l_arm = [l_arm_pos_map[name] for name in self.joint_order_leader_l_arm]

            act_pos = (
                ordered_leader_r_hand +
                ordered_leader_l_hand +
                ordered_leader_r_arm +
                ordered_leader_l_arm +
                list(leader_neck_msg.points[0].positions) +
                list(leader_linear_msg.points[0].positions)
            )
            act_tensor = torch.tensor(act_pos, dtype=torch.float32)

            self.latest_observation = {'observation.state': obs_tensor}
            self.latest_action = {'action': act_tensor}
            self.get_logger().info('Synced data received.')

        except KeyError as e:
            self.get_logger().error(f'Joint name missing in incoming message: {e}')

    def get_latest_data(self) -> Optional[tuple[dict, dict]]:
        if self.latest_observation is not None and self.latest_action is not None:
            return self.latest_observation, self.latest_action
        return None, None


def main(args=None):
    rclpy.init(args=args)
    node = RobotDataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from rclpy.time import Time

# from sensor_msgs.msg import JointState
# from message_filters import Subscriber, ApproximateTimeSynchronizer

# import numpy as np
# import torch
# from typing import Optional


# class RobotDataSubscriber(Node):
#     def __init__(self):
#         super().__init__('robot_data_subscriber')
        
#         self.declare_parameter('expected_joint_order_follower', ['joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint'])
#         self.declare_parameter('expected_joint_order_leader', ['joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint'])
#         self.expected_joint_order_follower = self.get_parameter('expected_joint_order_follower').value
#         self.expected_joint_order_leader = self.get_parameter('expected_joint_order_leader').value
#         self.latest_observation = None
#         self.latest_action = None

#         # Create message_filters.Subscriber for each topic
#         self.obs_sub = Subscriber(self, JointState, '/joint_states')
#         self.act_sub = Subscriber(self, JointState, '/leader/joint_states')  # TODO: Replace with actual topic like /leader/joint_states

#         # Synchronize the topics with approximate time
#         self.sync = ApproximateTimeSynchronizer(
#             [self.obs_sub, self.act_sub],
#             queue_size=10,
#             slop=0.05  # Allowable time difference in seconds
#         )
#         self.sync.registerCallback(self.synced_callback)

#     def synced_callback(self, obs_msg: JointState, act_msg: JointState):
#         obs_pos_map = dict(zip(obs_msg.name, obs_msg.position))
#         act_pos_map = dict(zip(act_msg.name, act_msg.position))

#         ordered_obs = [obs_pos_map[name] for name in self.expected_joint_order_follower]
#         ordered_act = [act_pos_map[name] for name in self.expected_joint_order_leader]
            
#         obs_tensor = torch.tensor(ordered_obs, dtype=torch.float32)
#         act_tensor = torch.tensor(ordered_act, dtype=torch.float32)

#         # Convert ROS time to float (seconds)
#         obs_stamp = Time.from_msg(obs_msg.header.stamp).nanoseconds / 1e9
#         act_stamp = Time.from_msg(act_msg.header.stamp).nanoseconds / 1e9

#         self.latest_observation = {
#             'observation.state': obs_tensor,
#             # 'timestamp': obs_stamp,
#         }
#         self.latest_action = {
#             'action': act_tensor,
#             # 'timestamps': act_stamp,
#         }

#         self.get_logger().info(f'Received observation: {obs_tensor} @ {obs_stamp:.3f}s')
#         self.get_logger().info(f'Received action: {act_tensor} @ {act_stamp:.3f}s')

#     def get_latest_data(self) -> Optional[tuple[dict, dict]]:
#         if self.latest_observation is not None and self.latest_action is not None:
#             return self.latest_observation, self.latest_action
#         return None, None


# def main(args=None):
#     rclpy.init(args=args)
#     node = RobotDataSubscriber()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main() 