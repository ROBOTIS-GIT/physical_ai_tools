import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState
from message_filters import Subscriber, ApproximateTimeSynchronizer

import torch
from typing import Optional


class RobotDataSubscriber(Node):
    def __init__(self):
        super().__init__('robot_data_subscriber')

        self.declare_parameter(
            'joint_order_follower', [
                'right_thumb_1_joint', 'right_thumb_2_joint',
                'right_index_1_joint', 'right_middle_1_joint',
                'right_ring_1_joint', 'right_little_1_joint',
                'left_thumb_1_joint', 'left_thumb_2_joint',
                'left_index_1_joint', 'left_middle_1_joint',
                'left_ring_1_joint', 'left_little_1_joint',
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
            'joint_order_leader_arms', [
                'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
                'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7',
                'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
                'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7',
            ]
        )

        self.joint_order_follower = self.get_parameter('joint_order_follower').value
        self.joint_order_leader_r_hand = self.get_parameter('joint_order_leader_r_hand').value
        self.joint_order_leader_l_hand = self.get_parameter('joint_order_leader_l_hand').value
        self.joint_order_leader_arms = self.get_parameter('joint_order_leader_arms').value

        self.latest_observation = None
        self.latest_action = None

        self.follower_joint_states = Subscriber(self, JointState, '/follower/joint_states')
        self.leader_hand_right = Subscriber(self, JointState, '/leader/right_hand_states')
        self.leader_hand_left = Subscriber(self, JointState, '/leader/left_hand_states')
        self.leader_arms = Subscriber(self, JointState, '/leader/arm_states')

        self.leader_neck = Subscriber(self, JointState, '/neck_controller/joint_states')
        self.leader_linear = Subscriber(self, JointState, '/body_controller/joint_states')

        self.sync = ApproximateTimeSynchronizer(
            [
                self.follower_joint_states,
                self.leader_hand_right,
                self.leader_hand_left,
                self.leader_arms,
                self.leader_neck,
                self.leader_linear
            ],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self.synced_callback)

    def synced_callback(
        self,
        follower_joint_states_msg: JointState,
        leader_hand_right_msg: JointState,
        leader_hand_left_msg: JointState,
        leader_arms_msg: JointState,
        leader_neck_msg: JointState,
        leader_linear_msg: JointState
    ):
        try:
            obs_pos_map = dict(zip(follower_joint_states_msg.name, follower_joint_states_msg.position))
            ordered_obs = [obs_pos_map[name] for name in self.joint_order_follower]
            obs_tensor = torch.tensor(ordered_obs, dtype=torch.float32)

            r_hand_pos_map = dict(zip(leader_hand_right_msg.name, leader_hand_right_msg.position))
            ordered_leader_r_hand = [r_hand_pos_map[name] for name in self.joint_order_leader_r_hand]

            l_hand_pos_map = dict(zip(leader_hand_left_msg.name, leader_hand_left_msg.position))
            ordered_leader_l_hand = [l_hand_pos_map[name] for name in self.joint_order_leader_l_hand]

            arms_pos_map = dict(zip(leader_arms_msg.name, leader_arms_msg.position))
            ordered_leader_arms = [arms_pos_map[name] for name in self.joint_order_leader_arms]

            act_pos = (
                ordered_leader_r_hand +
                ordered_leader_l_hand +
                ordered_leader_arms +
                list(leader_neck_msg.position) +
                list(leader_linear_msg.position)
            )
            act_tensor = torch.tensor(act_pos, dtype=torch.float32)

            self.latest_observation = {"observation.state": obs_tensor}
            self.latest_action = {"action": act_tensor}

            self.get_logger().info("Synced data received.")

        except KeyError as e:
            self.get_logger().error(f"Joint name missing in incoming message: {e}")

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
