import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
import torch
from typing import Optional


class ObservationCollector(Node):
    def __init__(self):
        super().__init__('topic_to_observation')

        self.declare_parameter('expected_joint_order_follower', [
            'joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint'
        ])
        self.expected_joint_order_follower = self.get_parameter('expected_joint_order_follower').value
        self.latest_observation = None
        self.sub = self.create_subscription(JointState, '/joint_states', self.callback, 10)

    def callback(self, msg: JointState):
        pos_map = dict(zip(msg.name, msg.position))
        ordered_pos = [pos_map[name] for name in self.expected_joint_order_follower]
        obs_tensor = torch.tensor(ordered_pos, dtype=torch.float32)

        self.latest_observation = {
            'observation.state': obs_tensor,
        }

    def get_latest_data(self) -> Optional[dict]:
        return self.latest_observation

def main(args=None):
    rclpy.init(args=args)
    node = ObservationCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 