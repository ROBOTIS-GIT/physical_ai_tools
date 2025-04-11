import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    config_dir = get_package_share_directory('data_collector')
    robot_params = load_yaml(os.path.join(config_dir, 'config', 'joint_order.yaml'))
    return LaunchDescription([
        Node(
            package='policy_to_trajectory',
            executable='action_to_trajectory',
            name='policy_trajectory_node',
            output='screen',
        ),
        Node(
            package='policy_to_trajectory',
            executable='topic_to_observation',
            name='policy_trajectory_node',
            output='screen',
            parameters=[
                robot_params['data_collector']['ros__parameters'],
            ]
        ),
    ])
