
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    config_dir = get_package_share_directory('data_collector')
    robot_params = load_yaml(os.path.join(config_dir, 'config', 'joint_order.yaml'))

    return LaunchDescription([
        Node(
            package='data_collector',
            executable='data_collector',
            name='data_collector',
            output='screen',
            parameters=[
                robot_params['data_collector']['ros__parameters'],
            ]
        )
    ])