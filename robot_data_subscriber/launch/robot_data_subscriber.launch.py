from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    config_dir = get_package_share_directory('robot_data_subscriber')
    robot_params = load_yaml(os.path.join(config_dir, 'config', 'robot_config.yaml'))

    return LaunchDescription([
        Node(
            package='robot_data_subscriber',
            executable='robot_data_subscriber',
            name='robot_data_subscriber',
            output='screen',
            parameters=[
                robot_params['robot_data_subscriber']['ros__parameters'],
            ]
        )
    ])