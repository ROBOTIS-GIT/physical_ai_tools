from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('lerobot_recorder')

    # Declare launch arguments
    repo_id_arg = DeclareLaunchArgument(
        'repo_id',
        default_value='wdean/test1',
        description='Repository ID for the dataset'
    )

    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Recording frame rate'
    )

    task_arg = DeclareLaunchArgument(
        'task',
        default_value='joint_state_task',
        description='Task name for the dataset'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the recorder node'
    )

    # Load joint names from config file if it exists
    joint_names = [
        'arm_r_joint1', 'r_rh_r1_joint', 'arm_r_joint3', 'arm_l_joint3',
        'l_rh_r1_joint', 'arm_l_joint6', 'neck_joint1', 'neck_joint2',
        'arm_l_joint7', 'arm_l_joint1', 'arm_l_joint2', 'arm_r_joint7',
        'arm_r_joint6', 'arm_r_joint5', 'arm_r_joint4', 'arm_l_joint5',
        'linear_joint', 'arm_r_joint2', 'arm_l_joint4'
    ]

    config_path = os.path.join(package_dir, 'config', 'joint_names.yaml')
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            if 'joint_names' in config:
                joint_names = config['joint_names']

    return LaunchDescription([
        # Launch arguments
        repo_id_arg,
        fps_arg,
        task_arg,
        log_level_arg,

        # Recorder node
        Node(
            package='lerobot_recorder',
            executable='joint_state_recorder',
            name='joint_state_recorder',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[
                {
                    'repo_id': LaunchConfiguration('repo_id'),
                    'fps': LaunchConfiguration('fps'),
                    # 'root_dir': os.path.join(os.path.expanduser('~'), '.cache/huggingface/lerobot'),
                    'task': LaunchConfiguration('task'),
                    'joint_names': joint_names
                }
            ]
        )
    ])
