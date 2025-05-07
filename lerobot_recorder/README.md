# LeRobot Recorder

A ROS2 node for recording robot data in the LeRobot dataset format.

## Overview

This package provides a ROS2 node that subscribes to camera images, robot states, and actions, and records them in the LeRobot dataset format. The recorded data can be used for training robot learning models.

## Features

- Records camera images, robot states, and actions at a configurable frame rate
- Saves data in the LeRobot dataset format compatible with Hugging Face
- Supports video and state data recording
- Configurable parameters for dataset creation

## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd /path/to/your/ros2_ws/src
git clone https://github.com/yourusername/lerobot_recorder.git
```

2. Install dependencies:
```bash
cd /path/to/your/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select lerobot_recorder
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Usage

Run the recorder node:
```bash
ros2 run lerobot_recorder ros2_lerobot_recorder
```

### Parameters

- `repo_id` (string, default: "my_robot_dataset"): The repository ID for the dataset
- `fps` (int, default: 30): The frame rate for recording
- `root_dir` (string, default: "~/.cache/huggingface/lerobot"): The root directory for storing the dataset
- `task` (string, default: "default_task"): The task name for the dataset

Example with parameters:
```bash
ros2 run lerobot_recorder ros2_lerobot_recorder --ros-args -p repo_id:=my_custom_dataset -p fps:=60 -p task:=pick_and_place
```

## Topics

The node subscribes to the following topics:

- `/camera/image_raw`: Camera images (sensor_msgs/Image)
- `/joint_states`: Robot joint states (std_msgs/Float32MultiArray)
- `/joint_commands`: Robot joint commands (std_msgs/Float32MultiArray)

## Dataset Format

The recorded data is saved in the LeRobot dataset format, which includes:

- Camera images as videos
- Robot states as float arrays
- Actions as float arrays
- Timestamps and task information

## License

This package is licensed under the Apache License 2.0. 