# Rosbag to LeRobot Dataset Converter

This ROS2 package converts rosbag data to the LeRobot dataset format. It reads episode data from rosbag files and converts them into a structured dataset that can be used for robot learning.

## Features

- Converts ROS2 rosbag files to LeRobot dataset format
- Supports multiple camera streams
- Extracts joint state data
- Synchronizes data by timestamps
- Configurable parameters via ROS2 parameters or launch files

## Installation

1. Build the package:
```bash
cd /root/ros2_ws
colcon build --packages-select rosbag_to_lerobot
source install/setup.bash
```

## Usage

### Using the Launch File

The easiest way to run the converter is using the provided launch file:

```bash
ros2 launch rosbag_to_lerobot converter.launch.py
```

You can customize the parameters:

```bash
ros2 launch rosbag_to_lerobot converter.launch.py \
    rosbag_dir:=/path/to/your/rosbag/data \
    output_repo_id:=your_dataset_name \
    task_name:=your_task_name \
    fps:=30 \
    use_videos:=true \
    robot_type:=mobile_robot
```

### Using the Node Directly

You can also run the node directly:

```bash
ros2 run rosbag_to_lerobot rosbag_to_lerobot_converter
```

### Parameters

- `rosbag_dir` (string): Directory containing rosbag episode data
- `output_repo_id` (string): Output repository ID for the LeRobot dataset
- `task_name` (string): Task name for the dataset
- `fps` (int): Frames per second for the dataset
- `use_videos` (bool): Whether to use video format for images
- `robot_type` (string): Robot type for the dataset

## Data Structure

The converter expects the following directory structure:

```
rosbag_dir/
├── 0/          # Episode 0
│   ├── rosbag_0.db3
│   └── ...
├── 1/          # Episode 1
│   ├── rosbag_1.db3
│   └── ...
└── ...
```

## Supported Topics

The converter is configured to read the following topics:

### Joint States
- All joint positions from the robot

### Camera Topics
- `cam_head`: `/zed/zed_node/left/image_rect_color/compressed`
- `cam_wrist_left`: `/camera_left/camera_left/color/image_rect_raw/compressed`
- `cam_wrist_right`: `/camera_right/camera_right/color/image_rect_raw/compressed`

## Joint Names

The converter extracts the following joint positions:
- `arm_l_joint1` through `arm_l_joint7`
- `gripper_l_joint1`
- `arm_r_joint1` through `arm_r_joint7`
- `gripper_r_joint1`
- `head_joint1`, `head_joint2`
- `lift_joint`
- `linear_x`, `linear_y`, `angular_z`

## Output

The converter creates a LeRobot dataset in the default cache directory:
`~/.cache/huggingface/lerobot/{output_repo_id}/`

## Dependencies

- ROS2 (Humble or later)
- Python packages: numpy, opencv-python, torch, cv-bridge
- LeRobot library

## Example

```bash
# Convert a dataset
ros2 launch rosbag_to_lerobot converter.launch.py \
    rosbag_dir:=/workspace/physical_ai_server/test/ffw_sg2_rev1_pickcoffeepat9 \
    output_repo_id:=test/ffw_sg2_rev1_pickcoffeepat9_converted \
    task_name:=pick_coffee_pat9
```

This will read all episodes from the specified directory and create a LeRobot dataset with the given repository ID.
