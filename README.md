# physical_ai_tools
This repository offers an interface for developing physical AI applications using LeRobot

## Installation

### 1. Download our source code.:
```bash
cd ~/your_work_space/src
git clone https://github.com/ROBOTIS-GIT/open_platform_ai.git
```

### 2. Install 🤗 LeRobot.:
```bash
cd ~/your_work_space/src/open_source_ai/lerobot
pip install -e .
```

### 3. Navigate to your ROS 2 workspace directory and build the package using colcon.
```bash
cd ~/your_work_space
colcon build --symlink-install
```

### 4. After the build completes successfully, don't forget to source the setup script before running any nodes.:
```bash
source install/setup.bash
```

### 5. To make the package available as a Python module in your current environment, navigate to the package directory and install it using pip.
```bash
cd ~/your_work_space/src/open_source_ai/robot_data_subscriber
pip install .
```
## Record LeRobot datasets

### 1. To start the node that subscribes to joint state data, use the following ros2 launch command. 
```bash
ros2 launch robot_data_subscriber robot_data_subscriber.launch.py 
```

### 2. Navigate to the lerobot directory and run the following command to start recording data for your Hugging Face dataset.
```bash
cd ~/your_work_space/src/open_source_ai/lerobot
```

```bash
cd ~/your_work_space/src/open_source_ai/lerobot
python lerobot/scripts/control_robot.py \
  --robot.type=noza \
  --control.type=record \
  --control.single_task="pick and place objects" \
  --control.fps=30 \
  --control.repo_id=${HF_USER}/noza_test \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=20 \
  --control.reset_time_s=10 \
  --control.num_episodes=2 \
  --control.push_to_hub=true \
  --control.use_ros=true
```

### 🔧 Key Parameters to Customize

To create your own dataset, you only need to modify the following three options:

- **`--control.episode_time_s`**  
  Duration (in seconds) to record each episode.

- **`--control.reset_time_s`**  
  Time allocated (in seconds) for resetting your environment between episodes.

- **`--control.num_episodes`**  
  Total number of episodes to record for the dataset.

These parameters help define the structure and timing of your dataset recording sessions. Adjust them according to the complexity and length of your task.
