# physical_ai_tools

This repository offers an interface for developing physical AI applications using LeRobot and ROS 2.

## Installation

### 1. Clone the Source Code
```bash
cd ~/${WORKSPACE}/src
git clone https://github.com/ROBOTIS-GIT/physical_ai_tools.git --recursive
```

### 2. Install 🤗 LeRobot
```bash
cd ~/${WORKSPACE}/src/physical_ai_tools/lerobot
pip install --no-binary=av -e .
```

> **NOTE:** If you encounter build errors, you may need to install additional dependencies (`cmake`, `build-essential`, and `ffmpeg libs`). On Linux, run:
`sudo apt-get install cmake build-essential python-dev pkg-config libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libswscale-dev libswresample-dev libavfilter-dev pkg-config`. For other systems, see: [Compiling PyAV](https://pyav.org/docs/develop/overview/installation.html#bring-your-own-ffmpeg)

If you're using a Docker container, you may need to add the `--break-system-packages` option when installing with `pip`.
```bash
pip install --no-binary=av -e . --break-system-packages
```

### 3. Build the Workspace
Navigate to your ROS 2 workspace directory and build the package using `colcon`:
```bash
cd ~/${WORKSPACE}
colcon build --symlink-install --packages-select physical_ai_tools
```

### 4. Source the Workspace
After the build completes successfully, source the setup script:
```bash
source ~/${WORKSPACE}/install/setup.bash
```

### 5. Install Packages
Make the packages available as a Python module in your current environment:
```bash
cd ~/${WORKSPACE}/src/physical_ai_tools/data_collector
pip install .
```
```bash
cd ~/${WORKSPACE}/src/physical_ai_tools/policy_to_trajectory
pip install .
```
## Record LeRobot Datasets

### 1. Authenticate with Hugging Face
Make sure you've logged in using a **write-access token** generated from your [Hugging Face settings](https://huggingface.co/settings/tokens):
```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN} --add-to-git-credential
```

Store your Hugging Face username in a variable:
```bash
HF_USER=$(huggingface-cli whoami | head -n 1)
echo $HF_USER
```

---

### 2. Check Your Camera Indexes

To include image data, check which camera indexes are available on your system:
```bash
cd ~/${WORKSPACE}/src/physical_ai_tools/lerobot
```
```bash
python lerobot/common/robot_devices/cameras/opencv.py \
    --images-dir outputs/images_from_opencv_cameras
```

Example output:

```text
Linux detected. Finding available camera indices through scanning '/dev/video*' ports
Camera found at index 0
Camera found at index 1
Camera found at index 2
...
Saving images to outputs/images_from_opencv_cameras
```

Check the saved images in `outputs/images_from_opencv_cameras` to determine which index corresponds to which physical camera:
```text
camera_00_frame_000000.png
camera_01_frame_000000.png
...
```

Once identified, update the camera indexes in the `"ffw"` robot configuration file:

```
cd lerobot/common/robot_devices/robots/configs.py
```

Modify it like so:
```python
@RobotConfig.register_subclass("ffw")
@dataclass
class FFWRobotConfig(ManipulatorRobotConfig):
    [...]
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "cam_head": OpenCVCameraConfig(
                camera_index=0,  # To be changed
                fps=30,
                width=640,
                height=480,
            ),
            "cam_wrist_1": OpenCVCameraConfig(
                camera_index=1,  # To be changed
                fps=30,
                width=640,
                height=480,
            ),
            "cam_wrist_2": OpenCVCameraConfig(
                camera_index=2,  # To be changed
                fps=30,
                width=640,
                height=480,
            ),
        }
    )

    mock: bool = False
```

---

### 3. Record Your Dataset

Launch the ROS 2 data collector node.
```bash
# For OpenManipulator-X
ros2 launch data_collector data_collector.launch.py mode:=omx
# For AI Worker
ros2 launch data_collector data_collector.launch.py mode:=worker
```

Open a new terminal, and navigate to the `lerobot` directory:
```bash
cd ~/${WORKSPACE}/src/physical_ai_tools/lerobot
```

Run the following command to start recording your Hugging Face dataset:
```bash
python lerobot/scripts/control_robot.py \
  --robot.type=ffw \
  --control.type=record \
  --control.single_task="pick and place objects" \
  --control.fps=30 \
  --control.repo_id=${HF_USER}/ffw_test \
  --control.tags='["tutorial"]' \
  --control.episode_time_s=20 \
  --control.reset_time_s=10 \
  --control.num_episodes=2 \
  --control.push_to_hub=true \
  --control.use_ros=true  \
  --control.play_sounds=false
```

💡 Make sure to replace `${HF_USER}` with your actual Hugging Face username.

💡 If you don't want to push your Hugging Face dataset to hub, set --control.push_to_hub=false.

---

### 🔧 Key Parameters to Customize

To create your own dataset, you only need to modify the following five options:

- **`--control.repo_id`**  
  The Hugging Face dataset repository ID in the format `<username>/<dataset_name>`. This is where your dataset will be saved and optionally pushed to the Hugging Face Hub.

- **`--control.single_task`**  
  The name of the task you're performing (e.g., "pick and place objects").

- **`--control.episode_time_s`**  
  Duration (in seconds) to record each episode.

- **`--control.reset_time_s`**  
  Time allocated (in seconds) for resetting your environment between episodes.

- **`--control.num_episodes`**  
  Total number of episodes to record for the dataset.

Of course, you can modify other parameters as needed to better suit your use case.

---
🎉 All set — now you’re ready to create your dataset!

📺 Need a walkthrough? Check out this [video tutorial on YouTube](https://www.youtube.com/watch?v=n_Ljp_xuFEM) to see the full process of recording a dataset with LeRobot.

### 4. Visualize Your Dataset

You can also view your recorded dataset through a local web server. This is useful for quickly checking the collected data.

Run the following command:

```bash
python lerobot/scripts/visualize_dataset_html.py \
  --repo-id ${HF_USER}/ffw_test
```

🖥️ This will start a local web server and open your dataset in a browser-friendly format.

## Train a policy on Your Data

Run the following command to start training a policy using your dataset:

```bash
python lerobot/scripts/train.py \
  --dataset.repo_id=${HF_USER}/ffw_test \
  --policy.type=act \
  --output_dir=outputs/train/act_ffw_test \
  --job_name=act_ffw_test \
  --policy.device=cuda \
  --wandb.enable=true
```

(Optional) You can upload the latest checkpoint to the Hugging Face Hub with the following command:

```bash
huggingface-cli upload ${HF_USER}/act_ffw_test \
  outputs/train/act_ffw_test/checkpoints/last/pretrained_model
```

## Evaluation

### 1. Launch the ROS 2 action_to_trajectory and topic_to_observation nodes.:

```bash
# For OpenManipulator-X
ros2 launch policy_to_trajectory policy_to_trajectory.launch.py mode:=omx
# For AI Worker
ros2 launch policy_to_trajectory policy_to_trajectory.launch.py mode:=worker
```

### 2. Evaluate your policy

You can evaluate the policy on the robot using the `record` mode, which allows you to visualize the evaluation later on.

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=ffw \
  --control.type=record \
  --control.single_task="pick and place objects" \
  --control.fps=30 \
  --control.repo_id=${HF_USER}/eval_ffw_test \
  --control.tags='["tutorial"]' \
  --control.episode_time_s=20 \
  --control.reset_time_s=10 \
  --control.num_episodes=2 \
  --control.push_to_hub=true \
  --control.use_ros=true \
  --control.policy.path=outputs/train/act_ffw_test/checkpoints/last/pretrained_model \
  --control.play_sounds=false
```

### 3. Visualize Evaluation

You can then visualize the evaluation results using the following command:

```bash
python lerobot/scripts/visualize_dataset_html.py \
  --repo-id ${HF_USER}/eval_act_ffw_test
```
