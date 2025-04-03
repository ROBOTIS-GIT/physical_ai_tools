# physical_ai_tools

This repository offers an interface for developing physical AI applications using LeRobot.

## Installation

### 1. Clone the Source Code
```bash
cd ~/your_workspace/src
git clone git@github.com:ROBOTIS-GIT/physical_ai_tools.git
```

### 2. Install 🤗 LeRobot
```bash
cd ~/your_workspace/src/physical_ai_tools/lerobot
pip install -e .
```

### 3. Build the Workspace
Navigate to your ROS 2 workspace directory and build the package using `colcon`:
```bash
cd ~/your_workspace
colcon build --symlink-install
```

### 4. Source the Workspace
After the build completes successfully, source the setup script:
```bash
source install/setup.bash
```

### 5. Install the Data Collector Package
Make the package available as a Python module in your current environment:
```bash
cd ~/your_workspace/src/physical_ai_tools/data_collector
pip install .
```

---

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
cd ~/your_workspace/src/physical_ai_tools/lerobot
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

Once identified, update the camera indexes in the `"noza"` robot configuration file:

```
lerobot/common/robot_devices/robots/configs.py
```

Modify it like so:
```python
@RobotConfig.register_subclass("noza")
@dataclass
class NozaRobotConfig(ManipulatorRobotConfig):
    [...]
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "cam_head": OpenCVCameraConfig(
                camera_index=0,
                fps=30,
                width=640,
                height=480,
            ),
            "cam_wrist_1": OpenCVCameraConfig(
                camera_index=1,
                fps=30,
                width=640,
                height=480,
            ),
            "cam_wrist_2": OpenCVCameraConfig(
                camera_index=2,
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

Navigate to the `lerobot` directory:
```bash
cd ~/your_workspace/src/physical_ai_tools/lerobot
```

Run the following command to start recording your Hugging Face dataset:
```bash
python lerobot/scripts/control_robot.py \
  --robot.type=noza \
  --control.type=record \
  --control.single_task="pick and place objects" \
  --control.fps=30 \
  --control.repo_id=${HF_USER}/noza_test \
  --control.tags='["tutorial"]' \
  --control.episode_time_s=20 \
  --control.reset_time_s=10 \
  --control.num_episodes=2 \
  --control.push_to_hub=true \
  --control.use_ros=true
```

💡 Make sure to replace `${HF_USER}` with your actual Hugging Face username.

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
