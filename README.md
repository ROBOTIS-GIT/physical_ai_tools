# physical_ai_tools
This repository offers an interface for developing physical AI applications using LeRobot

## Installation

### 1. Download our source code.:
```bash
cd ~/your_work_space/src
git clone git@github.com:ROBOTIS-GIT/physical_ai_tools.git
```

### 2. Install 🤗 LeRobot.:
```bash
cd ~/your_work_space/src/physical_ai_tools/lerobot
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
cd ~/your_work_space/src/physical_ai_tools/data_collector
pip install .
```


## Record LeRobot datasets

### 1. Please, make sure you've logged in using a 'write-access token' generated from the [Hugging Face settings](https://huggingface.co/settings/tokens):
```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN} --add-to-git-credential
```
 - Store your Hugging Face repository name in a variable.
   ```bash
   HF_USER=$(huggingface-cli whoami | head -n 1)
   echo $HF_USER
   ```

### 2. You need to check the index of your cameras to include image datas. 
```bash
cd ~/your_work_space/src/physical_ai_tools/lerobot
python lerobot/common/robot_devices/cameras/opencv.py \
    --images-dir outputs/images_from_opencv_cameras
```
The output will look something like this if you have two cameras connected:
```bash
Linux detected. Finding available camera indices through scanning '/dev/video*' ports
[...]
Camera found at index 0
Camera found at index 1
Camera found at index 2
[...]
Connecting cameras
OpenCVCamera(0, fps=30.0, width=1920.0, height=1080.0, color_mode=rgb)
OpenCVCamera(1, fps=24.0, width=1920.0, height=1080.0, color_mode=rgb)
OpenCVCamera(2, fps=24.0, width=1920.0, height=1080.0, color_mode=rgb)
Saving images to outputs/images_from_opencv_cameras
Frame: 0000	Latency (ms): 39.52
[...]
Frame: 0046	Latency (ms): 40.07
Images have been saved to outputs/images_from_opencv_cameras
```
Check the saved images in ``outputs/images_from_opencv_cameras`` to identify which camera index corresponds to which physical camera(e.g. 0 for camera_00):
```bash
camera_00_frame_000000.png
[...]
camera_00_frame_000047.png
camera_01_frame_000000.png
[...]
camera_01_frame_000047.png
```

Once you've identified the correct camera indexes for your system (e.g., using tools like `cheese` or `v4l2-ctl`), update the camera index values in the `"noza"` robot configuration located in: 
/lerobot/common/robot_devices/robots/configs.py 
```bash
Modify the `camera_index` fields under the `"noza"` section like so:

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
🎉 All set — now you’re ready to create your dataset!

### 3. Navigate to the lerobot directory and run the following command to start recording data for your Hugging Face dataset.
```bash
cd ~/your_work_space/src/physical_ai_tools/lerobot
```
Then, run the following command to start recording data for your Hugging Face dataset:
```bash
cd ~/your_work_space/src/physical_ai_tools/lerobot
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
💡 Make sure to replace ${HF_USER} with your actual Hugging Face username.

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

These parameters help define the structure and timing of your dataset recording sessions. Adjust them according to the complexity and length of your task.
