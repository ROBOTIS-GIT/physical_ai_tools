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

### 2. Check Your Camera Serial Numbers

To include image data, check serial numbers of your cameras:
```bash
rs-enumerate-devices --compact
```
Example output:

```text
Device Name                   Serial Number       Firmware Version
Intel RealSense D435I         111111111111        5.16.0.1
Device info: 
    Name                          : 	Intel RealSense D435I
    Serial Number                 : 	111111111111
    Firmware Version              : 	5.16.0.1
    Recommended Firmware Version  : 	5.16.0.1
    Physical Port                 : 	/sys/devices/pci0000:00/0000:00:14.0/usb2/2-6/2-6.1/2-6.1:1.0/video4linux/video0
    Debug Op Code                 : 	15
    Advanced Mode                 : 	YES
    Product Id                    : 	0B3A
    Camera Locked                 : 	YES
    Usb Type Descriptor           : 	3.2
    Product Line                  : 	D400
    Asic Serial Number            : 	001623050773
    Firmware Update Id            : 	001623050773
    Dfu Device Path               : 	
```

Once identified, update the camera serial numbers in the `"ffw"` robot configuration file:

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
            "cam_head": IntelRealSenseCameraConfig(
                serial_number='111111111111',  # To be chanaged
                fps=30,
                width=640,
                height=480,
            ),
            "cam_wrist_1": IntelRealSenseCameraConfig(
                serial_number='222222222222',  # To be chanaged
                fps=30,
                width=640,
                height=480,
            ),
            "cam_wrist_2": IntelRealSenseCameraConfig(
                serial_number='333333333333',  # To be chanaged
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

Open a terminal, and navigate to the `lerobot` directory:
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
  --control.warmup_time_s=5 \
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

### 1. Evaluate your policy

You can evaluate the policy on the robot using the `record` mode, which allows you to visualize the evaluation later on.

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=ffw \
  --control.type=record \
  --control.single_task="pick and place objects" \
  --control.fps=30 \
  --control.repo_id=${HF_USER}/eval_ffw_test \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=20 \
  --control.reset_time_s=10 \
  --control.num_episodes=2 \
  --control.push_to_hub=true \
  --control.use_ros=true \
  --control.policy.path=outputs/train/act_ffw_test/checkpoints/last/pretrained_model \
  --control.play_sounds=false
```

### 2. Visualize Evaluation

You can then visualize the evaluation results using the following command:

```bash
python lerobot/scripts/visualize_dataset_html.py \
  --repo-id ${HF_USER}/eval_act_ffw_test
```
