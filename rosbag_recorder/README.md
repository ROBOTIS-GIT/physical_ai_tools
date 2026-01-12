# ROSbag Recorder with Image MP4 Compression

## Overview

This package provides ROSbag recording functionality with automatic image topic compression to MP4 format. Images are stored as MP4 videos (for storage efficiency), while only metadata is stored in the ROSbag MCAP file.

## Features

- **MCAP Format**: Enterprise-grade ROSbag2 storage format
- **Image MP4 Compression**: Automatic compression of sensor_msgs/Image topics to MP4 videos
- **Metadata-Only Storage**: ROSbag stores only image metadata, actual frames in MP4
- **Synchronized Recording**: Maintains timestamp synchronization across all topics
- **Service-Based Control**: PREPARE → START → STOP workflow for controlled recording

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                  Image Bag Recorder Node                 │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌──────────────┐          ┌──────────────────┐         │
│  │   Regular    │          │  Image Topics    │         │
│  │   Topics     │          │ (sensor_msgs/    │         │
│  │              │          │     Image)       │         │
│  └──────┬───────┘          └────────┬─────────┘         │
│         │                           │                   │
│         │                           │                   │
│         ▼                           ▼                   │
│  ┌──────────────┐          ┌──────────────────┐         │
│  │   ROSbag     │          │  Image           │         │
│  │   (Full Data)│          │  Compressor      │         │
│  └──────┬───────┘          └────────┬─────────┘         │
│         │                           │                   │
│         │                    ┌──────┴──────┐            │
│         │                    │             │            │
│         │                    ▼             ▼            │
│         │            ┌──────────┐  ┌──────────────┐     │
│         │            │   MP4    │  │  Metadata    │     │
│         │            │  Videos  │  │  Messages    │     │
│         │            └──────────┘  └──────┬───────┘     │
│         │                                 │             │
│         └─────────────────────────────────┘             │
│                          │                              │
└──────────────────────────┼──────────────────────────────┘
                           │
                           ▼
              ┌────────────────────────┐
              │  ROSbag Directory      │
              ├────────────────────────┤
              │  ├─ metadata.yaml      │
              │  ├─ rosbag.db3         │
              │  │   (MCAP format)     │
              │  └─ videos/            │
              │      ├─ camera_image   │
              │      │   _raw.mp4      │
              │      └─ camera_depth   │
              │          _image.mp4    │
              └────────────────────────┘
```

## Building

### Prerequisites

Ensure you have the following dependencies installed:

```bash
# ROS2 Jazzy
sudo apt install ros-jazzy-rclcpp ros-jazzy-rosbag2-cpp \
  ros-jazzy-sensor-msgs ros-jazzy-std-msgs \
  ros-jazzy-cv-bridge ros-jazzy-image-transport

# OpenCV
sudo apt install libopencv-dev
```

### Build Package

```bash
cd /home/dongyun/main_ws
colcon build --packages-select rosbag_recorder --symlink-install
source install/setup.bash
```

## Usage

### 1. Launch the Recorder Node

```bash
# Basic launch
ros2 launch rosbag_recorder image_bag_recorder.launch.py

# With custom config
ros2 launch rosbag_recorder image_bag_recorder.launch.py \
  config_file:=/path/to/config.yaml
```

### 2. Control Recording via Service

The recorder uses a service-based workflow:

#### PREPARE: Set up topics to record
```bash
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 0, topics: ['/camera/image_raw', '/robot/joint_states']}"
```

#### START: Begin recording
```bash
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 1, uri: '/path/to/output_bag'}"
```

#### STOP: Stop recording
```bash
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 2}"
```

#### STOP_AND_DELETE: Stop and delete current recording
```bash
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 3}"
```

#### FINISH: Clean up subscriptions
```bash
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 4}"
```

### 3. Verify Recording

After recording, your output directory will contain:

```
output_bag/
├── metadata.yaml           # ROSbag metadata
├── rosbag_0.db3           # MCAP file with metadata
└── videos/                # MP4 compressed videos
    ├── camera_image_raw.mp4
    └── camera_depth_image_raw.mp4
```

## Testing in Docker

### Build and Run in physical_ai_server Container

```bash
# Start the container
docker compose -f physical_ai_tools/docker/docker-compose.yml up -d

# Enter the container
docker exec -it physical_ai_server bash

# Inside container: Build
cd /workspace
colcon build --packages-select rosbag_recorder --symlink-install
source install/setup.bash

# Run the node
ros2 run rosbag_recorder image_bag_recorder
```

### Example Test Script

Create a test publisher for image topics:

```bash
# Terminal 1: Run recorder
ros2 run rosbag_recorder image_bag_recorder

# Terminal 2: Prepare recording
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 0, topics: ['/test/image']}"

# Terminal 3: Publish test images (requires image_tools)
ros2 run image_tools cam2image --ros-args -r image:=/test/image

# Terminal 2: Start recording
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 1, uri: '/tmp/test_recording'}"

# Wait a few seconds...

# Terminal 2: Stop recording
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 2}"

# Verify output
ls -lh /tmp/test_recording/
ls -lh /tmp/test_recording/videos/
```

## Message Definitions

### ImageMetadata.msg

Stored in ROSbag instead of full Image messages:

```
std_msgs/Header header
uint32 frame_index           # Frame index in MP4 video
uint32 width                 # Original image width
uint32 height                # Original image height
string encoding              # Original encoding (e.g., "rgb8")
string video_file_path       # Relative path to MP4 file
string source_topic          # Source topic name
```

## Configuration

Edit `config/recorder_config.yaml`:

```yaml
# Topics to record
topics_to_record:
  - /camera/image_raw
  - /camera/depth/image_raw
  - /robot/joint_states

# Video compression settings
video_compression:
  fps: 30.0
  codec: "h264"
  quality: 85

# Recording settings
recording:
  output_directory: "~/rosbag_recordings"
  use_timestamp: true
  qos_depth: 100
```

## Data Playback

To reconstruct images from recorded data:

1. Read ImageMetadata messages from ROSbag
2. Use frame_index to extract corresponding frame from MP4 video
3. Reconstruct sensor_msgs/Image message with original encoding

Example playback script (to be implemented):

```python
# TODO: Create playback utility
# python3 scripts/playback_images.py /path/to/bag
```

## LeRobot Dataset Conversion

The recorded data can be converted to LeRobot format using the conversion tool in the LeRobot container:

```bash
# In LeRobot container
python scripts/convert_mcap_to_lerobot.py \
  --input /path/to/recorded_bag \
  --output /path/to/lerobot_dataset
```

## Troubleshooting

### Build Errors

If you encounter `ModuleNotFoundError: No module named 'em'`:

```bash
pip install empy
```

### Video Codec Issues

If H.264 codec fails, the recorder automatically falls back to MP4V codec. To check available codecs:

```bash
python3 -c "import cv2; print(cv2.getBuildInformation())"
```

### Performance Optimization

For high-frequency image topics (>30 Hz), consider:

1. Adjusting QoS depth in configuration
2. Using lower resolution cameras
3. Adjusting MP4 compression quality

## Code Quality

Lint checking:

```bash
# C++ lint
cd /home/dongyun/main_ws
ament_cpplint physical_ai_tools/rosbag_recorder

# Should output: "No problems found"
```

## License

Apache 2.0

## Authors

- Dongyun Kim
- Original service_bag_recorder: Woojin Wie, Kiwoong Park

## Contributing

1. Ensure `ament_cpplint` passes with no errors
2. Test in physical_ai_server Docker container
3. Follow ROS2 C++ coding conventions
4. Document all public APIs
