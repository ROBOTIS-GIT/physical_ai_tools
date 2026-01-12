# Docker Build and Test Guide

## Building in physical_ai_server Docker Container

### Prerequisites

1. Ensure Docker is running
2. Start the physical_ai_server container

```bash
cd /home/dongyun/main_ws
docker compose -f physical_ai_tools/docker/docker-compose.yml up -d physical_ai_server
```

### Build Process

#### Step 1: Enter Container

```bash
docker exec -it physical_ai_server bash
```

#### Step 2: Install Additional Dependencies (if needed)

```bash
# Inside container
apt-get update
apt-get install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    libopencv-dev \
    python3-pip

# Install empy (required for message generation)
pip3 install empy
```

#### Step 3: Build Package

```bash
# Inside container
cd /workspace
source /opt/ros/jazzy/setup.bash

# Build
colcon build --packages-select rosbag_recorder --symlink-install

# Source the workspace
source install/setup.bash
```

#### Step 4: Verify Build

```bash
# Check if executables exist
ls -l install/rosbag_recorder/lib/rosbag_recorder/

# Should show:
# - image_bag_recorder
# - service_bag_recorder

# Check lint
ament_cpplint physical_ai_tools/rosbag_recorder
```

## Testing

### Method 1: Manual Testing with Mock Publisher

#### Terminal 1: Start Recorder

```bash
docker exec -it physical_ai_server bash
cd /workspace
source install/setup.bash

ros2 run rosbag_recorder image_bag_recorder
```

#### Terminal 2: Prepare and Control Recording

```bash
docker exec -it physical_ai_server bash
cd /workspace
source install/setup.bash

# Prepare recording
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 0, topics: ['/test/image', '/test/joint_states']}"

# Start recording
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 1, uri: '/tmp/test_recording'}"

# After a few seconds, stop
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 2}"
```

#### Terminal 3: Publish Test Data

```bash
docker exec -it physical_ai_server bash
cd /workspace
source install/setup.bash

# Install image tools if needed
apt-get install -y ros-jazzy-image-tools

# Publish test images
ros2 run image_tools cam2image --ros-args -r image:=/test/image
```

### Method 2: Automated Test Script

```bash
# Terminal 1: Start recorder
docker exec -it physical_ai_server bash
cd /workspace
source install/setup.bash
ros2 run rosbag_recorder image_bag_recorder

# Terminal 2: Start test publisher
docker exec -it physical_ai_server bash
cd /workspace
source install/setup.bash
ros2 run image_tools cam2image --ros-args -r image:=/camera/image_raw

# Terminal 3: Run test script
docker exec -it physical_ai_server bash
cd /workspace
source install/setup.bash
bash physical_ai_tools/rosbag_recorder/scripts/test_recorder.sh
```

## Verifying Output

### Check Recorded Data

```bash
# List output directory
ls -lh /tmp/test_recording/

# Expected structure:
# /tmp/test_recording/
# ├── metadata.yaml
# ├── rosbag_0.db3
# └── videos/
#     └── camera_image_raw.mp4 (or test_image.mp4)

# Check bag info
ros2 bag info /tmp/test_recording/

# Play video (if GUI available)
ffplay /tmp/test_recording/videos/camera_image_raw.mp4

# Or check video info
ffprobe /tmp/test_recording/videos/camera_image_raw.mp4
```

### Verify Metadata Messages

```bash
# Play bag and echo metadata
ros2 bag play /tmp/test_recording/

# In another terminal
ros2 topic echo /camera/image_raw/metadata
```

## Common Issues

### Issue 1: ModuleNotFoundError: No module named 'em'

**Solution:**
```bash
docker exec -it physical_ai_server bash
pip3 install empy
```

### Issue 2: OpenCV not found

**Solution:**
```bash
docker exec -it physical_ai_server bash
apt-get update
apt-get install -y libopencv-dev
```

### Issue 3: cv_bridge not found

**Solution:**
```bash
docker exec -it physical_ai_server bash
apt-get install -y ros-jazzy-cv-bridge ros-jazzy-image-transport
```

### Issue 4: Video codec error

**Symptom:** Error like "Could not open video writer"

**Solution:** Check available codecs and use fallback:
```bash
# Check OpenCV build info
python3 -c "import cv2; print(cv2.getBuildInformation())"

# The code automatically falls back to MP4V if H.264 fails
```

### Issue 5: Permission denied writing to output directory

**Solution:**
```bash
# Use /tmp for testing
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 1, uri: '/tmp/test_recording'}"

# Or create directory with proper permissions
mkdir -p /workspace/recordings
chmod 777 /workspace/recordings
```

## Performance Testing

### High-Frequency Image Topics

Test with different frame rates:

```bash
# Publish at 30 FPS
ros2 run image_tools cam2image --ros-args -p frequency:=30.0

# Publish at 60 FPS
ros2 run image_tools cam2image --ros-args -p frequency:=60.0

# Monitor resource usage
docker stats physical_ai_server
```

### Multiple Image Topics

```bash
# Terminal 1: Camera 1
ros2 run image_tools cam2image --ros-args -r image:=/camera1/image_raw

# Terminal 2: Camera 2
ros2 run image_tools cam2image --ros-args -r image:=/camera2/image_raw

# Terminal 3: Record both
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 0, topics: ['/camera1/image_raw', '/camera2/image_raw']}"

ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 1, uri: '/tmp/multi_camera_test'}"
```

## Cleaning Up

```bash
# Remove test recordings
docker exec -it physical_ai_server bash
rm -rf /tmp/test_recording /tmp/multi_camera_test

# Stop container
docker compose -f physical_ai_tools/docker/docker-compose.yml down
```

## Next Steps

After successful testing in Docker:

1. Test with real robot hardware
2. Integrate with Physical AI Manager UI
3. Test dataset conversion to LeRobot format
4. Performance optimization for production use

## References

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [MCAP Format](https://mcap.dev/)
- [OpenCV VideoWriter](https://docs.opencv.org/4.x/dd/d43/tutorial_py_video_display.html)
- [cv_bridge](http://wiki.ros.org/cv_bridge)
