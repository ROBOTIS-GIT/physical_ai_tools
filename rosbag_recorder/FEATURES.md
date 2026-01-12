# ROSbag Recorder - FEATURES

## Overview
C++ ROS2 node-based ROSbag recorder. Supports MCAP format and image topic MP4 compression.

---

## Classes

### ServiceBagRecorder
**File**: `include/rosbag_recorder/service_bag_recorder.hpp`, `src/service_bag_recorder.cpp`

Service-based ROSbag recording node.

#### State Machine
```
IDLE → PREPARED → RECORDING → STOPPED → FINISHED
         ↑__________________________|
```

#### Service
| Service | Type | Description |
|---------|------|-------------|
| `rosbag_recorder/send_command` | SendCommand | Recording control commands |

#### Commands
| Command | Value | Description |
|---------|-------|-------------|
| `prepare` | - | Subscribe to topics, initialize bag |
| `start` | uri | Start recording (uri: save path) |
| `stop` | - | Stop recording (save) |
| `stop_and_delete` | - | Stop and delete bag |
| `finish` | - | Clean up all resources |

#### Private Methods
| Method | Parameters | Description |
|--------|------------|-------------|
| `handle_send_command` | request, response | Handle commands |
| `handle_prepare` | topics | Set up topic subscriptions |
| `handle_start` | uri | Start recording |
| `handle_stop` | - | Stop recording |
| `handle_stop_and_delete` | - | Stop and delete |
| `handle_finish` | - | Clean up resources |
| `handle_serialized_message` | topic, msg | Handle generic messages |
| `handle_image_message` | topic, msg | Image message → MP4 compression |
| `handle_compressed_image_message` | topic, msg | CompressedImage → MP4 |
| `create_subscriptions` | - | Create topic subscribers |

#### Topic Handling
| Topic Type | Handler | Output |
|------------|---------|--------|
| `sensor_msgs/Image` | Image subscription | MP4 + metadata in bag |
| `sensor_msgs/CompressedImage` | CompressedImage subscription | MP4 + metadata in bag |
| Other (JointState, etc.) | GenericSubscription | Direct bag storage |

---

### ImageCompressor
**File**: `include/rosbag_recorder/image_compressor.hpp`, `src/image_compressor.cpp`

FFmpeg pipe-based image → MP4 real-time compressor.

#### Attributes
| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `output_dir_` | string | - | Output directory |
| `fps_detection_frames_` | size_t | 10 | Frames for FPS detection |
| `writers_` | map | - | Per-topic FFmpeg pipes |
| `topic_buffers_` | map | - | FPS detection frame buffers |

#### Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `add_frame` | topic_name, image_msg | ImageMetadata | Add frame, return metadata |
| `finalize_all` | - | - | Close all writers |
| `finalize_writer` | topic_name | - | Close specific writer |
| `has_active_writer` | topic_name | bool | Check writer active |
| `is_tracking` | topic_name | bool | Check topic tracking |
| `get_detected_fps` | topic_name | double | Get detected FPS (0 = not detected) |

#### ImageMetadata
```cpp
struct ImageMetadata {
    uint32_t frame_index;
    int64_t timestamp_ns;
    uint32_t width;
    uint32_t height;
    std::string encoding;
};
```

#### FPS Detection
1. Buffer first N frames (`fps_detection_frames_`)
2. Calculate FPS from timestamp intervals
3. Initialize FFmpeg pipe
4. Flush buffered frames
5. Stream in real-time afterwards

---

### ImageBagRecorder
**File**: `include/rosbag_recorder/image_bag_recorder.hpp`, `src/image_bag_recorder.cpp`

Image-only ROSbag recorder (legacy).

---

## Output Structure

```
<bag_uri>/
├── metadata.yaml         # MCAP metadata
├── <bag_name>.mcap       # Non-image topics + image metadata
└── videos/
    ├── camera1.mp4       # Camera1 video
    ├── camera2.mp4       # Camera2 video
    └── ...
```

---

## Dependencies

### ROS2 Dependencies
| Package | Components |
|---------|------------|
| `rclcpp` | Node, Service, Subscription |
| `rosbag2_cpp` | Writer |
| `sensor_msgs` | Image, CompressedImage |

### External Dependencies
| Package | Components |
|---------|------------|
| `FFmpeg` | libavcodec, libavformat |
| `OpenCV` | cv::Mat, image conversion |

---

## Usage

### Launch
```bash
ros2 run rosbag_recorder service_bag_recorder
```

### Service Call
```bash
# Prepare
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 'prepare', topics: ['/camera/image', '/joint_states']}"

# Start
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 'start', uri: '/data/my_recording'}"

# Stop
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 'stop'}"

# Finish
ros2 service call /rosbag_recorder/send_command rosbag_recorder/srv/SendCommand \
  "{command: 'finish'}"
```

### Python Client (Communicator)
```python
# Communicator.prepare_rosbag(topics)
# Communicator.start_rosbag(uri)
# Communicator.stop_rosbag()
# Communicator.finish_rosbag()
```

---

## FFmpeg Command

```bash
ffmpeg -y -f rawvideo -vcodec rawvideo \
  -s {width}x{height} -pix_fmt bgr24 \
  -r {fps} -i - \
  -c:v libx264 -preset fast -crf 23 \
  -pix_fmt yuv420p \
  {output_path}
```

---

## Notes
- Image topics compressed to MP4, only metadata stored in bag
- Auto FPS detection (first 10 frame timestamp analysis)
- Uses MCAP format (enterprise compatibility)
- GenericSubscription handles all message types
- Image encoding converts BGR → RGB before sending to FFmpeg pipe
