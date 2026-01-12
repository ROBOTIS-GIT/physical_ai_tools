# Communication Module - FEATURES

## Overview
Communication module responsible for ROS2 topic subscription/publication and service management.

## Classes

### Communicator
**File**: `communicator.py`

Central manager for ROS2 communication. Handles camera/joint topic subscription, ROSbag recording control, and file browser services.

#### Constants
| Name | Value | Description |
|------|-------|-------------|
| `SOURCE_CAMERA` | 'camera' | Camera source category |
| `SOURCE_FOLLOWER` | 'follower' | Follower joint category |
| `SOURCE_LEADER` | 'leader' | Leader joint category |
| `MODE_COLLECTION` | 'collection' | Data collection mode (image+follower+leader) |
| `MODE_INFERENCE` | 'inference' | Inference mode (image+follower only) |

#### Methods

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `__init__` | node, operation_mode, params | - | Initialize subscribers/publishers/services |
| `get_all_topics` | - | List[str] | Return camera+joint+extra topic list |
| `get_latest_data` | - | Tuple[Dict, Dict, Dict] | Return latest camera/follower/leader messages |
| `clear_latest_data` | - | - | Clear all cached messages |
| `publish_action` | joint_msg_datas: Dict | - | Publish joint commands |
| `publish_status` | status: TaskStatus | - | Publish task status |
| `prepare_rosbag` | topics: List[str] | - | Prepare ROSbag recording |
| `start_rosbag` | rosbag_uri: str | - | Start ROSbag recording |
| `stop_rosbag` | - | - | Stop ROSbag recording |
| `stop_and_delete_rosbag` | - | - | Stop and delete ROSbag |
| `finish_rosbag` | - | - | Finish ROSbag recording |
| `cleanup` | - | - | Clean up all resources |

#### Subscribers (Auto-generated)
| Category | Message Type | Naming Rule |
|----------|-------------|-------------|
| camera | `CompressedImage` | Parsed from params['camera_topic_list'] |
| follower | `JointState` / `Odometry` | Name contains 'follower', uses Odometry if 'mobile' |
| leader | `JointTrajectory` / `Twist` | Name contains 'leader', uses Twist if 'mobile' |

#### Publishers
| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/task/status` | `TaskStatus` | Current recording/training status |
| `heartbeat` | `Empty` | Server heartbeat signal |
| (dynamic) | `JointTrajectory`/`Twist` | Leader joint command publishing |

#### Services
| Service | Type | Description |
|---------|------|-------------|
| `/image/get_available_list` | `GetImageTopicList` | Get camera topic list |
| `/browse_file` | `BrowseFile` | File system browsing |
| `/dataset/edit` | `EditDataset` | Dataset merge/delete |
| `/dataset/get_info` | `GetDatasetInfo` | Get dataset information |

#### Service Clients
| Service | Type | Description |
|---------|------|-------------|
| `rosbag_recorder/send_command` | `SendCommand` | ROSbag recording control |

---

### MultiSubscriber
**File**: `multi_subscriber.py`

Category-based subscriber management with source filtering support.

#### Methods

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `__init__` | node, enabled_sources | - | Set node and enabled sources |
| `is_source_enabled` | category: str | bool | Check if category is enabled |
| `set_enabled_sources` | enabled_sources: Set[str] | - | Change enabled sources |
| `add_subscriber` | category, name, topic, msg_type, callback, qos_profile | - | Add subscriber |
| `cleanup` | - | - | Clean up all subscribers |

#### QoS Default
```python
depth=1
reliability=BEST_EFFORT
history=KEEP_LAST
```

---

## Dependencies

### Internal
| Module | Usage |
|--------|-------|
| `data_processing.data_editor` | DataEditor class |
| `utils.file_browse_utils` | FileBrowseUtils class |
| `utils.parameter_utils` | parse_topic_list, parse_topic_list_with_names |

### External (ROS2)
| Package | Components |
|---------|------------|
| `rclpy` | Node, QoSProfile |
| `physical_ai_interfaces` | BrowserItem, DatasetInfo, TaskStatus, BrowseFile, EditDataset, GetDatasetInfo, GetImageTopicList |
| `rosbag_recorder` | SendCommand |
| `sensor_msgs` | CompressedImage, JointState |
| `geometry_msgs` | Twist |
| `nav_msgs` | Odometry |
| `trajectory_msgs` | JointTrajectory |
| `std_msgs` | Empty, String |

---

## Usage Example

```python
from physical_ai_server.communication.communicator import Communicator

# Initialize
communicator = Communicator(
    node=self.node,
    operation_mode=Communicator.MODE_COLLECTION,
    params={
        'camera_topic_list': ['cam1:/camera/image/compressed'],
        'joint_topic_list': ['follower_arm:/joint_states', 'leader_arm:/leader/joint_trajectory'],
        'rosbag_extra_topic_list': []
    }
)

# Get latest data
camera_msgs, follower_msgs, leader_msgs = communicator.get_latest_data()

# ROSbag recording
communicator.prepare_rosbag(communicator.get_all_topics())
communicator.start_rosbag('/path/to/rosbag')
# ... recording ...
communicator.stop_rosbag()
communicator.finish_rosbag()

# Cleanup
communicator.cleanup()
```

---

## Notes
- Joint topic names must contain 'follower' or 'leader' keyword
- Topics with 'mobile' keyword use Twist/Odometry messages
- Recording features disabled if ROSbag service is not connected
