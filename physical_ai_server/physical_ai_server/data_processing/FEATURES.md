# Data Processing Module - FEATURES

## Overview
Core data processing module responsible for data recording, conversion, editing, and HuggingFace integration.

---

## Classes

### DataConverter
**File**: `data_converter.py`

Utility for converting ROS2 message types to Tensor/NumPy arrays.

#### Methods
| Method | Input | Output | Description |
|--------|-------|--------|-------------|
| `compressed_image2cvmat` | CompressedImage, encoding | np.ndarray | Compressed image → OpenCV Mat |
| `joint_trajectory2tensor_array` | JointTrajectory, joint_order, format | np.array/torch.Tensor | Joint trajectory → array (ordered) |
| `joint_state2tensor_array` | JointState, joint_order, format | np.array/torch.Tensor | Joint state → array (ordered) |
| `twist2tensor_array` | Twist, format | np.array/torch.Tensor | Twist → [linear.x, linear.y, angular.z] |
| `odometry2tensor_array` | Odometry, format | np.array/torch.Tensor | Odometry → [vx, vy, wz] |
| `tensor_array2joint_msgs` | action, leader_topic_types, joint_orders | Dict[str, msg] | Array → ROS2 joint messages |

#### Dependencies
- `cv_bridge`: Image conversion
- `torch`, `numpy`: Tensor operations
- ROS2 msgs: `sensor_msgs`, `geometry_msgs`, `trajectory_msgs`, `nav_msgs`

---

### DataManager
**File**: `data_manager.py`

State machine-based data recording and HuggingFace integration manager.

#### State Machine
```
warmup → run → save → reset → run → ... → finish
                ↓
              stop (on abort)
```

| State | Description | Transition |
|-------|-------------|------------|
| `warmup` | Wait for recording preparation | → `run` (after warmup_time_s) |
| `run` | Recording frames | → `save` (after episode_time_s or low RAM) |
| `save` | Save episode and encode video | → `reset` (after encoding complete) |
| `reset` | Prepare next episode | → `run` (after reset_time_s) |
| `stop` | Stop recording and save | Remains |
| `finish` | Complete recording, HF upload | → Complete |

#### Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `record` | images, state, action | bool | Record frame (run state machine) |
| `save` | - | - | Save current episode |
| `create_frame` | images, state, action | dict | Create recording frame |
| `record_stop` | - | - | Stop recording |
| `record_finish` | - | - | Finish recording |
| `re_record` | - | - | Re-record current episode |
| `record_skip_task` | - | - | Skip current task |
| `record_next_episode` | - | - | Move to next episode |
| `get_current_record_status` | - | TaskStatus | Return current recording status |
| `check_lerobot_dataset` | images, joint_list | bool | Create/load dataset |
| `convert_msgs_to_raw_datas` | image_msgs, follower_msgs, ... | tuple | ROS2 messages → raw data |

#### Static Methods (HuggingFace)
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `get_huggingface_user_id` | - | List[str] | Get authenticated HF user ID list |
| `register_huggingface_token` | hf_token | bool | Register and verify HF token |
| `download_huggingface_repo` | repo_id, repo_type | result/bool | Download dataset/model |
| `upload_huggingface_repo` | repo_id, repo_type, local_dir | bool | Upload dataset/model |
| `delete_huggingface_repo` | repo_id, repo_type | bool | Delete repository |
| `get_huggingface_repo_list` | author, data_type | List[str] | Get repository list |

#### Constants
| Name | Value | Description |
|------|-------|-------------|
| `RAM_LIMIT_GB` | 2 | RAM limit (GB), auto-save when exceeded |
| `SKIP_TIME` | 0.1 | Wait time on task skip (seconds) |

---

### DataEditor
**File**: `data_editor.py`

LeRobot format dataset editing (merge, delete).

#### Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `merge_datasets` | dataset_paths, output_dir, chunk_name | MergeResult | Merge multiple datasets |
| `delete_episode` | dataset_dir, episode_index, chunk_name | DeleteResult | Delete single episode |
| `delete_episodes_batch` | dataset_dir, episode_indices, chunk_name | BatchDeleteResult | Batch episode delete (10x faster) |
| `get_dataset_info` | dataset_dir | dict | Get dataset metadata |

#### Merge Process
1. Build task mapping (`episodes.jsonl` based)
2. Copy parquet files and update indices
3. Merge metadata files (`info.json`, `episodes.jsonl`, `episodes_stats.jsonl`)
4. Copy and rename video files

#### Delete Process (Batch)
1. Create episode_mapping (old_idx → new_idx)
2. Delete parquet/video/image folders
3. Batch rename remaining files
4. Update metadata

#### Data Classes
```python
@dataclass
class MergeResult:
    output_dir: Path
    total_parquet_processed: int
    total_episodes: int | str
    dataset_episode_counts: List[int]

@dataclass
class DeleteResult:
    dataset_dir: Path
    deleted_episode: int
    frames_removed: int
    videos_removed: int
    success: bool

@dataclass
class BatchDeleteResult:
    dataset_dir: Path
    deleted_episodes: List[int]
    total_frames_removed: int
    total_videos_removed: int
    success: bool
```

---

### HfApiWorker
**File**: `hf_api_worker.py`

Worker that executes HuggingFace API operations in a separate process.

#### Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `start` | - | bool | Start worker process |
| `stop` | timeout | - | Stop worker process |
| `is_alive` | - | bool | Check process alive |
| `send_request` | request_data | bool | Send API request |
| `get_result` | block, timeout | tuple/None | Receive operation result |
| `check_task_status` | - | dict | Check current task status |
| `is_busy` | - | bool | Check if busy |

#### Request Format
```python
{
    'mode': 'upload' | 'download' | 'delete' | 'get_dataset_list' | 'get_model_list',
    'repo_id': 'user/repo_name',
    'repo_type': 'dataset' | 'model',
    'local_dir': '/path/to/local',
    'author': 'username'  # for list operations
}
```

#### Status Response
```python
{
    'operation': 'upload' | 'download' | 'delete' | ...,
    'status': 'Idle' | 'Uploading' | 'Downloading' | 'Success' | 'Failed',
    'repo_id': 'user/repo_name',
    'local_path': '/path/to/local',
    'message': 'status message',
    'progress': {
        'current': 50,
        'total': 100,
        'percentage': 50.0
    }
}
```

---

### HuggingFaceProgressTqdm
**File**: `progress_tracker.py`

Custom tqdm for HuggingFace download progress tracking.

#### Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `update` | n | - | Update progress and send to queue |

#### Progress Info
```python
{
    'current': 50,
    'total': 100,
    'percentage': 50.0,
    'is_downloading': True
}
```

---

### HuggingFaceLogCapture
**File**: `progress_tracker.py`

HuggingFace upload log capture and progress parsing.

#### Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `write` | text | int | Capture stdout and parse progress |
| `_parse_and_send_progress` | line | - | Parse upload progress |

#### Upload Progress Parsing
- **Hashing**: 30% weight
- **Pre-upload**: 40% weight
- **Commit**: 30% weight

---

## Dependencies

### Internal
| Module | Usage |
|--------|-------|
| `device_manager` | CPUChecker, RAMChecker, StorageChecker |
| `utils.file_utils` | FileIO |
| `lerobot_dataset_wrapper` | LeRobotDatasetWrapper |

### External
| Package | Components |
|---------|------------|
| `lerobot` | datasets, DEFAULT_FEATURES |
| `huggingface_hub` | HfApi, snapshot_download, upload_large_folder |
| `pandas` | DataFrame (parquet processing) |
| `cv2` | Image processing |
| `torch` | Tensor conversion |

---

## Usage Examples

### Data Recording
```python
from physical_ai_server.data_processing.data_manager import DataManager

# Create manager
manager = DataManager(
    save_root_path=Path('/data'),
    robot_type='aloha',
    task_info=task_info
)

# Prepare dataset
manager.check_lerobot_dataset(images, joint_list)

# Recording loop
while True:
    result = manager.record(images, state, action)
    if result == DataManager.RECORD_COMPLETED:
        break
```

### Dataset Merge
```python
from physical_ai_server.data_processing.data_editor import DataEditor

editor = DataEditor(verbose=True)
result = editor.merge_datasets(
    dataset_paths=['/data/ds1', '/data/ds2'],
    output_dir='/data/merged'
)
print(f'Total episodes: {result.total_episodes}')
```

### HuggingFace Upload (Async)
```python
from physical_ai_server.data_processing.hf_api_worker import HfApiWorker

worker = HfApiWorker()
worker.start()

worker.send_request({
    'mode': 'upload',
    'repo_id': 'user/dataset_name',
    'repo_type': 'dataset',
    'local_dir': '/path/to/dataset'
})

# Check progress
while worker.is_busy():
    status = worker.check_task_status()
    print(f"Progress: {status['progress']['percentage']}%")
    time.sleep(1)

worker.stop()
```

---

## Notes
- Auto-save episode when RAM usage falls below 2GB
- `delete_episodes_batch` is 10x faster than single delete
- HuggingFace operations run in separate process to prevent main thread blocking
- `.cache` folder auto-deleted on upload
