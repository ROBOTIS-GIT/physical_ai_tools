# Physical AI Interfaces - FEATURES

## Overview
ROS2 message and service definition package. Communication interface between physical_ai_server and UI.

---

## Messages (.msg)

### TaskInfo
**File**: `msg/TaskInfo.msg`

Recording task configuration information.

| Field | Type | Description |
|-------|------|-------------|
| `user_id` | string | HuggingFace user ID |
| `task_name` | string | Task name |
| `task_instruction` | string[] | Task instruction list |
| `num_episodes` | uint16 | Total episode count |
| `warmup_time_s` | uint16 | Warmup time (seconds) |
| `episode_time_s` | uint16 | Episode time (seconds) |
| `reset_time_s` | uint16 | Reset time (seconds) |
| `fps` | uint8 | Recording FPS |
| `push_to_hub` | bool | Upload to HuggingFace |
| `private_mode` | bool | Private mode |
| `tags` | string[] | Dataset tags |
| `record_rosbag2` | bool | ROSbag2 recording |
| `use_optimized_save_mode` | bool | Optimized save mode |

---

### TaskStatus
**File**: `msg/TaskStatus.msg`

Current recording status information.

| Constant | Value | Description |
|----------|-------|-------------|
| `READY` | 0 | Ready state |
| `WARMING_UP` | 1 | Warming up |
| `RESETTING` | 2 | Resetting |
| `RECORDING` | 3 | Recording |
| `SAVING` | 4 | Saving |
| `STOPPED` | 5 | Stopped |
| `INFERENCING` | 6 | Inferencing |

| Field | Type | Description |
|-------|------|-------------|
| `task_info` | TaskInfo | Task configuration |
| `robot_type` | string | Robot type |
| `phase` | uint8 | Current phase |
| `total_time` | uint16 | Total time (seconds) |
| `proceed_time` | uint16 | Elapsed time (seconds) |
| `current_episode_number` | uint16 | Current episode number |
| `current_scenario_number` | uint16 | Current scenario number |
| `current_task_instruction` | string | Current task instruction |
| `encoding_progress` | float32 | Encoding progress (%) |
| `used_storage_size` | float32 | Used storage (GB) |
| `total_storage_size` | float32 | Total storage (GB) |
| `used_cpu` | float32 | CPU usage (%) |
| `used_ram_size` | float32 | Used RAM (GB) |
| `total_ram_size` | float32 | Total RAM (GB) |
| `error` | string | Error message |

---

### TrainingInfo
**File**: `msg/TrainingInfo.msg`

Training configuration information.

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `dataset` | string | - | Dataset repo_id |
| `policy_type` | string | - | Policy type |
| `output_folder_name` | string | - | Output folder name |
| `policy_device` | string | cuda | Device |
| `seed` | uint32 | 1000 | Random seed |
| `num_workers` | uint8 | 4 | Worker count |
| `batch_size` | uint16 | 8 | Batch size |
| `steps` | uint32 | 100000 | Total steps |
| `eval_freq` | uint32 | 20000 | Evaluation frequency |
| `log_freq` | uint32 | 200 | Logging frequency |
| `save_freq` | uint32 | 1000 | Save frequency |

---

### TrainingStatus
**File**: `msg/TrainingStatus.msg`

Training progress status.

| Field | Type | Description |
|-------|------|-------------|
| `training_info` | TrainingInfo | Training configuration |
| `current_step` | uint32 | Current step |
| `current_loss` | float32 | Current loss |

---

### DatasetInfo
**File**: `msg/DatasetInfo.msg`

Dataset metadata.

| Field | Type | Description |
|-------|------|-------------|
| `repo_id` | string | Repository ID |
| `num_episodes` | uint32 | Episode count |
| `num_frames` | uint32 | Frame count |
| `fps` | uint8 | FPS |
| `robot_type` | string | Robot type |

---

### HFOperationStatus
**File**: `msg/HFOperationStatus.msg`

HuggingFace operation status.

| Field | Type | Description |
|-------|------|-------------|
| `operation` | string | Operation type (upload/download/delete) |
| `status` | string | Status (Idle/Uploading/Success/Failed) |
| `repo_id` | string | Repository ID |
| `local_path` | string | Local path |
| `message` | string | Message |
| `progress_current` | uint32 | Current progress |
| `progress_total` | uint32 | Total progress |
| `progress_percentage` | float32 | Progress (%) |

---

### BrowserItem
**File**: `msg/BrowserItem.msg`

File browser item.

| Field | Type | Description |
|-------|------|-------------|
| `name` | string | File/folder name |
| `full_path` | string | Full path |
| `is_directory` | bool | Is directory |
| `size` | int64 | Size (bytes, -1 for directories) |
| `modified_time` | string | Modified time |
| `has_target_file` | bool | Has target file |

---

## Services (.srv)

### SendCommand
**File**: `srv/SendCommand.srv`

Recording/inference control commands.

| Constant | Value | Description |
|----------|-------|-------------|
| `IDLE` | 0 | Idle |
| `START_RECORD` | 1 | Start recording |
| `START_INFERENCE` | 2 | Start inference |
| `STOP` | 3 | Stop |
| `MOVE_TO_NEXT` | 4 | Next episode |
| `RERECORD` | 5 | Re-record |
| `FINISH` | 6 | Finish |
| `SKIP_TASK` | 7 | Skip task |

**Request**
| Field | Type | Description |
|-------|------|-------------|
| `command` | uint8 | Command code |
| `task_info` | TaskInfo | Task configuration |

**Response**
| Field | Type | Description |
|-------|------|-------------|
| `success` | bool | Success |
| `message` | string | Message |

---

### SendTrainingCommand
**File**: `srv/SendTrainingCommand.srv`

Training control commands.

**Request**
| Field | Type | Description |
|-------|------|-------------|
| `command` | string | Command (start/stop/resume) |
| `training_info` | TrainingInfo | Training configuration |
| `resume_model_path` | string | Model path for resume |

**Response**
| Field | Type | Description |
|-------|------|-------------|
| `success` | bool | Success |
| `message` | string | Message |

---

### BrowseFile
**File**: `srv/BrowseFile.srv`

File browser service.

**Request**
| Field | Type | Description |
|-------|------|-------------|
| `action` | string | Action (browse/go_parent/get_path) |
| `current_path` | string | Current path |
| `target_name` | string | Target name |
| `target_files` | string[] | Target file list |
| `target_folders` | string[] | Target folder list |

**Response**
| Field | Type | Description |
|-------|------|-------------|
| `success` | bool | Success |
| `message` | string | Message |
| `current_path` | string | Current path |
| `parent_path` | string | Parent path |
| `selected_path` | string | Selected path |
| `items` | BrowserItem[] | Item list |

---

### EditDataset
**File**: `srv/EditDataset.srv`

Dataset editing service.

**Request**
| Field | Type | Description |
|-------|------|-------------|
| `action` | string | Action (merge/delete) |
| `dataset_paths` | string[] | Dataset path list |
| `output_path` | string | Output path |
| `episode_indices` | int32[] | Episode index list |

**Response**
| Field | Type | Description |
|-------|------|-------------|
| `success` | bool | Success |
| `message` | string | Message |

---

### ControlHfServer
**File**: `srv/ControlHfServer.srv`

HuggingFace server control.

**Request**
| Field | Type | Description |
|-------|------|-------------|
| `mode` | string | Mode (upload/download/delete/get_list) |
| `repo_id` | string | Repository ID |
| `repo_type` | string | Type (dataset/model) |
| `local_dir` | string | Local directory |
| `author` | string | Author |

**Response**
| Field | Type | Description |
|-------|------|-------------|
| `success` | bool | Success |
| `message` | string | Message |
| `repo_list` | string[] | Repository list |

---

### Other Services

| Service | Description |
|---------|-------------|
| `GetDatasetInfo` | Get dataset info |
| `GetDatasetList` | Get local dataset list |
| `GetImageTopicList` | Get camera topic list |
| `GetPolicyList` | Get available policy list |
| `GetSavedPolicyList` | Get saved policy list |
| `GetModelWeightList` | Get model weight list |
| `GetTrainingInfo` | Get current training info |
| `GetRobotTypeList` | Get robot type list |
| `GetUserList` | Get HuggingFace user list |
| `GetHFUser` | Get current HF user |
| `SetHFUser` | Set HF user |
| `SetRobotType` | Set robot type |

---

## Notes
- All messages/services built from `physical_ai_interfaces` package
- Python: `from physical_ai_interfaces.msg import TaskStatus`
- C++: `#include "physical_ai_interfaces/msg/task_status.hpp"`
