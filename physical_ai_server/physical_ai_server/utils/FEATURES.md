# Utils Module - FEATURES

## Overview
File I/O, file browser, and ROS2 parameter management utility module.

---

## Classes & Functions

### FileIO / file_utils
**File**: `file_utils.py`

JSON/JSONL file I/O and directory management utilities.

#### Functions
| Function | Parameters | Returns | Description |
|----------|------------|---------|-------------|
| `read_json` | file_path, default, silent | dict/None | Read JSON file (returns default on error) |
| `write_json` | file_path, data, indent | bool | Write JSON file |
| `read_jsonl` | path | List[dict] | Read JSONL file (line-by-line JSON) |
| `write_jsonl` | objs, path | bool | Write JSONL file |
| `safe_mkdir` | path | Path | Create directory (parents=True, exist_ok=True) |
| `read_json_file` | file_path | dict/None | Legacy API (read_json wrapper) |

#### FileIO Class
```python
class FileIO:
    read_json = staticmethod(read_json)
    write_json = staticmethod(write_json)
    read_jsonl = staticmethod(read_jsonl)
    write_jsonl = staticmethod(write_jsonl)
    safe_mkdir = staticmethod(safe_mkdir)
```

#### Usage
```python
from physical_ai_server.utils.file_utils import FileIO, read_json, write_json

# JSON read/write
config = read_json('/path/to/config.json', default={})
write_json('/path/to/output.json', {'key': 'value'})

# JSONL read/write
episodes = FileIO.read_jsonl('/path/to/episodes.jsonl')
FileIO.write_jsonl(episodes, '/path/to/new_episodes.jsonl')

# Create directory
FileIO.safe_mkdir('/path/to/new/directory')
```

---

### FileBrowseUtils
**File**: `file_browse_utils.py`

Parallel processing-based file system browser.

#### Attributes
| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_workers` | int | 4 | ThreadPoolExecutor worker count |
| `logger` | Logger | None | Logger object |

#### Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `handle_get_path_action` | current_path | dict | Return current path info |
| `handle_go_parent_action` | current_path | dict | Move to parent directory |
| `handle_go_parent_with_target_check` | current_path, target_files, target_folders | dict | Go parent + target check |
| `handle_browse_action` | current_path, target_name | dict | Browse directory/file |
| `handle_browse_with_target_check` | current_path, target_name, target_files, target_folders | dict | Browse + parallel target check |

#### Response Format
```python
{
    'success': bool,
    'message': str,
    'current_path': str,
    'parent_path': str,
    'selected_path': str,
    'items': [
        {
            'name': 'folder_name',
            'full_path': '/full/path',
            'is_directory': True,
            'size': -1,  # -1 for directories
            'modified_time': '2025-01-01 12:00:00',
            'has_target_file': True  # Only with target_check
        }
    ]
}
```

#### Usage
```python
from physical_ai_server.utils.file_browse_utils import FileBrowseUtils

browser = FileBrowseUtils(max_workers=4)

# Basic browse
result = browser.handle_browse_action('/home/user', 'Downloads')

# Browse with target file check (find dataset folders)
result = browser.handle_browse_with_target_check(
    current_path='/home/user/.cache/huggingface/lerobot',
    target_name=None,
    target_files={'info.json'},
    target_folders={'meta', 'videos'}
)

# Filter valid dataset folders by has_target_file
valid_datasets = [
    item for item in result['items']
    if item['is_directory'] and item.get('has_target_file', False)
]
```

---

### parameter_utils
**File**: `parameter_utils.py`

ROS2 node parameter management utilities.

#### Functions
| Function | Parameters | Returns | Description |
|----------|------------|---------|-------------|
| `declare_parameters` | node, robot_type, param_names, default_value | None | Declare parameters |
| `load_parameters` | node, robot_type, param_names | dict | Load parameters |
| `log_parameters` | node, params, log_level | None | Log parameters |
| `parse_topic_list_with_names` | topic_list | dict | 'name:/topic' → {name: topic} |
| `parse_topic_list` | topic_list | List[str] | 'name:/topic' → [topic] |

#### Topic List Formats
```python
# Format 1: 'name:/topic/path'
topic_list = ['cam1:/camera/image', 'arm:/joint_states']

# parse_topic_list_with_names
result = {'cam1': '/camera/image', 'arm': '/joint_states'}

# parse_topic_list
result = ['/camera/image', '/joint_states']

# Format 2: '/topic/path' (direct)
topic_list = ['/camera/image', '/joint_states']

# parse_topic_list
result = ['/camera/image', '/joint_states']
```

#### Usage
```python
from physical_ai_server.utils.parameter_utils import (
    declare_parameters,
    load_parameters,
    parse_topic_list_with_names
)

# Declare and load parameters
declare_parameters(node, 'aloha', ['fps', 'camera_topic_list'], default_value=[])
params = load_parameters(node, 'aloha', ['fps', 'camera_topic_list'])

# Parse topic list
camera_topics = parse_topic_list_with_names(params['camera_topic_list'])
# {'cam1': '/camera1/image', 'cam2': '/camera2/image'}
```

---

## Dependencies

### External
| Package | Components |
|---------|------------|
| `pathlib` | Path object |
| `json` | JSON parsing |
| `concurrent.futures` | ThreadPoolExecutor |
| `os` | File system operations |
| `rclpy` | Node, parameters |

---

## Notes
- `read_json` returns `default` on file not found/parse error (no exception)
- `FileBrowseUtils` excludes hidden files (except `.cache`)
- `_check_directories_for_targets` runs directory checks in parallel (I/O bound)
- `parse_topic_list_with_names` uses ':' as name/topic separator
