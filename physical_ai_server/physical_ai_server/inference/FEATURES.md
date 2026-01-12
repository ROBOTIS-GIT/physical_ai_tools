# Inference Module - FEATURES

## Overview
Module responsible for loading trained policy models and real-time inference. Supports ZMQ-based server/client architecture.

---

## Classes

### InferenceManager
**File**: `inference_manager.py`

Policy model loading and inference manager.

#### Supported Policies
| Policy | Class | Description |
|--------|-------|-------------|
| `tdmpc` | TDMPCPolicy | TD-MPC |
| `diffusion` | DiffusionPolicy | Diffusion Policy |
| `act` | ACTPolicy | Action Chunking Transformer |
| `vqbet` | VQBeTPolicy | VQ-BeT |
| `pi0` | PI0Policy | Pi0 policy |
| `pi0fast` | PI0FASTPolicy | Pi0 Fast |
| `smolvla` | SmolVLAPolicy | SmolVLA |

#### Attributes
| Attribute | Type | Description |
|-----------|------|-------------|
| `device` | str | Inference device (cuda/cpu) |
| `policy_type` | str | Loaded policy type |
| `policy_path` | str | Policy file path |
| `policy` | PreTrainedPolicy | Loaded policy model |

#### Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `validate_policy` | policy_path | tuple[bool, str] | Validate policy path and config.json |
| `load_policy` | - | bool | Load policy model |
| `clear_policy` | - | - | Release policy memory |
| `get_policy_config` | - | dict | Return policy configuration |
| `predict` | images, state, task_instruction | list | Observation → action prediction |
| `get_available_policies` (static) | - | list[str] | Get supported policy list |
| `get_saved_policies` (static) | - | tuple[list, list] | Get cached policy paths/types |

#### Predict Input Format
```python
images = {
    'cam1': np.ndarray,  # [H, W, C], RGB, uint8 or float32
    'cam2': np.ndarray
}
state = [0.1, 0.2, ...]  # Joint state list
task_instruction = "pick up the red cube"  # Optional, for VLA models
```

#### Predict Output Format
```python
action = [0.1, 0.2, ...]  # Action array (numpy)
```

---

### ServerInference
**File**: `server_inference.py`

ZeroMQ REP socket-based inference server.

#### Attributes
| Attribute | Type | Description |
|-----------|------|-------------|
| `inference_manager` | InferenceManager | Inference manager |
| `running` | bool | Server running state |
| `context` | zmq.Context | ZMQ context |
| `socket` | zmq.Socket | REP socket |

#### Endpoints
| Endpoint | Input | Output | Description |
|----------|-------|--------|-------------|
| `ping` | - | `{status, message}` | Check server status |
| `kill` | - | - | Shutdown server |
| `get_action` | `{images, state, task_instruction}` | action | Predict action |
| `get_modality_config` | - | config | Get policy configuration |

#### Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `run` | - | - | Run server loop |
| `register_endpoint` | name, handler, requires_input | - | Register endpoint |

#### Request Format
```python
{
    'endpoint': 'get_action',  # Default
    'data': {
        'images': {...},
        'state': [...],
        'task_instruction': 'pick up red cube'
    }
}
```

---

### TorchSerializer
**File**: `server_inference.py`

ZMQ message serialization utility.

#### Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `to_bytes` (static) | data: dict | bytes | Dict → bytes (torch.save) |
| `from_bytes` (static) | data: bytes | dict | bytes → Dict (torch.load) |

---

## Dependencies

### Internal
| Module | Usage |
|--------|-------|
| `utils.file_utils` | read_json_file |

### External
| Package | Components |
|---------|------------|
| `lerobot` | PreTrainedPolicy, policy classes |
| `torch` | Tensor operations, inference_mode |
| `zmq` | ZeroMQ server |
| `numpy` | Array processing |

---

## Usage Examples

### Direct Inference
```python
from physical_ai_server.inference.inference_manager import InferenceManager

manager = InferenceManager(device='cuda')

# Validate and load policy
success, msg = manager.validate_policy('/path/to/policy')
if success:
    manager.load_policy()

    # Inference
    action = manager.predict(
        images={'cam1': img_array},
        state=[0.1, 0.2, 0.3],
        task_instruction='pick up the cube'
    )
    print(action)
```

### Run ZMQ Server
```python
from physical_ai_server.inference.server_inference import ServerInference

server = ServerInference(
    policy_type='act',
    policy_path='/path/to/policy',
    device='cuda',
    server_address='0.0.0.0',
    port=5555
)
server.run()  # Blocking
```

### ZMQ Client Request
```python
import zmq
from server_inference import TorchSerializer

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect('tcp://localhost:5555')

# Ping request
request = {'endpoint': 'ping'}
socket.send(TorchSerializer.to_bytes(request))
response = TorchSerializer.from_bytes(socket.recv())
print(response)  # {'status': 'ok', 'message': 'Server is running'}

# Inference request
request = {
    'endpoint': 'get_action',
    'data': {
        'images': {'cam1': image_array},
        'state': [0.1, 0.2, 0.3]
    }
}
socket.send(TorchSerializer.to_bytes(request))
action = TorchSerializer.from_bytes(socket.recv())
```

---

## Policy Storage Locations

```
# HuggingFace cache (downloaded models)
~/.cache/huggingface/hub/models--<user>--<model>/
└── snapshots/
    └── <commit_hash>/
        └── pretrained_model/
            ├── config.json
            ├── model.safetensors
            └── train_config.json

# Local training output
lerobot/outputs/train/<output_folder>/
└── pretrained_model/
    ├── config.json
    ├── model.safetensors
    └── train_config.json
```

---

## Notes
- `predict()` runs inside `torch.inference_mode()`
- Images normalized from [0, 255] → [0, 1] then converted to [C, H, W] format
- VLA models (smolvla, etc.) require `task_instruction` parameter
- GR00T N1 support is in TODO state
