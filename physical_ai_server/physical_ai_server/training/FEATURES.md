# Training Module - FEATURES

## Overview
LeRobot-based Imitation Learning policy training pipeline management module.

---

## Classes

### TrainingManager
**File**: `training_manager.py`

Training pipeline configuration and execution manager. Supports resume functionality.

#### Supported Policies
| Policy | Trainer | Description |
|--------|---------|-------------|
| `pi0` | LerobotTrainer | Pi0 policy |
| `pi0fast` | LerobotTrainer | Pi0 Fast policy |
| `diffusion` | LerobotTrainer | Diffusion Policy |
| `act` | LerobotTrainer | ACT (Action Chunking Transformer) |
| `tdmpc` | LerobotTrainer | TD-MPC |
| `vqbet` | LerobotTrainer | VQ-BeT |
| `smolvla` | LerobotTrainer | SmolVLA |

#### Attributes
| Attribute | Type | Description |
|-----------|------|-------------|
| `training_info` | TrainingInfo | Training configuration message |
| `trainer` | Trainer | Currently active trainer |
| `cfg` | TrainPipelineConfig | draccus configuration object |
| `stop_event` | threading.Event | Training stop event |
| `resume` | bool | Resume mode flag |
| `resume_model_path` | str | Model path for resume |

#### Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `train` | - | - | Execute training pipeline |
| `get_current_training_status` | - | TrainingStatus | Get current training status (step, loss) |
| `get_available_list` (static) | - | tuple[list, list] | Get supported policy/device list |
| `get_weight_save_root_path` (static) | - | Path | Get checkpoint save path |

#### Training Parameters (TrainingInfo)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `policy_type` | - | Policy type (pi0, act, diffusion, etc.) |
| `policy_device` | cuda | Device (cuda/cpu) |
| `dataset` | - | HuggingFace dataset repo_id |
| `output_folder_name` | - | Output folder name |
| `seed` | 1000 | Random seed |
| `num_workers` | 4 | DataLoader worker count |
| `batch_size` | 8 | Batch size |
| `steps` | 100000 | Total training steps |
| `eval_freq` | 20000 | Evaluation frequency |
| `log_freq` | 200 | Logging frequency |
| `save_freq` | 1000 | Checkpoint save frequency |

---

### Trainer (Abstract)
**File**: `trainers/trainer.py`

Abstract base class for all trainers.

#### Abstract Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `train` | cfg, stop_event | - | Execute training |

---

### LerobotTrainer
**File**: `trainers/lerobot/lerobot_trainer.py`

LeRobot framework-based training implementation. Supports resume and mixed precision.

#### Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `train` | cfg, stop_event | - | Execute training pipeline |
| `get_current_step` | - | int | Get current training step |
| `get_current_loss` | - | float | Get current loss value |

#### Training Pipeline Steps
1. **Setup**: Working directory setup, configuration preparation
2. **Validation**: Configuration validation (skipped in resume mode)
3. **Logging Setup**: WandB or local logging setup
4. **Dataset Creation**: Create training dataset
5. **Policy Creation**: Create policy model
6. **Optimizer Setup**: Create optimizer, scheduler, GradScaler
7. **State Loading**: Load previous state on resume
8. **Training Loop**: Execute main training loop
9. **Evaluation**: Periodic evaluation (if eval_env exists)
10. **Checkpointing**: Save checkpoints

#### Training Loop Features
| Feature | Description |
|---------|-------------|
| **Mixed Precision** | AMP (Automatic Mixed Precision) support |
| **Gradient Clipping** | grad_clip_norm based gradient clipping |
| **EpisodeAwareSampler** | Sampler for drop_n_last_frames supporting policies |
| **MetricsTracker** | Track loss, grad_norm, lr, update_s, dataloading_s |
| **WandB Integration** | Experiment tracking and video logging |
| **Graceful Stop** | Safe stop via stop_event |

#### Resume Functionality
```python
# States loaded on resume
- step: Current training step
- optimizer_state: Optimizer state
- lr_scheduler_state: Scheduler state
- model_weights: Based on train_config.json
```

---

## Directory Structure

```
training/
├── __init__.py
├── training_manager.py          # TrainingManager
├── FEATURES.md                  # This file
└── trainers/
    ├── __init__.py
    ├── trainer.py               # Abstract Trainer
    ├── lerobot/
    │   └── lerobot_trainer.py   # LeRobot training implementation
    ├── gr00tn1/
    │   └── gr00tn1_trainer.py   # GR00T N1 (TODO)
    └── openvla/
        └── openvla_trainer.py   # OpenVLA (TODO)
```

---

## Dependencies

### Internal
| Module | Usage |
|--------|-------|
| `physical_ai_interfaces` | TrainingInfo, TrainingStatus |

### External
| Package | Components |
|---------|------------|
| `lerobot` | TrainPipelineConfig, make_dataset, make_policy, make_env, PreTrainedPolicy |
| `draccus` | Configuration parsing |
| `torch` | Optimizer, GradScaler, DataLoader |
| `wandb` (optional) | WandBLogger |

---

## Usage Examples

### Start New Training
```python
from physical_ai_server.training.training_manager import TrainingManager
from physical_ai_interfaces.msg import TrainingInfo

manager = TrainingManager()

# Configure training
manager.training_info = TrainingInfo()
manager.training_info.policy_type = 'act'
manager.training_info.policy_device = 'cuda'
manager.training_info.dataset = 'user/my_dataset'
manager.training_info.output_folder_name = 'my_training'
manager.training_info.batch_size = 8
manager.training_info.steps = 100000

# Run training
manager.train()
```

### Resume Training
```python
manager = TrainingManager()
manager.training_info.policy_type = 'act'
manager.resume = True
manager.resume_model_path = 'user/my_training/checkpoints/100000'

# Change steps for additional training
manager.training_info.steps = 200000

manager.train()
```

### Stop Training
```python
import threading

manager = TrainingManager()
stop_event = threading.Event()
manager.stop_event = stop_event

# Start training in separate thread
thread = threading.Thread(target=manager.train)
thread.start()

# Stop signal
stop_event.set()
thread.join()
```

### Check Training Status
```python
status = manager.get_current_training_status()
print(f"Step: {status.current_step}")
print(f"Loss: {status.current_loss}")
```

---

## Output Structure

```
outputs/train/<output_folder_name>/
├── train_config.json             # Training configuration
├── pretrained_model/             # Latest checkpoint
│   ├── config.json
│   ├── model.safetensors
│   └── train_config.json
├── checkpoints/
│   ├── 001000/                   # Per-step checkpoints
│   ├── 002000/
│   └── ...
└── eval/
    └── videos_step_XXXXXX/       # Evaluation videos
```

---

## Notes
- In resume mode, policy configuration auto-loads from config.json
- Logs saved to local filesystem when WandB disabled
- EpisodeAwareSampler only used for policies supporting drop_n_last_frames
- GR00T N1, OpenVLA trainers are in TODO state
