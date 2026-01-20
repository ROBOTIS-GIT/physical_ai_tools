# Third-Party Model Workspace Structure

## Overview

This document defines the standard workspace structure for all opensource AI model integrations in Physical AI Tools. All models (LeRobot, OpenVLA, GR00T, etc.) MUST follow this structure.

## Standard Directory Structure

```
third_party/<model_name>/
├── <model_name>/           # Original repository (git submodule)
├── executor.py             # Zenoh-based executor for ROS2 communication
├── Dockerfile              # Container build definition
├── entrypoint.sh           # Container entrypoint script
├── README.md               # Integration documentation
├── requirements.txt        # Additional Python dependencies (if needed)
└── workspace/              # Runtime data directory (gitignored)
    ├── checkpoints/        # Trained model weights
    ├── datasets/           # Training/evaluation datasets
    └── outputs/            # Logs, tensorboard, misc outputs
```

## Workspace Directory Details

### `/workspace/checkpoints/`
- **Purpose**: Store trained model weights and checkpoints
- **Container path**: `/workspace/checkpoints`
- **Contents**: 
  - `*.safetensors`, `*.pt`, `*.pth`, `*.ckpt` files
  - Checkpoint directories with `config.json` and model files
- **Naming convention**: `<policy>_<dataset>_<timestamp>/`

### `/workspace/datasets/`
- **Purpose**: Store local datasets for training/evaluation
- **Container path**: `/workspace/datasets`
- **Contents**:
  - LeRobot format datasets
  - Converted rosbag data
  - Downloaded HuggingFace datasets (optional cache)
- **Naming convention**: `<user>/<dataset_name>/`

### `/workspace/outputs/`
- **Purpose**: Store training logs and miscellaneous outputs
- **Container path**: `/workspace/outputs`
- **Contents**:
  - TensorBoard logs
  - Training metrics CSV/JSON
  - Evaluation results

## Docker Volume Mapping

In `docker-compose.yml`, each model service MUST map workspace:

```yaml
services:
  <model_name>:
    volumes:
      # Workspace mapping (REQUIRED)
      - ../third_party/<model_name>/workspace:/workspace
      
      # Zenoh SDK (REQUIRED for communication)
      - ../third_party/zenoh_ros2_sdk:/zenoh_sdk:ro
      
      # HuggingFace cache (optional, for model downloads)
      - ./huggingface:/root/.cache/huggingface
    
    environment:
      # Standard workspace environment variable
      - MODEL_WORKSPACE=/workspace
      - CHECKPOINT_DIR=/workspace/checkpoints
      - DATASET_DIR=/workspace/datasets
      - OUTPUT_DIR=/workspace/outputs
```

## Environment Variables

Each executor MUST respect these environment variables:

| Variable | Default | Description |
|----------|---------|-------------|
| `MODEL_WORKSPACE` | `/workspace` | Root workspace directory |
| `CHECKPOINT_DIR` | `/workspace/checkpoints` | Model checkpoint storage |
| `DATASET_DIR` | `/workspace/datasets` | Dataset storage |
| `OUTPUT_DIR` | `/workspace/outputs` | Output/log storage |

## Git Ignore Rules

Each workspace MUST have a `.gitignore` file that:
1. Ignores all runtime data (checkpoints, datasets, outputs)
2. Keeps directory structure with `.gitkeep` files
3. Ignores large binary files (*.safetensors, *.pt, etc.)

Example `.gitignore`:
```
# Ignore all content
checkpoints/
datasets/
outputs/

# Keep directory structure
!checkpoints/.gitkeep
!datasets/.gitkeep
!outputs/.gitkeep

# Large binary files
*.safetensors
*.pt
*.pth
*.ckpt
*.bin
```

## Adding a New Model

When integrating a new opensource model:

1. **Create directory structure**:
   ```bash
   mkdir -p third_party/<model_name>/workspace/{checkpoints,datasets,outputs}
   touch third_party/<model_name>/workspace/{checkpoints,datasets,outputs}/.gitkeep
   ```

2. **Add git submodule** (if using original repo):
   ```bash
   git submodule add <repo_url> third_party/<model_name>/<model_name>
   ```

3. **Create workspace .gitignore**:
   Copy from existing model (e.g., `third_party/lerobot/workspace/.gitignore`)

4. **Update docker-compose.yml**:
   Add service with proper volume mappings

5. **Implement executor.py**:
   Follow the standard executor pattern with Zenoh communication

## Examples

### LeRobot
```
third_party/lerobot/
├── lerobot/              # HuggingFace LeRobot submodule
├── executor.py           # LeRobotExecutor class
├── Dockerfile
├── entrypoint.sh
└── workspace/
    ├── checkpoints/      # ACT, Diffusion, etc. model weights
    ├── datasets/         # pusht, aloha_sim, etc.
    └── outputs/          # Training logs
```

### OpenVLA (future)
```
third_party/openvla/
├── openvla/              # OpenVLA submodule
├── executor.py           # OpenVLAExecutor class
├── Dockerfile
├── entrypoint.sh
└── workspace/
    ├── checkpoints/
    ├── datasets/
    └── outputs/
```

### GR00T (future)
```
third_party/groot/
├── Isaac-GR00T/          # NVIDIA GR00T submodule
├── executor.py           # GR00TExecutor class
├── Dockerfile
├── entrypoint.sh
└── workspace/
    ├── checkpoints/
    ├── datasets/
    └── outputs/
```

## Best Practices

1. **Never commit binary files**: All model weights stay in workspace (gitignored)
2. **Use environment variables**: Don't hardcode paths in executor.py
3. **Consistent naming**: Follow `<policy>_<dataset>_<timestamp>` for checkpoints
4. **Document paths**: Update README.md with model-specific path requirements
5. **Test volume mounts**: Verify workspace is writable from container
