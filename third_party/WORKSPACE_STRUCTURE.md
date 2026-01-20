# Third-Party Model Workspace Structure

## Overview

This document defines the standard workspace structure for all opensource AI model integrations in Physical AI Tools. All models (LeRobot, OpenVLA, GR00T, etc.) MUST follow this structure.

## Standard Directory Structure

```
third_party/<model_name>/
├── <model_name>/           # Original repository (git submodule) ← SUBMODULE HERE
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

## Git Submodule Rules (CRITICAL)

### Submodule Path Convention

**CORRECT**: Submodule은 반드시 `third_party/<model>/<model>/` 경로에 추가

```bash
# CORRECT - submodule은 nested 경로에
git submodule add https://github.com/huggingface/lerobot.git third_party/lerobot/lerobot

# .gitmodules 결과:
[submodule "third_party/lerobot/lerobot"]
    path = third_party/lerobot/lerobot
    url = https://github.com/huggingface/lerobot.git
```

**WRONG**: Submodule을 `third_party/<model>/`에 직접 추가하면 안됨

```bash
# WRONG - 이렇게 하면 우리 통합 코드(executor.py 등)를 넣을 공간이 없음
git submodule add https://github.com/huggingface/lerobot.git third_party/lerobot
```

### 파일 소유권 구분

| 경로 | 소유권 | 설명 |
|------|--------|------|
| `third_party/<model>/` | **우리 코드** | executor.py, Dockerfile, workspace/ 등 |
| `third_party/<model>/<model>/` | **외부 레포** | 원본 오픈소스 코드 (submodule, 수정 금지) |

### Submodule 추가 절차

1. **먼저 통합 폴더 생성**:
   ```bash
   mkdir -p third_party/<model>/workspace/{checkpoints,datasets,outputs}
   ```

2. **그 다음 submodule 추가**:
   ```bash
   git submodule add <repo_url> third_party/<model>/<model>
   ```

3. **통합 코드 작성**:
   ```bash
   # 이 파일들은 우리가 작성/관리
   third_party/<model>/executor.py
   third_party/<model>/Dockerfile
   third_party/<model>/entrypoint.sh
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
