# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Physical AI Tools is a ROS2-based platform for end-to-end physical AI development. It supports the complete workflow from robot data collection through model training to real-time inference deployment. The system integrates with HuggingFace Hub for dataset and model management.

**Technology Stack:**
- **Robot Framework:** ROS2 Jazzy with Zenoh (rmw_zenoh_cpp)
- **Frontend:** React 19, Redux Toolkit, TailwindCSS, roslib.js
- **Backend:** Python (ROS2 nodes), C++17 (high-performance recording)
- **ML Training:** LeRobot, GR00T N1 (in Docker containers)
- **Communication:** ROS2 topics/services, rosbridge WebSocket, Zenoh, ZMQ
- **Data Formats:** MCAP (rosbags), MP4 (video), LeRobot dataset format
- **Containerization:** Docker Compose, multi-arch (AMD64/ARM64)

**Three Main Components:**
1. **physical_ai_server** - Python ROS2 node (main orchestrator)
2. **physical_ai_manager** - React web UI for control and monitoring
3. **rosbag_recorder** - C++ ROS2 node for high-performance data recording

## Essential Commands

### Build Commands

```bash
# ROS2 packages (run from repository root)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Quick rebuild alias (set in Docker container)
cb  # Alias for the above command

# Build specific package
colcon build --packages-select physical_ai_interfaces
colcon build --packages-select rosbag_recorder
colcon build --packages-select physical_ai_server

# Frontend (Node.js)
cd physical_ai_manager
npm install
npm run build  # Production build

# Docker (RECOMMENDED for development)
cd docker
./container.sh start  # Auto-detects AMD64 or ARM64 architecture
```

### Running the System

```bash
# Full system launch (ROS2 native)
ros2 launch physical_ai_server physical_ai_server_bringup.launch.py

# Individual components
ros2 launch physical_ai_server physical_ai_server.launch.py
ros2 launch rosbag_recorder image_bag_recorder.launch.py

# Frontend dev server (port 3000)
cd physical_ai_manager && npm start

# Docker-based (recommended)
cd docker
./container.sh start   # Start all containers
./container.sh enter   # Enter physical_ai_server container shell
./container.sh stop    # Stop all containers

# Inside Docker container
ai_server  # Alias for physical_ai_server_bringup.launch.py
```

### Testing Commands

```bash
# Python unit tests
pytest physical_ai_server/tests/ -v
pytest physical_ai_server/tests/data_processing/ -v

# Python tests with coverage
pytest physical_ai_server/tests/ --cov=physical_ai_server --cov-report=html

# ROS2 package tests
colcon test
colcon test-result --verbose

# Frontend tests
cd physical_ai_manager
npm test
npm test -- --coverage --watchAll=false

# LeRobot end-to-end tests
cd lerobot
make DEVICE=cpu test-end-to-end
make test-act-ete-train
```

### Linting

```bash
# Python (if configured)
flake8 physical_ai_server/physical_ai_server --max-line-length=120

# ROS2 linting (requires ROS environment)
source /opt/ros/jazzy/setup.bash
ament_flake8 physical_ai_server
ament_cpplint rosbag_recorder

# Frontend
cd physical_ai_manager && npm run lint  # If configured
```

## Architecture (Big Picture)

### Multi-Container System

```
┌─────────────────────────────────────────────────────────────────┐
│                     Physical AI Tools System                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐  WebSocket  ┌──────────────┐  ROS2 Topics    │
│  │  React UI    │◄────────────►│ physical_ai  │◄───────────────┐│
│  │  (Manager)   │   :9090      │   _server    │                ││
│  │  Port 3000   │              │   (Python)   │                ││
│  └──────────────┘              └──────┬───────┘                ││
│                                       │                         ││
│           Zenoh/ZMQ                   │ ROS2                    ││
│         ┌─────────┴────────┐          │                         ││
│         ▼                  ▼          ▼                         ││
│  ┌────────────┐    ┌────────────┐  ┌──────────────┐           ││
│  │  LeRobot   │    │   GR00T    │  │   rosbag     │───────────┘│
│  │ Container  │    │ Container  │  │  _recorder   │             │
│  │ (Training) │    │(Inference) │  │    (C++)     │             │
│  └────────────┘    └────────────┘  └──────────────┘             │
│                                                                  │
│         ↓ Model Weights          ↓ MCAP Bags + MP4 Videos       │
│  ┌────────────────┐         ┌──────────────────────┐            │
│  │  HuggingFace   │         │  Datasets (LeRobot   │            │
│  │      Hub       │         │       format)        │            │
│  └────────────────┘         └──────────────────────┘            │
└─────────────────────────────────────────────────────────────────┘
```

### Communication Protocols

- **ROS2 Topics/Services:** Real-time sensor data, status updates, command control
- **rosbridge WebSocket (port 9090):** React UI ↔ ROS2 bridge using roslib.js
- **Zenoh:** ROS2 Server ↔ Docker containers (shared memory IPC for performance)
- **ZMQ:** Training/inference data exchange between server and containers

### Core Workflows

**1. Data Collection:**
```
UI (RecordPage) → Server (DataManager) → Recorder (C++) → MCAP bags + MP4 videos
```

**2. Training:**
```
UI (TrainingPage) → Server (TrainingManager) → LeRobot Container → Trained weights → HF Hub
```

**3. Inference:**
```
UI (InferencePage) → Server (InferenceManager) → ZMQ socket → Model containers → Robot actions
```

**4. Dataset Management:**
```
UI (EditDatasetPage) → Server (DataEditor) → HuggingFace API → Upload/Download/Merge/Delete
```

**5. Replay Viewer:**
```
UI (ReplayPage) → Server (ReplayDataHandler) → MCAP reader + VideoFileServer → Synchronized playback
```

## Important Technical Details

### Zenoh Configuration (CRITICAL)

The system uses Zenoh for high-performance ROS2 communication. Required environment:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=30
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'
```

**Docker Compose Configuration:**
```yaml
network_mode: host  # Required for Zenoh peer discovery
ipc: host           # Required for shared memory transport
environment:
  - RMW_IMPLEMENTATION=rmw_zenoh_cpp
  - ZENOH_SHM_ENABLED=true
```

**Key Point:** All containers and the server must use the same Zenoh configuration to communicate.

### File Formats

- **MCAP:** Enterprise-grade rosbag format (better than SQLite3-based rosbag2)
  - Used by `rosbag_recorder` for all recordings
  - More efficient storage and faster playback

- **MP4:** Video streams encoded via FFmpeg pipelines
  - Real-time encoding during recording
  - Automatic FPS detection from first 10 frames
  - Stored alongside MCAP bags with synchronized timestamps

- **LeRobot Dataset Format:**
  - Parquet files for state/action data
  - MP4 videos for camera observations
  - JSON metadata with episode information

### Submodule Management

**Active Submodules (4):**
1. `lerobot/` - HuggingFace LeRobot (main branch)
2. `third_party/lerobot/lerobot` - ROBOTIS fork (feature-robotis branch)
3. `third_party/groot/Isaac-GR00T` - NVIDIA GR00T
4. `third_party/zenoh_ros2_sdk` - ROBOTIS Zenoh SDK

**Clone with submodules:**
```bash
git clone -b jazzy https://github.com/ROBOTIS-GIT/physical_ai_tools.git --recursive
```

**Update submodules:**
```bash
git submodule update --init --recursive
```

**Known Issue - LIBERO Submodule:**
The LIBERO submodule (nested in Isaac-GR00T) may fail with "Needed a single revision" error.

**Solution:**
```bash
cd third_party/groot/Isaac-GR00T/external_dependencies/LIBERO
git fetch origin
git checkout 8f1084e3132a39270c3a13ebe37270a43ece2a01  # Or the commit referenced
cd ../../../../..
git submodule update --init --recursive
```

### Architecture Detection (AMD64 vs ARM64)

Docker builds automatically detect architecture via `container.sh`:

```bash
# Auto-detection
MACHINE_ARCH=$(uname -m)
# aarch64 or arm64 → ARM64 (Jetson)
# x86_64 → AMD64

# Manual override
ARCH=amd64 docker compose up --build
ARCH=arm64 docker compose up --build
```

**Base Images:**
- AMD64: `robotis/ros:jazzy-ros-base-torch2.7.0-cuda12.8.0`
- ARM64: Same base, but with Jetson-specific pip indices

## Code Structure

### Repository Layout

```
physical_ai_tools/
├── physical_ai_interfaces/    # ROS2 msg/srv definitions (CMake)
│   ├── msg/                   # TaskStatus, TrainingStatus, DatasetInfo, etc.
│   └── srv/                   # SendCommand, TrainModel, GetDatasetList, etc.
│
├── physical_ai_server/        # Main Python ROS2 node (8 modules)
│   ├── physical_ai_server.py  # Entry point (1811 lines)
│   ├── communication/         # ROS2 topics/services, Zenoh client
│   ├── data_processing/       # MCAP, MP4, HuggingFace integration
│   ├── training/              # LeRobot/GR00T trainer orchestration
│   ├── inference/             # ZMQ inference server
│   ├── video_encoder/         # FFmpeg MP4 encoding
│   ├── device_manager/        # CPU/RAM/Storage monitoring
│   ├── evaluation/            # Model evaluation, visualization
│   └── utils/                 # File I/O, parameter management
│
├── physical_ai_manager/       # React web UI
│   ├── src/pages/             # Record, Training, Inference, EditDataset, Replay
│   ├── src/features/          # Redux slices (ros, tasks, training, replay, etc.)
│   ├── src/components/        # Reusable UI components
│   └── src/hooks/             # Custom React hooks
│
├── rosbag_recorder/           # C++ ROS2 recorder
│   ├── src/service_bag_recorder.cpp   # Main state machine
│   ├── src/image_compressor.cpp       # Real-time MP4 encoding
│   └── include/rosbag_recorder/       # Headers
│
├── docker/                    # Docker Compose orchestration
│   ├── docker-compose.yml     # 4 services: manager, server, lerobot, groot
│   └── container.sh           # Helper script
│
├── third_party/               # Submodules
│   ├── lerobot/               # ROBOTIS LeRobot fork
│   ├── groot/                 # NVIDIA GR00T
│   └── zenoh_ros2_sdk/        # Zenoh ROS2 SDK
│
└── tests/                     # Integration tests
```

### Key Files to Understand

**Entry Points:**
- [physical_ai_server/physical_ai_server/physical_ai_server.py](physical_ai_server/physical_ai_server/physical_ai_server.py) - Main ROS2 node
- [physical_ai_manager/src/App.js](physical_ai_manager/src/App.js) - React app root
- [rosbag_recorder/src/service_bag_recorder.cpp](rosbag_recorder/src/service_bag_recorder.cpp) - Recorder state machine

**Configuration:**
- [docker/docker-compose.yml](docker/docker-compose.yml) - Container orchestration
- [physical_ai_server/config/*.yaml](physical_ai_server/config/) - Robot configurations
- [rosbag_recorder/config/recorder_config.yaml](rosbag_recorder/config/recorder_config.yaml) - Recording settings

**Documentation:**
- [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md) - Detailed system design
- [docs/BUILD_TROUBLESHOOTING.md](docs/BUILD_TROUBLESHOOTING.md) - ARM64/Jetson build issues
- Each module has `FEATURES.md` - Component specifications

## Development Workflow

### Recommended: Docker-Based Development

```bash
# Start all containers
cd docker
./container.sh start

# Enter development shell
./container.sh enter

# Inside container - convenient aliases
cb                  # colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
ai_server           # ros2 launch physical_ai_server physical_ai_server_bringup.launch.py
zenoh               # ros2 run rmw_zenoh_cpp rmw_zenohd

# View logs
docker compose logs -f physical_ai_server
docker compose logs -f lerobot
```

**Why Docker?**
- Pre-configured environment with all dependencies
- Consistent builds across AMD64 and ARM64
- Isolated ML containers (LeRobot, GR00T)
- Shared volumes for development

### Frontend Development

```bash
# Terminal 1: Start backend
cd docker && ./container.sh start

# Terminal 2: Start frontend dev server
cd physical_ai_manager
npm install
npm start  # Runs on http://localhost:3000

# Frontend connects to rosbridge at localhost:9090
```

### Native Development (Not Recommended)

Requires manual installation of:
- ROS2 Jazzy
- CUDA 12.8 + cuDNN
- PyTorch 2.7 with CUDA support
- All Python dependencies (mcap, matplotlib, huggingface_hub, etc.)
- Node.js 22 for frontend

**Only use for:**
- Quick testing without Docker
- CI/CD workflows
- Debugging specific ROS2 issues

## Common Issues and Solutions

### ARM64/Jetson Build Issues

**Issue 1: Build context too large / Disk full**

Docker tries to copy entire repository (including 20GB+ model files).

**Solution:** Use `.dockerignore` to exclude:
```
docker/huggingface/
docker/workspace/
third_party/lerobot/lerobot/
third_party/groot/Isaac-GR00T/
**/*.safetensors
**/*.pt
**/*.pth
```

**Additional:** Clean Docker cache if needed:
```bash
docker builder prune -af
```

**Issue 2: matplotlib pip install fails on ARM64**

Jetson pip index doesn't have matplotlib.

**Solution:** Add PyPI as secondary index in `Dockerfile.arm64`:
```dockerfile
RUN pip install matplotlib --extra-index-url https://pypi.org/simple
```

**Issue 3: decord has no ARM64 wheels**

**Solution:** Build from source in `Dockerfile.arm64`:
```dockerfile
RUN git clone --recursive https://github.com/dmlc/decord /tmp/decord && \
    cd /tmp/decord && mkdir build && cd build && \
    cmake .. -DUSE_CUDA=OFF && make -j$(nproc) && \
    pip install /tmp/decord/python && rm -rf /tmp/decord
```

### Git Submodule Issues

**LIBERO submodule fails with "Needed a single revision"**

This happens when the submodule clone is interrupted or incomplete.

**Solution:**
```bash
cd third_party/groot/Isaac-GR00T/external_dependencies/LIBERO
git fetch origin
git checkout <commit-hash>  # Find in parent .gitmodules or git ls-tree
```

### File Permission Issues in Git

**Symptom:** All files show as modified (yellow in VSCode), but `git diff` shows only permission changes.

**Cause:** File permissions changed from 644 to 755 (or vice versa).

**Solution:** Ignore file permission changes:
```bash
git config core.fileMode false
```

### HuggingFace CLI Usage

**Setup:** All Docker containers include HuggingFace CLI installed via `pip install huggingface_hub`.

**Available commands:**
```bash
# Authentication
hf auth login
hf auth whoami

# Dataset operations
hf download lerobot/pusht --repo-type dataset
hf upload username/repo-name /path/to/folder --repo-type dataset

# Legacy alias (backward compatibility)
huggingface-cli auth login  # Same as 'hf auth login'
```

**Environment:** Custom HuggingFace endpoint is pre-configured:
- `HF_ENDPOINT=http://192.168.60.152:1000` (set in all containers)

**Note:** The `hf` command is the official HuggingFace CLI tool installed at `/usr/local/bin/hf`. A backward-compatible `huggingface-cli` alias is also provided for legacy scripts

## Key Files to Modify

### For ROS2 Interface Changes

**Add new messages:**
1. Create `physical_ai_interfaces/msg/YourMessage.msg`
2. Add to `physical_ai_interfaces/CMakeLists.txt`:
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/YourMessage.msg"
     # ... existing messages
   )
   ```
3. Rebuild: `colcon build --packages-select physical_ai_interfaces`
4. Source: `source install/setup.bash`

**Add new services:**
1. Create `physical_ai_interfaces/srv/YourService.srv`
2. Add to CMakeLists.txt (same as messages)
3. Rebuild and source

### For Server Logic Changes

**Main orchestrator:**
- [physical_ai_server/physical_ai_server/physical_ai_server.py](physical_ai_server/physical_ai_server/physical_ai_server.py)

**Module-specific:**
- Data processing: `physical_ai_server/physical_ai_server/data_processing/`
- Training: `physical_ai_server/physical_ai_server/training/`
- Inference: `physical_ai_server/physical_ai_server/inference/`

**After Python changes:**
```bash
# With --symlink-install, no rebuild needed for Python code changes
# Just restart the node
ros2 launch physical_ai_server physical_ai_server.launch.py
```

### For UI Changes

**Pages:** `physical_ai_manager/src/pages/*.js`
- RecordPage, TrainingPage, InferencePage, EditDatasetPage, ReplayPage

**State management:** `physical_ai_manager/src/features/*/`
- Redux slices for ros, tasks, training, replay, etc.

**Components:** `physical_ai_manager/src/components/`
- Reusable UI components

**Hooks:** `physical_ai_manager/src/hooks/`
- `useRosTopicSubscription.js` - Subscribe to ROS topics
- `useRosServiceCaller.js` - Call ROS services
- `useHybridVideoLoader.js` - Load video with fallback

**After React changes:**
- Dev server auto-reloads
- Production: `npm run build` and restart nginx

### For Recording Changes

**Main recorder:** [rosbag_recorder/src/service_bag_recorder.cpp](rosbag_recorder/src/service_bag_recorder.cpp)
- State machine: IDLE → PREPARED → RECORDING → STOPPED → FINISHED

**MP4 encoding:** [rosbag_recorder/src/image_compressor.cpp](rosbag_recorder/src/image_compressor.cpp)
- FFmpeg pipelines, FPS detection

**After C++ changes:**
```bash
colcon build --packages-select rosbag_recorder
source install/setup.bash
ros2 launch rosbag_recorder image_bag_recorder.launch.py
```

## Testing Strategy

### Unit Tests

**Python (pytest):**
```bash
# All tests
pytest physical_ai_server/tests/ -v

# Specific module
pytest physical_ai_server/tests/data_processing/ -v

# With coverage
pytest --cov=physical_ai_server --cov-report=html
```

**React (Jest):**
```bash
cd physical_ai_manager
npm test
npm test -- --coverage --watchAll=false
```

### Integration Tests

**Docker integration:**
```bash
pytest tests/test_lerobot_docker_integration.py -v
```

**Training pipeline:**
```bash
pytest tests/test_training_log_publish.py -v
```

### End-to-End Tests

**LeRobot policies:**
```bash
cd lerobot
make DEVICE=cpu test-end-to-end
make test-act-ete-train
make test-diffusion-ete-eval
```

**Full workflow:**
Requires robot hardware or simulation environment.

### CI/CD

**GitHub Actions:**
- `.github/workflows/ros-ci.yml` - Builds on ROS2 jazzy/rolling
- `.github/workflows/ros-lint.yml` - Linting (cpplint, flake8, pep257, etc.)

**Triggered on:**
- Push to `main` or `jazzy` branches
- Pull requests

## Contributing

All contributions must include DCO sign-off:

```bash
git commit -s -m "Your commit message"
```

This certifies you have the right to submit the code under Apache 2.0 license.

See [CONTRIBUTING.md](CONTRIBUTING.md) for details.

## Additional Resources

- **Documentation:** https://ai.robotis.com/
- **Tutorial Videos:** https://www.youtube.com/@ROBOTISOpenSourceTeam
- **Datasets & Models:** https://huggingface.co/ROBOTIS
- **Docker Images:** https://hub.docker.com/r/robotis/ros/tags
- **AI Worker ROS2 Packages:** https://github.com/ROBOTIS-GIT/ai_worker
- **Simulation Models:** https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie
