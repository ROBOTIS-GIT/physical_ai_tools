# Physical AI Tools - System Architecture

## Overview

Physical AI Tools is a ROS2-based open-source platform that supports End-to-End Physical AI workflow from robot data collection to model training and inference.

---

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Physical AI Tools                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │   React UI      │    │  Physical AI    │    │   ROSbag        │         │
│  │   (Manager)     │◄──►│  Server (ROS2)  │◄──►│   Recorder      │         │
│  │                 │    │                 │    │   (C++)         │         │
│  └─────────────────┘    └────────┬────────┘    └─────────────────┘         │
│                                  │                                          │
│          ┌───────────────────────┼───────────────────────┐                 │
│          │                       │                       │                 │
│          ▼                       ▼                       ▼                 │
│  ┌───────────────┐    ┌──────────────────┐    ┌──────────────────┐        │
│  │ Data          │    │ Training         │    │ Inference        │        │
│  │ Processing    │    │ Module           │    │ Module           │        │
│  └───────────────┘    └──────────────────┘    └──────────────────┘        │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                          Zenoh / ZMQ Protocol
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      Docker Containers (Model Backends)                      │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌───────────┐  ┌───────────┐  ┌───────────┐  ┌───────────┐               │
│  │  LeRobot  │  │  OpenVLA  │  │ GR00T N1  │  │  Future   │               │
│  │           │  │           │  │           │  │  Models   │               │
│  └───────────┘  └───────────┘  └───────────┘  └───────────┘               │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Package Structure

```
physical_ai_tools/
├── physical_ai_interfaces/    # ROS2 message/service definitions
├── physical_ai_server/        # Core server (Python ROS2 node)
├── physical_ai_manager/       # React web UI
├── rosbag_recorder/           # ROSbag recording (C++ node)
└── docker/                    # Model container Dockerfiles
```

---

## Core Modules

### 1. Physical AI Server (ROS2 Python)

The main server that orchestrates all functionalities.

| Module | Responsibility | Agent |
|--------|---------------|-------|
| **communication** | ROS2 topic subscribe/publish, service clients | Data Collector |
| **data_processing** | Data recording, conversion, editing, HuggingFace integration | Data Collector |
| **training** | LeRobot model training management | Training Expert |
| **inference** | Real-time inference, ZMQ server | Inference/Deploy |
| **evaluation** | Model evaluation, visualization | Training Expert |
| **video_encoder** | Image → MP4 encoding | Data Collector |
| **device_manager** | CPU/RAM/Storage monitoring | System Architect |
| **utils** | File I/O, parameter management | General |

**Details**: See each module's `FEATURES.md`

---

### 2. ROSbag Recorder (ROS2 C++)

High-performance ROSbag recording node.

| Feature | Description |
|---------|-------------|
| MCAP Format | Uses MCAP format for enterprise compatibility |
| Image Compression | Image topics → Real-time MP4 compression |
| Generic Subscription | Dynamic subscription for all message types |

**Details**: See `rosbag_recorder/FEATURES.md`

---

### 3. Physical AI Manager (React UI)

Web-based user interface.

| Page | Function |
|------|----------|
| HomePage | Robot type selection, HuggingFace login |
| RecordPage | Data recording |
| TrainingPage | Model training |
| InferencePage | Real-time inference |
| EditDatasetPage | Dataset editing |

**Details**: See `physical_ai_manager/FEATURES.md`

---

### 4. Physical AI Interfaces (ROS2 msgs/srvs)

Communication interfaces between system components.

| Category | Examples |
|----------|----------|
| Messages | TaskStatus, TrainingStatus, DatasetInfo |
| Services | SendCommand, SendTrainingCommand, BrowseFile |

**Details**: See `physical_ai_interfaces/FEATURES.md`

---

## Core Workflows

### Workflow 1: Data Collection

```
┌──────────┐    ┌──────────────┐    ┌─────────────┐    ┌────────────┐
│  React   │───►│ physical_ai  │───►│  rosbag     │───►│   MCAP     │
│  UI      │    │ _server      │    │  _recorder  │    │  + MP4     │
│ (Record) │    │(DataManager) │    │  (C++)      │    │  Files     │
└──────────┘    └──────────────┘    └─────────────┘    └────────────┘
```

**Related Agents**: Data Collector, ROSbag Data Manager

---

### Workflow 2: Training

```
┌──────────┐    ┌──────────────┐    ┌─────────────┐    ┌────────────┐
│  React   │───►│ physical_ai  │───►│  LeRobot    │───►│  Trained   │
│  UI      │    │ _server      │    │  Container  │    │  Model     │
│(Training)│    │(TrainingMgr) │    │  (Docker)   │    │  Weights   │
└──────────┘    └──────────────┘    └─────────────┘    └────────────┘
```

**Related Agents**: Training Expert, Docker Model Manager

---

### Workflow 3: Inference

```
┌──────────┐    ┌──────────────┐    ┌─────────────┐    ┌────────────┐
│  React   │───►│ physical_ai  │───►│  Inference  │───►│   Robot    │
│  UI      │    │ _server      │    │  Module     │    │  Action    │
│(Inference│    │(InferenceMgr)│    │  (ZMQ)      │    │  Commands  │
└──────────┘    └──────────────┘    └─────────────┘    └────────────┘
```

**Related Agents**: Inference/Deploy, Model Integrator

---

### Workflow 4: Dataset Management

```
┌──────────┐    ┌──────────────┐    ┌─────────────┐
│  React   │───►│ physical_ai  │───►│ HuggingFace │
│  UI      │    │ _server      │    │    Hub      │
│(EditData)│    │(DataEditor)  │    │             │
└──────────┘    │(HfApiWorker) │    └─────────────┘
                └──────────────┘
```

**Related Agents**: Data Collector, Third-party Manager

---

## Communication Protocols

| Protocol | Usage | Components |
|----------|-------|------------|
| **ROS2 Topics** | Real-time sensor data, status publishing | Server ↔ Recorder |
| **ROS2 Services** | Command control, configuration queries | UI ↔ Server |
| **rosbridge WebSocket** | Web UI connection | React ↔ ROS2 |
| **ZMQ** | Inference server communication | Server ↔ Inference |
| **Zenoh** | Docker container communication | Server ↔ Containers |

---

## React ↔ Physical AI Server Communication

### Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         React UI (Browser)                               │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐         │
│  │ Redux Store     │  │ React Hooks     │  │ roslib.js       │         │
│  │ (State Mgmt)    │  │ (ROS Wrappers)  │  │ (WebSocket)     │         │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘         │
│           │                    │                    │                   │
│           └────────────────────┼────────────────────┘                   │
│                                │                                        │
└────────────────────────────────┼────────────────────────────────────────┘
                                 │ WebSocket (ws://localhost:9090)
                                 ▼
┌────────────────────────────────────────────────────────────────────────┐
│                       rosbridge_server (ROS2)                           │
│                    WebSocket ↔ ROS2 Protocol Bridge                     │
└────────────────────────────────┬───────────────────────────────────────┘
                                 │ ROS2 Topics/Services
                                 ▼
┌────────────────────────────────────────────────────────────────────────┐
│                     Physical AI Server (ROS2 Node)                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                  │
│  │ Topic        │  │ Service      │  │ Modules      │                  │
│  │ Publishers   │  │ Servers      │  │ (Processing) │                  │
│  └──────────────┘  └──────────────┘  └──────────────┘                  │
└────────────────────────────────────────────────────────────────────────┘
```

### Communication Flow

#### 1. Topic Subscription (Real-time Status Reception)

```
React                    rosbridge                Physical AI Server
  │                          │                            │
  │  Subscribe(/task/status) │                            │
  │─────────────────────────►│                            │
  │                          │  ROS2 Subscribe            │
  │                          │───────────────────────────►│
  │                          │                            │
  │                          │     TaskStatus message     │
  │                          │◄───────────────────────────│
  │   WebSocket message      │                            │
  │◄─────────────────────────│                            │
  │                          │                            │
  │  Redux: setTaskStatus()  │                            │
  │                          │                            │
```

**Subscribed Topics**:
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/task/status` | TaskStatus | Recording status, progress |
| `/training/status` | TrainingStatus | Training status, loss |
| `/heartbeat` | Empty | Server connection check |

#### 2. Service Call (Command Transmission)

```
React                    rosbridge                Physical AI Server
  │                          │                            │
  │  Call /send_command      │                            │
  │  {command: START_RECORD} │                            │
  │─────────────────────────►│                            │
  │                          │  ROS2 Service Call         │
  │                          │───────────────────────────►│
  │                          │                            │
  │                          │  {success: true}           │
  │                          │◄───────────────────────────│
  │   WebSocket response     │                            │
  │◄─────────────────────────│                            │
  │                          │                            │
```

**Service List**:
| Service | Type | Description |
|---------|------|-------------|
| `/send_command` | SendCommand | Recording/inference control |
| `/training/send_command` | SendTrainingCommand | Training control |
| `/browse_file` | BrowseFile | File system browsing |
| `/dataset/edit` | EditDataset | Dataset editing |
| `/control_hf_server` | ControlHfServer | HuggingFace operations |

### React Implementation

#### Hooks (ROS2 Communication Wrappers)

```javascript
// useRosTopicSubscription.js - Topic subscription
useEffect(() => {
  const topic = new ROSLIB.Topic({
    ros: rosConnection,
    name: '/task/status',
    messageType: 'physical_ai_interfaces/msg/TaskStatus'
  });
  topic.subscribe((message) => {
    dispatch(setTaskStatus(message));
  });
}, []);

// useRosServiceCaller.js - Service call
const callService = (serviceName, request) => {
  const service = new ROSLIB.Service({
    ros: rosConnection,
    name: serviceName,
    serviceType: 'physical_ai_interfaces/srv/SendCommand'
  });
  return new Promise((resolve) => {
    service.callService(request, resolve);
  });
};
```

#### Redux Store (State Management)

```javascript
// Topic message → Redux state
{
  tasks: {
    taskStatus: { phase, current_episode, ... },  // /task/status
    taskInfo: { ... }
  },
  training: {
    trainingStatus: { current_step, current_loss, ... },  // /training/status
  },
  ros: {
    connected: true,  // Based on /heartbeat
    rosHost: 'ws://localhost:9090'
  }
}
```

### Connection Setup

```bash
# 1. Launch rosbridge server (together with Physical AI Server)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 2. Connect from React app
rosConnectionManager.connect('ws://localhost:9090');
```

**Default Port**: `9090` (rosbridge WebSocket)

---

## Agent Mapping

Responsible agents for each functional area.

| Area | Primary Agent | Secondary Agent |
|------|--------------|-----------------|
| Data Collection | Data Collector | ROSbag Data Manager |
| Data Augmentation | Data Augmentor | - |
| Model Training | Training Expert | Docker Model Manager |
| Inference/Deployment | Inference/Deploy | Model Integrator |
| UI Development | UI/Frontend | - |
| System Design | System Architect | - |
| Code Quality | Code Quality | - |
| Container Management | Docker Model Manager | - |
| Third-party Management | Third-party Manager | - |

---

## Development Guidelines

### Adding New Feature

1. **Define Interface**: Add msg/srv to `physical_ai_interfaces/`
2. **Implement Server**: Implement module in `physical_ai_server/`
3. **Integrate UI**: Add component to `physical_ai_manager/`
4. **Update Documentation**: Update the module's `FEATURES.md`

### Adding New Model

1. **Docker Setup**: Add Dockerfile to `docker/` folder
2. **Zenoh Client**: Implement Zenoh communication in container
3. **Integration Test**: Test with Model Integrator Agent

---

## File References

| Document | Location | Description |
|----------|----------|-------------|
| Project Config | `/CLAUDE.md` | Overall project configuration |
| communication | `physical_ai_server/.../communication/FEATURES.md` | Communication module |
| data_processing | `physical_ai_server/.../data_processing/FEATURES.md` | Data processing |
| training | `physical_ai_server/.../training/FEATURES.md` | Training module |
| inference | `physical_ai_server/.../inference/FEATURES.md` | Inference module |
| evaluation | `physical_ai_server/.../evaluation/FEATURES.md` | Evaluation module |
| video_encoder | `physical_ai_server/.../video_encoder/FEATURES.md` | Video encoding |
| device_manager | `physical_ai_server/.../device_manager/FEATURES.md` | Device management |
| utils | `physical_ai_server/.../utils/FEATURES.md` | Utilities |
| rosbag_recorder | `rosbag_recorder/FEATURES.md` | ROSbag recording |
| interfaces | `physical_ai_interfaces/FEATURES.md` | ROS2 interfaces |
| manager | `physical_ai_manager/FEATURES.md` | React UI |

---

## Tech Stack Summary

| Category | Technology |
|----------|------------|
| Robot Framework | ROS2 Jazzy |
| Communication | Zenoh (rmw_zenoh), ZMQ, rosbridge |
| Data Format | MCAP, MP4, LeRobot Dataset |
| ML Framework | PyTorch, LeRobot |
| Container | Docker, Docker Compose |
| UI | React 19, Redux Toolkit, TailwindCSS |
| Languages | Python, C++, JavaScript |
