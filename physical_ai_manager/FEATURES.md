# Physical AI Manager (React UI) - FEATURES

## Overview
React-based Physical AI Tools web UI. Real-time topic subscription and service calls via ROS2 rosbridge.

---

## Pages

### HomePage
**File**: `pages/HomePage.js`

Initial setup page (robot type selection, HuggingFace login).

| Feature | Description |
|---------|-------------|
| Robot Type Selector | Robot type selection dropdown |
| HuggingFace Login | HF token input and verification |
| Connection Status | ROS2 connection status display |

---

### RecordPage
**File**: `pages/RecordPage.js`

Data recording page.

| Component | Description |
|-----------|-------------|
| ImageGrid | Real-time camera image display |
| ControlPanel | Recording control buttons (Start/Stop/Finish) |
| TaskStatus | Current recording status display |
| EpisodeStatus | Episode progress status |
| SystemStatus | CPU/RAM/Storage monitoring |
| InfoPanel | Task settings panel |

| Feature | Description |
|---------|-------------|
| Task Instruction Input | Task instruction list input |
| Episode Settings | Episode count, time settings |
| Tag Input | Dataset tag input |
| Image Topic Selection | Camera topic selection modal |

---

### TrainingPage
**File**: `pages/TrainingPage.js`

Model training page.

| Component | Description |
|-----------|-------------|
| DatasetSelector | Dataset selection (local/HuggingFace) |
| PolicySelector | Policy type selection (ACT, Diffusion, etc.) |
| TrainingControlPanel | Training control (Start/Stop/Resume) |
| TrainingProgressBar | Training progress display |
| TrainingLossDisplay | Current loss display |
| TrainingOptionInput | Training option settings (batch_size, steps, etc.) |
| ResumePolicySelector | Checkpoint selection for resume |

---

### InferencePage
**File**: `pages/InferencePage.js`

Real-time inference page.

| Component | Description |
|-----------|-------------|
| ImageGrid | Camera image display |
| InferencePanel | Inference control panel |
| PolicyDownloadModal | Policy download modal |
| ModelWeightSelector | Model weight selection |

---

### EditDatasetPage
**File**: `pages/EditDatasetPage.js`

Dataset editing page.

| Section | Description |
|---------|-------------|
| DatasetMergeSection | Merge multiple datasets |
| DatasetDeleteSection | Delete episodes |
| DatasetHuggingfaceSection | HF upload/download/delete |

---

## Components

### Image & Camera
| Component | File | Description |
|-----------|------|-------------|
| ImageGrid | `ImageGrid.js` | Camera image grid layout |
| ImageGridCell | `ImageGridCell.js` | Individual camera image cell |
| ImageTopicSelectModal | `ImageTopicSelectModal.js` | Camera topic selection modal |

### Status Display
| Component | File | Description |
|-----------|------|-------------|
| TaskStatus / FullTaskStatus | `FullTaskStatus.js` | Full task status |
| EpisodeStatus | `EpisodeStatus.js` | Episode progress status |
| SystemStatus | `SystemStatus.js` | System resource status |
| CompactSystemStatus | `CompactSystemStatus.js` | Compact system status |
| HeartbeatStatus | `HeartbeatStatus.js` | ROS2 connection heartbeat |
| ProgressBar | `ProgressBar.js` | General progress bar |

### Control Panels
| Component | File | Description |
|-----------|------|-------------|
| ControlPanel | `ControlPanel.js` | Recording control panel |
| InferencePanel | `InferencePanel.js` | Inference control panel |
| TrainingControlPanel | `TrainingControlPanel.js` | Training control panel |
| InfoPanel | `InfoPanel.js` | Task info panel |

### Selectors
| Component | File | Description |
|-----------|------|-------------|
| RobotTypeSelector | `RobotTypeSelector.js` | Robot type selection |
| DatasetSelector | `DatasetSelector.js` | Dataset selection |
| PolicySelector | `PolicySelector.js` | Policy type selection |
| ModelWeightSelector | `ModelWeightSelector.js` | Model weight selection |
| TaskSelector | `TaskSelector.js` | Task selection |
| ResumePolicySelector | `ResumePolicySelector.js` | Resume policy selection |

### File Browser
| Component | File | Description |
|-----------|------|-------------|
| FileBrowser | `FileBrowser.js` | File system browser |
| FileBrowserModal | `FileBrowserModal.js` | Modal file browser |

### Input Components
| Component | File | Description |
|-----------|------|-------------|
| TaskInstructionInput | `TaskInstructionInput.js` | Task instruction input |
| TagInput | `TagInput.js` | Tag input |
| TrainingOptionInput | `TrainingOptionInput.js` | Training option input |
| TrainingOutputFolderInput | `TrainingOutputFolderInput.js` | Output folder input |
| TokenInputPopup | `TokenInputPopup.js` | HF token input popup |

### Training Display
| Component | File | Description |
|-----------|------|-------------|
| TrainingProgressBar | `TrainingProgressBar.js` | Training progress |
| TrainingLossDisplay | `TrainingLossDisplay.js` | Loss display |

### Modals
| Component | File | Description |
|-----------|------|-------------|
| PolicyDownloadModal | `PolicyDownloadModal.js` | Policy download |

### Utility
| Component | File | Description |
|-----------|------|-------------|
| Tooltip | `Tooltip.js` | Tooltip component |

---

## Redux Store (State Management)

### Slices
| Slice | File | Description |
|-------|------|-------------|
| `tasks` | `taskSlice.js` | Recording task state, TaskInfo, TaskStatus |
| `ros` | `rosSlice.js` | ROS2 connection state, host settings |
| `ui` | `uiSlice.js` | Current page, UI state |
| `training` | `trainingSlice.js` | Training state, TrainingInfo, TrainingStatus |
| `editDataset` | `editDatasetSlice.js` | Dataset editing state |

### State Structure
```javascript
{
  tasks: {
    taskStatus: { /* TaskStatus message */ },
    taskInfo: { /* TaskInfo message */ },
    topicReceived: boolean
  },
  ros: {
    rosHost: string,
    connected: boolean
  },
  ui: {
    currentPage: PageType
  },
  training: {
    trainingInfo: { /* TrainingInfo */ },
    trainingStatus: { /* TrainingStatus */ },
    topicReceived: boolean
  },
  editDataset: {
    selectedDatasets: [],
    hfOperationStatus: { /* HFOperationStatus */ }
  }
}
```

---

## Hooks

### useRosTopicSubscription
**File**: `hooks/useRosTopicSubscription.js`

ROS2 topic subscription management hook.

| Subscribed Topics | Type | Description |
|------------------|------|-------------|
| `/task/status` | TaskStatus | Recording status |
| `/training/status` | TrainingStatus | Training status |
| `/heartbeat` | Empty | Server heartbeat |

---

### useRosServiceCaller
**File**: `hooks/useRosServiceCaller.js`

ROS2 service call hook.

| Service | Type | Description |
|---------|------|-------------|
| `/send_command` | SendCommand | Recording/inference control |
| `/training/send_command` | SendTrainingCommand | Training control |
| `/browse_file` | BrowseFile | File browser |
| `/dataset/edit` | EditDataset | Dataset editing |
| `/control_hf_server` | ControlHfServer | HuggingFace control |

---

## Utils

### rosConnectionManager
**File**: `utils/rosConnectionManager.js`

WebSocket-based rosbridge connection management.

| Method | Description |
|--------|-------------|
| `connect(host)` | Connect to rosbridge |
| `disconnect()` | Disconnect |
| `isConnected()` | Check connection status |
| `setOnConnected(callback)` | Set connection callback |
| `getRos()` | Return ROSLIB.Ros object |

---

## Constants

### PageType
**File**: `constants/pageType.js`

| Constant | Value | Description |
|----------|-------|-------------|
| `HOME` | 'home' | Home page |
| `RECORD` | 'record' | Recording page |
| `TRAINING` | 'training' | Training page |
| `INFERENCE` | 'inference' | Inference page |
| `EDIT_DATASET` | 'edit_dataset' | Dataset editing |

### taskPhases
**File**: `constants/taskPhases.js`

| Constant | Value | Description |
|----------|-------|-------------|
| `READY` | 0 | Ready |
| `WARMING_UP` | 1 | Warming up |
| `RESETTING` | 2 | Resetting |
| `RECORDING` | 3 | Recording |
| `SAVING` | 4 | Saving |
| `STOPPED` | 5 | Stopped |
| `INFERENCING` | 6 | Inferencing |

### taskCommand
**File**: `constants/taskCommand.js`

Recording control command constants.

### trainingCommand
**File**: `constants/trainingCommand.js`

Training control command constants.

---

## Dependencies

| Package | Version | Description |
|---------|---------|-------------|
| `react` | 19.1 | React framework |
| `react-dom` | 19.1 | React DOM rendering |
| `@reduxjs/toolkit` | 2.8 | Redux state management |
| `react-redux` | 9.2 | React-Redux bindings |
| `roslib` | 1.4 | ROS2 WebSocket communication |
| `react-icons` | 5.5 | Icon components |
| `react-hot-toast` | 2.5 | Toast notifications |
| `clsx` | - | Conditional class utility |
| `tailwindcss` | 3.4 | CSS framework |

---

## Development

### Run
```bash
cd physical_ai_manager
npm install
npm start
```

### Build
```bash
npm run build
```

### Environment Variables
| Variable | Description |
|----------|-------------|
| `REACT_APP_DEBUG` | Debug mode (true/false) |

---

## Notes
- rosbridge WebSocket port: 9090 (default)
- Camera images subscribe to ROS2 CompressedImage topics
- Auto page transition during training
- Recording/inference/training pages blocked if robot type not selected
