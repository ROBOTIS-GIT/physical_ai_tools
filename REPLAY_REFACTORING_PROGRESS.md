# Replay Viewer Refactoring Progress

**Date**: 2026-01-12  
**Branch**: feature/replay-refactoring (based on feature-1.0.0)

---

## ✅ Completed Work (2026-01-12 Session 3)

### Bug Fix - Korean IME Keyboard Shortcuts

- **Problem**: Keyboard shortcuts (S, E, M, A, etc.) didn't work with Korean input mode
- **Solution**: Changed `e.key` to `e.code` for physical key detection
- **Affected keys**: A, S, E, M, D, X and digit keys (1-9)

---

## ✅ Completed Work (2026-01-12 Session 2)

### Backend Refactoring - Module Separation

New modules created in `physical_ai_server/data_processing/`:

| File | Description | Lines |
|------|-------------|-------|
| `bag_reader.py` | ROSbag file reader (MCAP/SQLite3) | 238 |
| `metadata_manager.py` | Metadata CRUD (robot_config.yaml) | 315 |
| `video_metadata_extractor.py` | Video metadata extraction | 215 |
| `replay_data_handler.py` | Main orchestrator (refactored) | 513 |

**Benefits:**
- Single responsibility per module
- Reusable across other projects
- Easier testing and debugging
- Total: 1,281 lines (modularized from 700-line monolith)

### Frontend Refactoring - JointChart Integration

- **ReplayPage.js**: 2711 → 2490 lines (-221 lines)
- JointChart component now imported from `components/replay/`
- Removed duplicate component definition from ReplayPage.js

### Analysis: PlaybackControls Component

- **Decision**: Keep inline in ReplayPage.js
- **Reason**: Tight coupling with video refs, dispatch, and 20+ callbacks
- **Benefit**: Avoiding excessive prop drilling maintains code clarity

---

## ✅ Previously Completed Work

### 1. Utility Function Extraction (`chartUtils.js`)

**File**: `physical_ai_manager/src/utils/chartUtils.js`

Extracted functions:
- `lttbDownsample()` - LTTB downsampling algorithm
- `prepareChartData()` - Chart data preparation
- `formatFileSize()` - File size formatting
- `formatDateTime()` - Date/time formatting
- `formatTime()` - Time formatting (MM:SS)

### 2. Unit Tests (`chartUtils.test.js`)

**File**: `physical_ai_manager/src/utils/chartUtils.test.js`

- Jest tests for all utility functions completed
- Run with: `npm test -- --testPathPattern=chartUtils`

### 3. Port Configuration Changes

| Service | Before | After | File |
|---------|--------|-------|------|
| VideoFileServer | - | 8082 | `physical_ai_server.py` |
| web_video_server (ROS) | 8081 | 8085 | `physical_ai_server_bringup.launch.py` |
| getReplayData | 8081 | 8082 | `useRosServiceCaller.js` |
| getRosbagList | 8081 | 8082 | `useRosServiceCaller.js` |

### 4. API Response Field Addition

**File**: `physical_ai_manager/src/hooks/useRosServiceCaller.js`

Added missing fields to `getReplayData` return object:
- `task_markers`
- `robot_type`, `recording_date`, `file_size_bytes`
- `trim_points`, `exclude_regions`, `frame_counts`

---

## 📁 Available Components (Ready for Integration)

### Custom Hooks

| File | Description | Lines |
|------|-------------|-------|
| `hooks/useHybridVideoLoader.js` | Hybrid video loading strategy | ~260 |
| `hooks/useVideoSync.js` | Multi-video synchronization | ~280 |
| `hooks/useKeyboardShortcuts.js` | Keyboard shortcut management | ~190 |

### Replay Components

| File | Description | Status |
|------|-------------|--------|
| `components/replay/JointChart.js` | Joint chart component | ✅ Integrated |
| `components/replay/VideoPlayer.js` | Video player component | Available |
| `components/replay/PlaybackControls.js` | Playback control UI | ⏸️ Deferred (see analysis) |
| `components/replay/RosbagInfoPanel.js` | ROSbag info panel | Available |
| `components/replay/index.js` | Component re-exports | ✅ Updated |

---

## 🔧 Remaining Work (Low Priority)

### Frontend Integration (Optional)

1. **useKeyboardShortcuts Integration**
   - Move keyboard shortcut logic to Hook
   - Already using `e.code` for IME compatibility

2. **useVideoSync Integration**
   - Separate video synchronization logic
   - Includes play/pause, seeking, A-B loop

### Backend Tests

3. **Unit Tests for New Modules**
   - Test files created: `test_metadata_manager.py`, `test_video_metadata_extractor.py`
   - Run in Docker container with ROS2 environment

---

## 📝 Testing Instructions

```bash
# 1. Start server (inside container)
cd /home/dongyun/main_ws/physical_ai_tools/docker
./container.sh enter
ai_server

# 2. Build frontend
cd /home/dongyun/main_ws/physical_ai_tools/docker
docker compose build physical_ai_manager
docker compose up -d physical_ai_manager

# 3. Run unit tests
cd /home/dongyun/main_ws/physical_ai_tools/physical_ai_manager
npm test -- --testPathPattern=chartUtils

# 4. Browser testing
# Visit http://localhost:8080 and test Replay Viewer
```

---

## ⚠️ Important Notes

1. **Keep ai_server running**: Required for Replay Viewer functionality
2. **Port separation**: VideoFileServer (8082) and web_video_server (8085) use different ports
3. **Browser cache**: Changes may not appear immediately → Force refresh (Ctrl+Shift+R)

---

## 📊 Code Change Summary

| Item | Before | After | Change |
|------|--------|-------|--------|
| ReplayPage.js | 2711 lines | ~2490 lines | -221 lines |
| Backend (replay_data_handler) | 700 lines | 513 lines | -187 lines |
| New Backend modules | 0 | 3 files (768 lines) | +768 lines |
| New Frontend components | 0 | 5 files (~1000 lines) | Reusable |
| Backend unit tests | 0 | 2 files | test_metadata_manager, test_video_metadata_extractor |
| Keyboard shortcuts | e.key (EN only) | e.code (IME compatible) | Korean input fix |
