# Replay Viewer Refactoring Progress

**Date**: 2026-01-12  
**Branch**: feature-1.0.0

---

## ✅ Completed Work

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

### 5. ReplayPage.js Cleanup

- Original functionality preserved (2711 lines)
- ~96 lines of duplicate code removed via `chartUtils.js` import
- All features verified working

---

## 📁 Created Files (Not Yet Integrated)

The following files were created but not yet integrated into ReplayPage.js:

### Custom Hooks

| File | Description | Lines |
|------|-------------|-------|
| `hooks/useHybridVideoLoader.js` | Hybrid video loading strategy | ~260 |
| `hooks/useVideoSync.js` | Multi-video synchronization | ~280 |
| `hooks/useKeyboardShortcuts.js` | Keyboard shortcut management | ~190 |

### Replay Components

| File | Description | Lines |
|------|-------------|-------|
| `components/replay/VideoPlayer.js` | Video player component | ~165 |
| `components/replay/PlaybackControls.js` | Playback control UI | ~300 |
| `components/replay/RosbagInfoPanel.js` | ROSbag info panel | ~110 |
| `components/replay/JointChart.js` | Joint chart component | ~270 |
| `components/replay/index.js` | Component re-exports | ~12 |

---

## 🔧 Remaining Work (Gradual Integration)

### Phase 1: Component Integration (Low Risk)

1. **JointChart Separation**
   - Extract `JointChart` component from ReplayPage
   - Use `components/replay/JointChart.js`

2. **RosbagInfoPanel Separation**
   - Extract Metadata panel as separate component
   - Use `components/replay/RosbagInfoPanel.js`

### Phase 2: Hook Integration (Medium Risk)

3. **useKeyboardShortcuts Integration**
   - Move keyboard shortcut logic to Hook
   - Ref-based approach to prevent stale closures

4. **useVideoSync Integration**
   - Separate video synchronization logic
   - Includes play/pause, seeking, A-B loop

### Phase 3: Advanced Features (Higher Risk)

5. **useHybridVideoLoader Integration**
   - Streaming + background download strategy
   - Replaces existing video download logic

6. **VideoPlayer & PlaybackControls Integration**
   - Separate video rendering and control UI
   - Final step in refactoring

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

| Item | Before | After |
|------|--------|-------|
| ReplayPage.js | 2800 lines | 2711 lines (-89 lines) |
| New files | 0 | 10 files |
| Reusable code | 0 | ~1600 lines |
| Unit tests | None | chartUtils tests complete |
