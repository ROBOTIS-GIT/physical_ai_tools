/*
 * Copyright 2025 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Kiwoong Park
 */

import { createSlice } from '@reduxjs/toolkit';
import TaskPhase from '../../constants/taskPhases';

const initialState = {
  taskInfo: {
    taskNum: '',
    taskName: '',
    taskType: '',
    taskInstruction: [],
    policyPath: '',
    recordInferenceMode: false,
    controlHz: 100,
    inferenceHz: 15,
    chunkAlignWindowS: 0.3,
  },
  taskStatus: {
    robotType: '',
    taskName: 'idle',
    running: false,
    phase: TaskPhase.READY,
    progress: 0,
    totalTime: 0,
    proceedTime: 0,
    currentEpisodeNumber: 0,
    currentScenarioNumber: 0,
    currentTaskInstruction: '',
    currentSegmentIndex: 0,
    segmentCount: 0,
    segmentPrimitives: [],
    userId: '',
    usedStorageSize: 0,
    totalStorageSize: 0,
    usedCpu: 0,
    usedRamSize: 0,
    totalRamSize: 0,
    error: '',
    topicReceived: false,
  },
  // Pending primitive_description for the NEXT segment to be started.
  // Always pushed as '' now — the legacy primitive selection UI was
  // replaced by a free-text Korean sub_task field.
  pendingPrimitive: '',
  // Pending Korean sub_task text for the NEXT segment to be started.
  pendingSubTask: '',
  // Sub-task plan for the current/next episode. Held in Redux — not local
  // component state — so the plan survives navigating away from the recorder
  // panel and back, matching how taskInfo persists.
  plannedCount: 0,
  plannedSubTasks: [],
  slotToServerIdx: [],
  activeSlotIndex: 0,
  availableRobots: [],
  availableCameras: [],
  policyList: [],
  datasetList: [],
  heartbeatStatus: 'disconnected',
  lastHeartbeatTime: 0,
  joystickMode: '',
  // Per-topic live monitor snapshot from rosbag_recorder (1 Hz while recording).
  recordingMonitor: {
    topics: [],         // [{name, rateHz, baselineHz, secondsSinceLast, status}]
    totalReceived: 0,
    totalWritten: 0,
  },
};

const taskSlice = createSlice({
  name: 'tasks',
  initialState,
  reducers: {
    setTaskInfo: (state, action) => {
      state.taskInfo = { ...state.taskInfo, ...action.payload };
    },
    resetTaskInfo: (state) => {
      state.taskInfo = initialState.taskInfo;
    },
    setTaskStatus: (state, action) => {
      state.taskStatus = { ...state.taskStatus, ...action.payload };
    },
    selectRobotType: (state, action) => {
      state.taskStatus.robotType = action.payload;
    },
    resetTaskStatus: (state) => {
      state.taskStatus = initialState.taskStatus;
    },
    setTaskType: (state, action) => {
      state.taskInfo.taskType = action.payload;
    },
    setTaskInstruction: (state, action) => {
      state.taskInfo.taskInstruction = action.payload;
    },
    setPolicyPath: (state, action) => {
      state.taskInfo.policyPath = action.payload;
    },
    setRecordInferenceMode: (state, action) => {
      state.taskInfo.recordInferenceMode = action.payload;
    },
    setHeartbeatStatus: (state, action) => {
      state.heartbeatStatus = action.payload;
    },
    setLastHeartbeatTime: (state, action) => {
      state.lastHeartbeatTime = action.payload;
    },
    setJoystickMode: (state, action) => {
      state.joystickMode = action.payload || '';
    },
    setRecordingMonitor: (state, action) => {
      state.recordingMonitor = action.payload;
    },
    setPendingPrimitive: (state, action) => {
      state.pendingPrimitive = action.payload || '';
    },
    setPendingSubTask: (state, action) => {
      state.pendingSubTask = action.payload || '';
    },
    setPlannedCount: (state, action) => {
      state.plannedCount = action.payload;
    },
    setPlannedSubTasks: (state, action) => {
      state.plannedSubTasks = action.payload;
    },
    setPlannedSubTaskAt: (state, action) => {
      const { index, value } = action.payload;
      if (index >= 0 && index < state.plannedSubTasks.length) {
        state.plannedSubTasks[index] = value;
      }
    },
    setSlotToServerIdx: (state, action) => {
      state.slotToServerIdx = action.payload;
    },
    setActiveSlotIndex: (state, action) => {
      state.activeSlotIndex = action.payload;
    },
    resetSegmentPlan: (state) => {
      state.plannedCount = 0;
      state.plannedSubTasks = [];
      state.slotToServerIdx = [];
      state.activeSlotIndex = 0;
    },
  },
});

export const {
  setTaskInfo,
  resetTaskInfo,
  setTaskStatus,
  selectRobotType,
  resetTaskStatus,
  setTaskType,
  setTaskInstruction,
  setPolicyPath,
  setRecordInferenceMode,
  setHeartbeatStatus,
  setLastHeartbeatTime,
  setJoystickMode,
  setRecordingMonitor,
  setPendingPrimitive,
  setPendingSubTask,
  setPlannedCount,
  setPlannedSubTasks,
  setPlannedSubTaskAt,
  setSlotToServerIdx,
  setActiveSlotIndex,
  resetSegmentPlan,
} = taskSlice.actions;

export default taskSlice.reducer;
