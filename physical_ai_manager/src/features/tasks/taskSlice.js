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
    taskName: '',
    taskType: '',
    taskInstruction: [],
    policyPath: '',
    recordInferenceMode: false,
    userId: undefined,
    fps: 30,
    tags: [],
    warmupTime: 5,
    episodeTime: 20,
    resetTime: 5,
    numEpisodes: 5,
    token: '',
    pushToHub: true,
    privateMode: false,
    useOptimizedSave: true,
    recordRosBag2: false,
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
    userId: '',
    usedStorageSize: 0,
    totalStorageSize: 0,
    usedCpu: 0,
    usedRamSize: 0,
    totalRamSize: 0,
    error: '',
    topicReceived: false,
  },
  availableRobots: [],
  availableCameras: [],
  policyList: [],
  datasetList: [],
  heartbeatStatus: 'disconnected',
  lastHeartbeatTime: 0,
  useMultiTaskMode: false,
  multiTaskIndex: undefined,
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
    addTag: (state, action) => {
      if (!state.taskInfo.tags.includes(action.payload)) {
        state.taskInfo.tags.push(action.payload);
      }
    },
    removeTag: (state, action) => {
      state.taskInfo.tags = state.taskInfo.tags.filter((tag) => tag !== action.payload);
    },
    removeAllTags: (state) => {
      state.taskInfo.tags = [];
    },
    setHeartbeatStatus: (state, action) => {
      state.heartbeatStatus = action.payload;
    },
    setLastHeartbeatTime: (state, action) => {
      state.lastHeartbeatTime = action.payload;
    },
    setUseMultiTaskMode: (state, action) => {
      state.useMultiTaskMode = action.payload;
    },
    setMultiTaskIndex: (state, action) => {
      state.multiTaskIndex = action.payload;
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
  addTag,
  removeTag,
  removeAllTags,
  setHeartbeatStatus,
  setLastHeartbeatTime,
  setUseMultiTaskMode,
  setMultiTaskIndex,
} = taskSlice.actions;

export default taskSlice.reducer;
