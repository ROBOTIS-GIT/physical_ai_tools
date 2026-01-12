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
 * Author: Dongyun Kim
 */

import { createSlice } from '@reduxjs/toolkit';

const initialState = {
  // Selected bag path
  selectedBagPath: null,

  // Loading state
  isLoading: false,
  isLoaded: false,
  error: null,

  // Video information
  videoFiles: [],
  videoTopics: [],
  videoNames: [],  // Human-readable camera names
  videoFps: [],
  videoServerPort: 8081,
  bagPath: null,

  // Frame metadata
  frameIndices: [],
  frameTimestamps: [],

  // Joint data
  jointTimestamps: [],
  jointNames: [],
  jointPositions: [],

  // Action data
  actionTimestamps: [],
  actionNames: [],
  actionValues: [],

  // Duration info
  startTime: 0,
  endTime: 0,
  duration: 0,

  // Playback state
  currentTime: 0,
  isPlaying: false,
  isVideoLoaded: false,
  videoLoadProgress: 0,

  // Extended metadata
  robotType: '',
  recordingDate: null,
  fileSizeBytes: 0,
  taskMarkers: [],
  frameCounts: {},
};

const replaySlice = createSlice({
  name: 'replay',
  initialState,
  reducers: {
    setSelectedBagPath: (state, action) => {
      state.selectedBagPath = action.payload;
      // Reset loaded state when bag path changes
      state.isLoaded = false;
      state.error = null;
    },
    setLoading: (state, action) => {
      state.isLoading = action.payload;
    },
    setReplayData: (state, action) => {
      const data = action.payload;
      state.videoFiles = data.video_files || [];
      state.videoTopics = data.video_topics || [];
      state.videoNames = data.video_names || [];
      state.videoFps = data.video_fps || [];
      state.videoServerPort = data.video_server_port || 8082;
      state.bagPath = data.bag_path || null;
      state.frameIndices = data.frame_indices || [];
      state.frameTimestamps = data.frame_timestamps || [];
      state.jointTimestamps = data.joint_timestamps || [];
      state.jointNames = data.joint_names || [];
      state.jointPositions = data.joint_positions || [];
      state.actionTimestamps = data.action_timestamps || [];
      state.actionNames = data.action_names || [];
      state.actionValues = data.action_values || [];
      state.startTime = data.start_time || 0;
      state.endTime = data.end_time || 0;
      state.duration = data.duration || 0;
      // Extended metadata
      state.robotType = data.robot_type || '';
      state.recordingDate = data.recording_date || null;
      state.fileSizeBytes = data.file_size_bytes || 0;
      state.taskMarkers = data.task_markers || [];
      state.frameCounts = data.frame_counts || {};
      state.isLoaded = true;
      state.isLoading = false;
      state.error = null;
    },
    setTaskMarkers: (state, action) => {
      state.taskMarkers = action.payload;
    },
    setError: (state, action) => {
      state.error = action.payload;
      state.isLoading = false;
      state.isLoaded = false;
    },
    setCurrentTime: (state, action) => {
      state.currentTime = action.payload;
    },
    setIsPlaying: (state, action) => {
      state.isPlaying = action.payload;
    },
    setIsVideoLoaded: (state, action) => {
      state.isVideoLoaded = action.payload;
    },
    setVideoLoadProgress: (state, action) => {
      state.videoLoadProgress = action.payload;
    },
    resetReplayState: (state) => {
      return initialState;
    },
  },
});

export const {
  setSelectedBagPath,
  setLoading,
  setReplayData,
  setTaskMarkers,
  setError,
  setCurrentTime,
  setIsPlaying,
  setIsVideoLoaded,
  setVideoLoadProgress,
  resetReplayState,
} = replaySlice.actions;

export default replaySlice.reducer;
