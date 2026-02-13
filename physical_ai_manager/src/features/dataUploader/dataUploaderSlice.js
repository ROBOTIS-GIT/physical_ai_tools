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
 */

import { createSlice } from '@reduxjs/toolkit';

const initialState = {
  // HuggingFace 사용자 정보
  hfUserId: '',
  hfToken: '',

  // 선택된 폴더들
  selectedFolders: [],

  // 업로드 진행률
  uploadProgress: [
    // { folderName: 'my_robot_task1', status: 'uploading', percentage: 80 }
  ],

  // 전체 상태
  isUploading: false,
  error: null,
};

const dataUploaderSlice = createSlice({
  name: 'dataUploader',
  initialState,
  reducers: {
    setHfUserId: (state, action) => {
      state.hfUserId = action.payload;
    },

    setHfToken: (state, action) => {
      state.hfToken = action.payload;
    },

    setSelectedFolders: (state, action) => {
      state.selectedFolders = action.payload;
    },

    addSelectedFolder: (state, action) => {
      const folder = action.payload;
      const exists = state.selectedFolders.find(f => f.path === folder.path);
      if (!exists) {
        state.selectedFolders.push(folder);
      }
    },

    removeSelectedFolder: (state, action) => {
      const path = action.payload;
      state.selectedFolders = state.selectedFolders.filter(f => f.path !== path);
    },

    clearSelectedFolders: (state) => {
      state.selectedFolders = [];
    },

    startUpload: (state) => {
      state.isUploading = true;
      state.uploadProgress = state.selectedFolders.map(folder => ({
        folderName: folder.name,
        repoId: `${state.hfUserId}/${folder.name}`,
        status: 'pending',
        percentage: 0,
      }));
      state.error = null;
    },

    updateUploadProgress: (state, action) => {
      const { folderName, percentage, status } = action.payload;
      const item = state.uploadProgress.find(p => p.folderName === folderName);
      if (item) {
        item.percentage = percentage;
        if (status) item.status = status;
      }
    },

    completeUpload: (state) => {
      state.isUploading = false;
    },

    setError: (state, action) => {
      state.error = action.payload;
      state.isUploading = false;
    },

    resetUploadProgress: (state) => {
      state.uploadProgress = [];
      state.isUploading = false;
      state.error = null;
    },
  },
});

export const {
  setHfUserId,
  setHfToken,
  setSelectedFolders,
  addSelectedFolder,
  removeSelectedFolder,
  clearSelectedFolders,
  startUpload,
  updateUploadProgress,
  completeUpload,
  setError,
  resetUploadProgress,
} = dataUploaderSlice.actions;

export default dataUploaderSlice.reducer;
