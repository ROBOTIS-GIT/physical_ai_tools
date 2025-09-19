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

const initialState = {
  mergeDatasetList: [],
  datasetToDeleteEpisode: '',
  datasetInfo: {
    totalEpisodes: 0,
    totalTasks: 0,
    fps: 0,
    codebaseVersion: '',
    robotType: '',
  },
  mergeOutputPath: '',
  mergeOutputFolderName: '',
  deleteEpisodeNums: [],
  uploadHuggingface: false,
  hfUserId: '',
  hfRepoIdUpload: '',
  hfRepoIdDownload: '',
  hfStatus: 'Idle',
  hfDataType: 'dataset',
  uploadStatus: {
    current: 0,
    total: 0,
    percentage: 0.0,
  },
  downloadStatus: {
    current: 0,
    total: 0,
    percentage: 0.0,
  },
};

const editDatasetSlice = createSlice({
  name: 'editDataset',
  initialState,
  reducers: {
    setMergeDatasetList: (state, action) => {
      state.mergeDatasetList = action.payload;
    },
    setDatasetToDeleteEpisode: (state, action) => {
      state.datasetToDeleteEpisode = action.payload;
    },
    setDatasetInfo: (state, action) => {
      state.datasetInfo = action.payload;
    },
    setMergeOutputPath: (state, action) => {
      state.mergeOutputPath = action.payload;
    },
    setMergeOutputFolderName: (state, action) => {
      state.mergeOutputFolderName = action.payload;
    },
    setDeleteEpisodeNums: (state, action) => {
      state.deleteEpisodeNums = action.payload;
    },
    setUploadHuggingface: (state, action) => {
      state.uploadHuggingface = action.payload;
    },
    setHFUserId: (state, action) => {
      state.hfUserId = action.payload;
    },
    setHFRepoIdUpload: (state, action) => {
      state.hfRepoIdUpload = action.payload;
    },
    setHFRepoIdDownload: (state, action) => {
      state.hfRepoIdDownload = action.payload;
    },
    setHFStatus: (state, action) => {
      state.hfStatus = action.payload;
    },
    setHFDataType: (state, action) => {
      state.hfDataType = action.payload;
    },
    setUploadStatus: (state, action) => {
      state.uploadStatus = action.payload;
    },
    setDownloadStatus: (state, action) => {
      state.downloadStatus = action.payload;
    },
  },
});

export const {
  setMergeDatasetList,
  setDatasetToDeleteEpisode,
  setDatasetInfo,
  setMergeOutputPath,
  setMergeOutputFolderName,
  setDeleteEpisodeNums,
  setUploadHuggingface,
  setHFUserId,
  setHFRepoIdUpload,
  setHFRepoIdDownload,
  setHFStatus,
  setHFDataType,
  setUploadStatus,
  setDownloadStatus,
} = editDatasetSlice.actions;

export default editDatasetSlice.reducer;
