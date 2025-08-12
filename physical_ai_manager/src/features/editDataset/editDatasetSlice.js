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
  datasetList: [],
  deleteDatasetPath: '',
  outputPath: '',
  deleteEpisodeNum: [],
  uploadHuggingface: false,
};

const editDatasetSlice = createSlice({
  name: 'editDataset',
  initialState,
  reducers: {
    setEditDatasetInfo: (state, action) => {
      state.editDatasetInfo = action.payload;
    },
    setDatasetList: (state, action) => {
      state.datasetList = action.payload;
    },
    setDeleteDatasetPath: (state, action) => {
      state.deleteDatasetPath = action.payload;
    },
    setOutputPath: (state, action) => {
      state.outputPath = action.payload;
    },
    setDeleteEpisodeNum: (state, action) => {
      state.deleteEpisodeNum = action.payload;
    },
    setUploadHuggingface: (state, action) => {
      state.uploadHuggingface = action.payload;
    },
    setDefaultEditDatasetInfo: (state) => {
      state.editDatasetInfo = {
        ...state.editDatasetInfo,
        uploadHuggingface: false,
      };
    },
  },
});

export const {
  setEditDatasetInfo,
  setDatasetList,
  setDeleteDatasetPath,
  setOutputPath,
  setDeleteEpisodeNum,
  setUploadHuggingface,
  setDefaultEditDatasetInfo,
} = editDatasetSlice.actions;

export default editDatasetSlice.reducer;
