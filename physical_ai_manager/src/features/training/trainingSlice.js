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
  userList: [],
  datasetList: [],
  selectedUser: undefined,
  selectedDataset: undefined,
  policyList: [],
  deviceList: [],
  modelWeightList: [],
  selectedModelWeight: undefined,
  resumePolicyPath: undefined,
  hasTrainConfig: null,
  isTrainingInfoLoaded: false, // Track if Load button was pressed
  trainingMode: 'new', // 'new' or 'resume'
  isTraining: false,
  topicReceived: false,
  lastUpdate: Date.now(),

  // Training progress
  currentStep: 0,
  updateCounter: 0,

  // Training loss (to be implemented)
  currentLoss: undefined,

  trainingInfo: {
    datasetRepoId: undefined,
    policyType: undefined,
    policyDevice: undefined,
    outputFolderName: undefined,
    resume: false,
    seed: 1000,
    numWorkers: 4,
    batchSize: 8,
    steps: 100000,
    evalFreq: 20000,
    logFreq: 200,
    saveFreq: 20000,
  },
};

const trainingSlice = createSlice({
  name: 'training',
  initialState,
  reducers: {
    setTrainingInfo: (state, action) => {
      state.trainingInfo = action.payload;
    },
    setTopicReceived: (state, action) => {
      state.topicReceived = action.payload;
    },
    setUserList: (state, action) => {
      state.userList = action.payload;
    },
    setDatasetList: (state, action) => {
      state.datasetList = action.payload;
    },
    setSelectedUser: (state, action) => {
      state.selectedUser = action.payload;
    },
    setSelectedDataset: (state, action) => {
      state.selectedDataset = action.payload;
    },
    setDatasetRepoId: (state, action) => {
      state.trainingInfo.datasetRepoId = action.payload;
    },
    setPolicyList: (state, action) => {
      state.policyList = action.payload;
    },
    setDeviceList: (state, action) => {
      state.deviceList = action.payload;
    },
    selectPolicyType: (state, action) => {
      state.trainingInfo.policyType = action.payload;
    },
    selectPolicyDevice: (state, action) => {
      state.trainingInfo.policyDevice = action.payload;
    },
    setOutputFolderName: (state, action) => {
      state.trainingInfo.outputFolderName = action.payload;
    },
    setModelWeightList: (state, action) => {
      state.modelWeightList = action.payload;
    },
    setSelectedModelWeight: (state, action) => {
      state.selectedModelWeight = action.payload;
    },
    setResumePolicyPath: (state, action) => {
      state.resumePolicyPath = action.payload;
      // When policy path changes, mark training info as not loaded
      state.isTrainingInfoLoaded = false;
    },
    setIsTrainingInfoLoaded: (state, action) => {
      state.isTrainingInfoLoaded = action.payload;
    },
    setHasTrainConfig: (state, action) => {
      state.hasTrainConfig = action.payload;
    },
    setTrainingMode: (state, action) => {
      state.trainingMode = action.payload;
      state.trainingInfo.resume = action.payload === 'resume';
    },
    setIsTraining: (state, action) => {
      console.log('setIsTraining', action.payload);
      state.isTraining = action.payload;
    },
    setSeed: (state, action) => {
      state.trainingInfo.seed = action.payload;
    },
    setNumWorkers: (state, action) => {
      state.trainingInfo.numWorkers = action.payload;
    },
    setBatchSize: (state, action) => {
      state.trainingInfo.batchSize = action.payload;
    },
    setSteps: (state, action) => {
      state.trainingInfo.steps = action.payload;
    },
    setEvalFreq: (state, action) => {
      state.trainingInfo.evalFreq = action.payload;
    },
    setLogFreq: (state, action) => {
      state.trainingInfo.logFreq = action.payload;
    },
    setSaveFreq: (state, action) => {
      state.trainingInfo.saveFreq = action.payload;
    },

    setDefaultTrainingInfo: (state) => {
      state.trainingInfo = {
        ...state.trainingInfo,
        seed: 1000,
        numWorkers: 4,
        batchSize: 8,
        steps: 100000,
        evalFreq: 20000,
        logFreq: 200,
        saveFreq: 20000,
      };
    },
    setCurrentStep: (state, action) => {
      state.currentStep = action.payload;
      state.updateCounter++;
    },
    setLastUpdate: (state, action) => {
      state.lastUpdate = action.payload;
    },
    setUpdateCounter: (state, action) => {
      state.updateCounter = action.payload;
    },
    setCurrentLoss: (state, action) => {
      state.currentLoss = action.payload;
    },
    resetTrainingProgress: (state) => {
      state.currentStep = 0;
      state.currentLoss = null;
      state.updateCounter = 0;
    },
  },
});

export const {
  setTrainingInfo,
  setTopicReceived,
  setUserList,
  setDatasetList,
  setSelectedUser,
  setSelectedDataset,
  setDatasetRepoId,
  setPolicyList,
  setDeviceList,
  selectPolicyType,
  selectPolicyDevice,
  setOutputFolderName,
  setModelWeightList,
  setSelectedModelWeight,
  setResumePolicyPath,
  setHasTrainConfig,
  setIsTrainingInfoLoaded,
  setTrainingMode,
  setIsTraining,
  setSeed,
  setNumWorkers,
  setBatchSize,
  setSteps,
  setEvalFreq,
  setLogFreq,
  setSaveFreq,
  setDefaultTrainingInfo,
  setCurrentStep,
  setLastUpdate,
  setUpdateCounter,
  setCurrentLoss,
  resetTrainingProgress,
} = trainingSlice.actions;

export default trainingSlice.reducer;
