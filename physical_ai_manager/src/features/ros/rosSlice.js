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

// Resolved at module load so the very first render already has a valid
// rosbridge URL — child components mount with a working connection target
// instead of waiting for an effect-time dispatch.
const defaultRosHost = typeof window !== 'undefined' ? window.location.hostname : '';

// On HTTPS pages, plain ws:// is blocked as mixed content. Route through the
// same-origin nginx /ws/ proxy so the browser sees wss:// → nginx → ws://localhost:9090.
const buildDefaultRosbridgeUrl = (host) => {
  if (!host) return '';
  if (typeof window !== 'undefined' && window.location.protocol === 'https:') {
    return `wss://${window.location.host}/ws/`;
  }
  return `ws://${host}:9090`;
};

const initialState = {
  connected: false,
  connecting: false,
  rosHost: defaultRosHost,
  rosbridgeUrl: buildDefaultRosbridgeUrl(defaultRosHost),
  imageTopicList: [],
  /** Persisted camera topic assignment [left, center, right] so it survives ImageGrid remounts */
  assignedImageTopics: [],
  connectionError: null,
};

const rosSlice = createSlice({
  name: 'ros',
  initialState,
  reducers: {
    setConnected: (state, action) => {
      state.connected = action.payload;
    },
    setConnecting: (state, action) => {
      state.connecting = action.payload;
    },
    setRosHost: (state, action) => {
      state.rosHost = action.payload;
      state.rosbridgeUrl = buildDefaultRosbridgeUrl(action.payload);
    },
    setRosbridgeUrl: (state, action) => {
      state.rosbridgeUrl = action.payload;
    },
    setImageTopicList: (state, action) => {
      state.imageTopicList = action.payload;
    },
    setAssignedImageTopics: (state, action) => {
      state.assignedImageTopics = action.payload;
    },
    setConnectionError: (state, action) => {
      state.connectionError = action.payload;
    },
    resetConnection: (state) => {
      state.connected = false;
      state.connecting = false;
      state.connectionError = null;
    },
  },
});

export const {
  setConnected,
  setConnecting,
  setRosHost,
  setRosbridgeUrl,
  setImageTopicList,
  setAssignedImageTopics,
  setConnectionError,
  resetConnection,
} = rosSlice.actions;

export default rosSlice.reducer;
