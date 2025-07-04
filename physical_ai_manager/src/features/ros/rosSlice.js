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
  connected: false,
  rosHost: '',
  rosbridgeUrl: '',
  imageTopicList: [],
};

const rosSlice = createSlice({
  name: 'ros',
  initialState,
  reducers: {
    setConnected: (state, action) => {
      state.connected = action.payload;
    },
    setRosHost: (state, action) => {
      state.rosHost = action.payload;
      state.rosbridgeUrl = `ws://${action.payload}:9090`;
    },
    setRosbridgeUrl: (state, action) => {
      state.rosbridgeUrl = action.payload;
    },
    setImageTopicList: (state, action) => {
      state.imageTopicList = action.payload;
    },
  },
});

export const { setConnected, setRosHost, setRosbridgeUrl, setImageTopicList } = rosSlice.actions;

export default rosSlice.reducer;
