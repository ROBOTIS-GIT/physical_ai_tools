// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import useFluxStore from './useFluxStore';
import AppStore from '../stores/AppStore';

/**
 * Custom hook to use AppStore in React components
 * @returns {Object} App state and methods
 */
export function useAppStore() {
  const state = useFluxStore(AppStore, (store) => store.getState());

  return {
    // State
    currentPage: state.currentPage,
    isFirstLoad: state.isFirstLoad,
    rosHost: state.rosHost,
    currentRobotType: state.currentRobotType,
    topics: state.topics,
    yamlContent: state.yamlContent,
    error: state.error,
    rosConnected: state.rosConnected,

    // Methods (can be extended if needed)
    getCurrentPage: () => AppStore.getCurrentPage(),
    getIsFirstLoad: () => AppStore.getIsFirstLoad(),
    getRosHost: () => AppStore.getRosHost(),
    getCurrentRobotType: () => AppStore.getCurrentRobotType(),
    getTopics: () => AppStore.getTopics(),
    getYamlContent: () => AppStore.getYamlContent(),
    getError: () => AppStore.getError(),
    getRosConnectionStatus: () => AppStore.getRosConnectionStatus(),
  };
}

export default useAppStore;
