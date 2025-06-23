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

import Dispatcher from '../dispatcher/Dispatcher';
import ActionTypes from './ActionTypes';

/**
 * Application Actions - All action creators for the application
 */
const AppActions = {
  // Navigation Actions
  navigateToPage(page) {
    Dispatcher.dispatch({
      type: ActionTypes.NAVIGATE_TO_PAGE,
      page,
    });
  },

  setFirstLoad(isFirstLoad) {
    Dispatcher.dispatch({
      type: ActionTypes.SET_FIRST_LOAD,
      isFirstLoad,
    });
  },

  // ROS Connection Actions
  setRosHost(rosHost) {
    Dispatcher.dispatch({
      type: ActionTypes.SET_ROS_HOST,
      rosHost,
    });
  },

  setRosConnectionStatus(connected) {
    Dispatcher.dispatch({
      type: ActionTypes.SET_ROS_CONNECTION_STATUS,
      connected,
    });
  },

  // Robot Type Actions
  setRobotType(robotType) {
    Dispatcher.dispatch({
      type: ActionTypes.SET_ROBOT_TYPE,
      robotType,
    });
  },

  // Topics Actions
  setTopics(topics) {
    Dispatcher.dispatch({
      type: ActionTypes.SET_TOPICS,
      topics,
    });
  },

  updateTopic(index, topic) {
    Dispatcher.dispatch({
      type: ActionTypes.UPDATE_TOPIC,
      index,
      topic,
    });
  },

  // Task Status Actions
  updateTaskStatus(taskStatus) {
    Dispatcher.dispatch({
      type: ActionTypes.UPDATE_TASK_STATUS,
      taskStatus,
    });
  },

  setTaskStatus(taskStatus) {
    Dispatcher.dispatch({
      type: ActionTypes.SET_TASK_STATUS,
      taskStatus,
    });
  },

  // Task Info Actions
  updateTaskInfo(taskInfo) {
    Dispatcher.dispatch({
      type: ActionTypes.UPDATE_TASK_INFO,
      taskInfo,
    });
  },

  setTaskInfo(taskInfo) {
    Dispatcher.dispatch({
      type: ActionTypes.SET_TASK_INFO,
      taskInfo,
    });
  },

  // YAML Configuration Actions
  setYamlContent(yamlContent) {
    Dispatcher.dispatch({
      type: ActionTypes.SET_YAML_CONTENT,
      yamlContent,
    });
  },

  // Control Actions
  sendCommand(command) {
    Dispatcher.dispatch({
      type: ActionTypes.SEND_COMMAND,
      command,
    });
  },

  // Error Actions
  setError(error) {
    Dispatcher.dispatch({
      type: ActionTypes.SET_ERROR,
      error,
    });
  },

  clearError() {
    Dispatcher.dispatch({
      type: ActionTypes.CLEAR_ERROR,
    });
  },
};

export default AppActions;
