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

/**
 * Action Types - Constants for all application actions
 */
export const ActionTypes = {
  // Navigation Actions
  NAVIGATE_TO_PAGE: 'NAVIGATE_TO_PAGE',
  SET_FIRST_LOAD: 'SET_FIRST_LOAD',

  // ROS Connection Actions
  SET_ROS_HOST: 'SET_ROS_HOST',

  // Robot Type Actions
  SET_ROBOT_TYPE: 'SET_ROBOT_TYPE',

  // Topics Actions
  SET_TOPICS: 'SET_TOPICS',
  UPDATE_TOPIC: 'UPDATE_TOPIC',

  // Task Status Actions
  UPDATE_TASK_STATUS: 'UPDATE_TASK_STATUS',
  SET_TASK_STATUS: 'SET_TASK_STATUS',

  // Task Info Actions
  UPDATE_TASK_INFO: 'UPDATE_TASK_INFO',
  SET_TASK_INFO: 'SET_TASK_INFO',

  // YAML Configuration Actions
  SET_YAML_CONTENT: 'SET_YAML_CONTENT',

  // Control Actions
  SEND_COMMAND: 'SEND_COMMAND',

  // Error Actions
  SET_ERROR: 'SET_ERROR',
  CLEAR_ERROR: 'CLEAR_ERROR',

  // ROS Connection Status Actions
  SET_ROS_CONNECTION_STATUS: 'SET_ROS_CONNECTION_STATUS',
};

export default ActionTypes;
