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

import BaseStore from './BaseStore';
import Dispatcher from '../dispatcher/Dispatcher';
import ActionTypes from '../actions/ActionTypes';

/**
 * App Store - Manages main application state
 */
class AppStore extends BaseStore {
  constructor() {
    super();
    this.state = {
      currentPage: 'home',
      isFirstLoad: true,
      rosHost: '',
      currentRobotType: '',
      topics: [null, null, null, null],
      yamlContent: null,
      error: null,
      rosConnected: false,
    };

    // Initialize with saved data from localStorage
    this.initializeFromLocalStorage();

    // Register with dispatcher
    this.dispatcherToken = Dispatcher.register(this.handleAction.bind(this));
  }

  /**
   * Initialize state from localStorage
   */
  initializeFromLocalStorage() {
    try {
      const savedYamlContent = localStorage.getItem('yamlFileContent');
      if (savedYamlContent) {
        this.state.yamlContent = JSON.parse(savedYamlContent);
      }
    } catch (error) {
      console.error('Error parsing YAML data from local storage:', error);
    }

    // Set default ROS host
    this.state.rosHost = window.location.hostname + ':8080';
  }

  /**
   * Handle dispatched actions
   * @param {Object} action - Dispatched action
   */
  handleAction(action) {
    switch (action.type) {
      case ActionTypes.NAVIGATE_TO_PAGE:
        this.state.currentPage = action.page;
        this.emitChange();
        break;

      case ActionTypes.SET_FIRST_LOAD:
        this.state.isFirstLoad = action.isFirstLoad;
        this.emitChange();
        break;

      case ActionTypes.SET_ROS_HOST:
        this.state.rosHost = action.rosHost;
        this.emitChange();
        break;

      case ActionTypes.SET_ROS_CONNECTION_STATUS:
        this.state.rosConnected = action.connected;
        this.emitChange();
        break;

      case ActionTypes.SET_ROBOT_TYPE:
        this.state.currentRobotType = action.robotType;
        this.emitChange();
        break;

      case ActionTypes.SET_TOPICS:
        this.state.topics = action.topics;
        this.emitChange();
        break;

      case ActionTypes.UPDATE_TOPIC:
        this.state.topics[action.index] = action.topic;
        this.emitChange();
        break;

      case ActionTypes.SET_YAML_CONTENT:
        this.state.yamlContent = action.yamlContent;
        try {
          localStorage.setItem('yamlFileContent', JSON.stringify(action.yamlContent));
        } catch (error) {
          console.error('Error saving YAML content to localStorage:', error);
        }
        this.emitChange();
        break;

      case ActionTypes.SET_ERROR:
        this.state.error = action.error;
        this.emitChange();
        break;

      case ActionTypes.CLEAR_ERROR:
        this.state.error = null;
        this.emitChange();
        break;

      default:
        // No action
        break;
    }
  }

  /**
   * Get current page
   * @returns {string} Current page
   */
  getCurrentPage() {
    return this.state.currentPage;
  }

  /**
   * Get first load status
   * @returns {boolean} Is first load
   */
  getIsFirstLoad() {
    return this.state.isFirstLoad;
  }

  /**
   * Get ROS host
   * @returns {string} ROS host
   */
  getRosHost() {
    return this.state.rosHost;
  }

  /**
   * Get ROS bridge URL
   * @returns {string} ROS bridge URL
   */
  getRosBridgeUrl() {
    return `ws://${this.state.rosHost.split(':')[0]}:9090`;
  }

  /**
   * Get ROS connection status
   * @returns {boolean} ROS connection status
   */
  getRosConnectionStatus() {
    return this.state.rosConnected;
  }

  /**
   * Get current robot type
   * @returns {string} Current robot type
   */
  getCurrentRobotType() {
    return this.state.currentRobotType;
  }

  /**
   * Get topics
   * @returns {Array} Topics array
   */
  getTopics() {
    return this.state.topics;
  }

  /**
   * Get YAML content
   * @returns {Object|null} YAML content
   */
  getYamlContent() {
    return this.state.yamlContent;
  }

  /**
   * Get error
   * @returns {string|null} Current error
   */
  getError() {
    return this.state.error;
  }

  /**
   * Get all state
   * @returns {Object} Complete state object
   */
  getState() {
    return { ...this.state };
  }
}

export default new AppStore();
