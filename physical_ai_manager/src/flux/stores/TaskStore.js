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
import TaskPhase from '../../constants/taskPhases';

/**
 * Task Store - Manages task status and information
 */
class TaskStore extends BaseStore {
  constructor() {
    super();
    this.state = {
      taskStatus: {
        robotType: '',
        taskName: 'idle',
        running: false,
        phase: TaskPhase.READY,
        progress: 0,
        totalTime: 0,
        proceedTime: 0,
        currentEpisodeNumber: 0,
        userId: '',
        usedStorageSize: 0,
        totalStorageSize: 0,
        usedCpu: 0,
        usedRamSize: 0,
        totalRamSize: 0,
        error: '',
        topicReceived: false,
      },
      taskInfo: {
        taskName: '',
        taskType: 'record',
        taskInstruction: '',
        userId: '',
        fps: 30,
        tags: [],
        warmupTime: 5,
        episodeTime: 20,
        resetTime: 5,
        numEpisodes: 5,
        token: '',
        pushToHub: true,
        privateMode: false,
        useOptimizedSave: true,
      },
    };

    // Register with dispatcher
    this.dispatcherToken = Dispatcher.register(this.handleAction.bind(this));
  }

  /**
   * Handle dispatched actions
   * @param {Object} action - Dispatched action
   */
  handleAction(action) {
    switch (action.type) {
      case ActionTypes.UPDATE_TASK_STATUS:
        this.state.taskStatus = { ...this.state.taskStatus, ...action.taskStatus };
        this.emitChange();
        break;

      case ActionTypes.SET_TASK_STATUS:
        this.state.taskStatus = action.taskStatus;
        this.emitChange();
        break;

      case ActionTypes.UPDATE_TASK_INFO:
        this.state.taskInfo = { ...this.state.taskInfo, ...action.taskInfo };
        this.emitChange();
        break;

      case ActionTypes.SET_TASK_INFO:
        this.state.taskInfo = action.taskInfo;
        this.emitChange();
        break;

      default:
        // No action
        break;
    }
  }

  /**
   * Get task status
   * @returns {Object} Task status
   */
  getTaskStatus() {
    return this.state.taskStatus;
  }

  /**
   * Get task info
   * @returns {Object} Task info
   */
  getTaskInfo() {
    return this.state.taskInfo;
  }

  /**
   * Get phase name
   * @param {number} phase - Phase number
   * @returns {string} Phase name
   */
  getPhaseName(phase) {
    const phaseNames = {
      [TaskPhase.READY]: 'NONE',
      [TaskPhase.WARMING_UP]: 'WARMING_UP',
      [TaskPhase.RESETTING]: 'RESETTING',
      [TaskPhase.RECORDING]: 'RECORDING',
      [TaskPhase.SAVING]: 'SAVING',
      [TaskPhase.STOPPED]: 'STOPPED',
    };
    return phaseNames[phase] || 'UNKNOWN';
  }

  /**
   * Check if task is in ready state
   * @returns {boolean} Is ready state
   */
  isReadyState() {
    return this.state.taskStatus.phase === TaskPhase.READY;
  }

  /**
   * Check if task is in stopped state
   * @returns {boolean} Is stopped state
   */
  isStoppedState() {
    return this.state.taskStatus.phase === TaskPhase.STOPPED;
  }

  /**
   * Check if task is in running state
   * @returns {boolean} Is running state
   */
  isRunningState() {
    const phase = this.state.taskStatus.phase;
    return (
      phase === TaskPhase.WARMING_UP ||
      phase === TaskPhase.RESETTING ||
      phase === TaskPhase.RECORDING ||
      phase === TaskPhase.SAVING
    );
  }

  /**
   * Get all task-related state
   * @returns {Object} Complete task state
   */
  getState() {
    return { ...this.state };
  }
}

export default new TaskStore();
