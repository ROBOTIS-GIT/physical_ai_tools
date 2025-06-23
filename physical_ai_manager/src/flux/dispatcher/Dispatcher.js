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
 * Flux Dispatcher - Central hub for all application actions
 * Manages the data flow in the application
 */
class Dispatcher {
  constructor() {
    this.callbacks = {};
    this.isDispatching = false;
    this.isHandled = {};
    this.isPending = {};
    this.lastID = 1;
  }

  /**
   * Register a callback to be invoked with every dispatched payload
   * @param {Function} callback - Callback function to handle dispatched actions
   * @returns {string} - Token for unregistering the callback
   */
  register(callback) {
    const id = 'ID_' + this.lastID++;
    this.callbacks[id] = callback;
    return id;
  }

  /**
   * Unregister a callback
   * @param {string} id - Token returned by register
   */
  unregister(id) {
    delete this.callbacks[id];
  }

  /**
   * Wait for specified callbacks to complete before continuing
   * @param {Array<string>} ids - Array of dispatcher tokens
   */
  waitFor(ids) {
    if (!this.isDispatching) {
      throw new Error('Dispatcher: waitFor called outside dispatch');
    }

    for (let i = 0; i < ids.length; i++) {
      const id = ids[i];
      if (this.isPending[id]) {
        if (!this.isHandled[id]) {
          throw new Error('Dispatcher: Circular dependency detected');
        }
        continue;
      }
      this.invokeCallback(id);
    }
  }

  /**
   * Dispatch an action to all registered callbacks
   * @param {Object} payload - Action payload
   */
  dispatch(payload) {
    if (this.isDispatching) {
      throw new Error('Dispatcher: Cannot dispatch in the middle of a dispatch');
    }

    this.startDispatching(payload);
    try {
      for (const id in this.callbacks) {
        if (this.isPending[id]) {
          continue;
        }
        this.invokeCallback(id);
      }
    } finally {
      this.stopDispatching();
    }
  }

  /**
   * Start dispatching process
   * @param {Object} payload - Action payload
   */
  startDispatching(payload) {
    for (const id in this.callbacks) {
      this.isPending[id] = false;
      this.isHandled[id] = false;
    }
    this.pendingPayload = payload;
    this.isDispatching = true;
  }

  /**
   * Invoke a specific callback
   * @param {string} id - Callback ID
   */
  invokeCallback(id) {
    this.isPending[id] = true;
    this.callbacks[id](this.pendingPayload);
    this.isHandled[id] = true;
  }

  /**
   * Stop dispatching process
   */
  stopDispatching() {
    delete this.pendingPayload;
    this.isDispatching = false;
  }
}

export default new Dispatcher();
