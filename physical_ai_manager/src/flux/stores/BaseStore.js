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

import { EventEmitter } from 'events';

/**
 * Base Store Class - All stores extend from this class
 * Provides common functionality for all stores
 */
class BaseStore extends EventEmitter {
  constructor() {
    super();
    this.setMaxListeners(50);
  }

  /**
   * Add change listener
   * @param {Function} callback - Callback function to be called on change
   */
  addChangeListener(callback) {
    this.on('change', callback);
  }

  /**
   * Remove change listener
   * @param {Function} callback - Callback function to be removed
   */
  removeChangeListener(callback) {
    this.removeListener('change', callback);
  }

  /**
   * Emit change event to notify all listeners
   */
  emitChange() {
    this.emit('change');
  }
}

export default BaseStore;
