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
//
// Author: Kiwoong Park

import ROSLIB from 'roslib';

/**
 * Singleton pattern for managing global ROS connection
 */
class RosConnectionManager {
  constructor() {
    this.ros = null;
    this.connecting = false;
    this.url = '';
    this.connectionPromise = null;
    this.onConnected = null;
  }

  /**
   * Set the callback function to be called when the ROS connection is established.
   * @param {function} onConnected - Callback function to execute on successful connection.
   */
  setOnConnected(onConnected) {
    if (typeof onConnected === 'function') {
      this.onConnected = onConnected;
    } else {
      console.warn('setOnConnected: provided callback is not a function');
      this.onConnected = null;
    }
  }

  /**
   * Get or create ROS connection
   * @param {string} rosbridgeUrl - rosbridge WebSocket URL
   * @returns {Promise<ROSLIB.Ros>} ROS connection object
   */
  async getConnection(rosbridgeUrl) {
    // If URL has changed, clean up existing connection
    if (this.url !== rosbridgeUrl) {
      this.disconnect();
      this.url = rosbridgeUrl;
    }

    // If already connected, return existing connection
    if (this.ros && this.ros.isConnected) {
      return this.ros;
    }

    // If connection attempt is in progress, return same Promise
    if (this.connecting && this.connectionPromise) {
      console.log('Connection attempt in progress, waiting...');
      return this.connectionPromise;
    }

    // Create new connection
    console.log('Creating new global ROS connection to:', rosbridgeUrl);
    this.connecting = true;

    this.connectionPromise = new Promise((resolve, reject) => {
      const ros = new ROSLIB.Ros({ url: rosbridgeUrl });

      const connectionTimeout = setTimeout(() => {
        this.connecting = false;
        this.connectionPromise = null;
        reject(new Error('ROS connection timeout - rosbridge server is not running'));
      }, 10000);

      ros.on('connection', () => {
        clearTimeout(connectionTimeout);
        console.log('Global ROS connection established');
        this.ros = ros;
        this.connecting = false;
        this.connectionPromise = null;
        resolve(ros);

        if (this.onConnected && typeof this.onConnected === 'function') {
          try {
            this.onConnected();
          } catch (error) {
            console.error('Error calling onConnected callback:', error);
          }
        }
      });

      ros.on('error', (error) => {
        clearTimeout(connectionTimeout);
        console.error('Global ROS connection error:', error);
        this.connecting = false;
        this.connectionPromise = null;
        this.ros = null;
        reject(new Error(`ROS connection failed: ${error.message || error}`));
      });

      ros.on('close', () => {
        console.log('Global ROS connection closed');
        this.ros = null;
        this.connecting = false;
        this.connectionPromise = null;
        // Reset connection attempts on close to allow reconnection
        this.connectionAttempts = 0;
      });
    });

    return this.connectionPromise;
  }

  /**
   * Disconnect ROS connection
   */
  disconnect() {
    if (this.ros) {
      console.log('Disconnecting global ROS connection');
      this.ros.close();
      this.ros = null;
    }
    this.connecting = false;
    this.connectionPromise = null;
    this.url = '';
  }

  /**
   * Check connection status
   */
  isConnected() {
    return this.ros && this.ros.isConnected;
  }

  /**
   * Return current connection object (only if connected)
   */
  getCurrentConnection() {
    return this.isConnected() ? this.ros : null;
  }
}

// Create singleton instance
const rosConnectionManager = new RosConnectionManager();

export default rosConnectionManager;
