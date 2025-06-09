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

import { useRef, useEffect, useState, useCallback } from 'react';
import ROSLIB from 'roslib';

export function useRosTaskStatus(rosbridgeUrl, topicName = '/task/status') {
  const rosRef = useRef(null);
  const topicRef = useRef(null);

  const [taskStatus, setTaskStatus] = useState({
    taskName: 'idle',
    running: false,
    phase: 4, // STOPED
    progress: 0,
    totalTime: 0,
    proceedTime: 0,
    currentEpisodeNumber: 0,
    repoId: '',
    usedStorageSize: 0,
    totalStorageSize: 0,
  });

  const [taskInfo, setTaskInfo] = useState({
    taskName: 'ai_worker_test_task',
    robotType: 'ai_worker',
    taskType: 'record',
    taskInstruction: 'pick and place objects',
    repoId: 'robotis/ai_worker_dataset',
    fps: 30,
    tags: ['tutorial'],
    warmupTime: 5,
    episodeTime: 60,
    resetTime: 60,
    numEpisodes: 100,
    resume: true,
    pushToHub: true,
    privateMode: false,
    useImageBuffer: false,
  });

  const [connected, setConnected] = useState(false);

  const getRosConnection = useCallback(() => {
    if (!rosRef.current || rosRef.current.isConnected === false) {
      rosRef.current = new ROSLIB.Ros({ url: rosbridgeUrl });

      rosRef.current.on('connection', () => {
        console.log('Connected to ROS bridge for task status');
        setConnected(true);
      });

      rosRef.current.on('error', (error) => {
        console.error('ROS task status connection error:', error);
        setConnected(false);
        rosRef.current = null;
      });

      rosRef.current.on('close', () => {
        console.log('ROS task status connection closed');
        setConnected(false);
        rosRef.current = null;
      });
    }
    return rosRef.current;
  }, [rosbridgeUrl]);

  const subscribeToTaskStatus = useCallback(() => {
    const ros = getRosConnection();
    if (!ros) return;

    // Unsubscribe existing topic if any
    if (topicRef.current) {
      topicRef.current.unsubscribe();
      topicRef.current = null;
    }

    try {
      topicRef.current = new ROSLIB.Topic({
        ros,
        name: topicName,
        messageType: 'physical_ai_interfaces/msg/TaskStatus',
      });

      topicRef.current.subscribe((msg) => {
        console.log('Received task status:', msg);

        // Calculate progress percentage
        const progress = msg.total_time > 0 ? (msg.proceed_time / msg.total_time) * 100 : 0;

        // Determine if task is running (phase 1, 2, or 3)
        const isRunning = msg.phase >= 1 && msg.phase <= 3;

        // ROS message to React state
        setTaskStatus({
          taskName: msg.task_info?.task_name || 'idle',
          running: isRunning,
          phase: msg.phase || 0,
          progress: Math.round(progress),
          totalTime: msg.total_time || 0,
          proceedTime: msg.proceed_time || 0,
          currentEpisodeNumber: msg.current_episode_number || 0,
          repoId: msg.task_info?.repo_id || '',
          usedStorageSize: msg.used_storage_size || 0,
          totalStorageSize: msg.total_storage_size || 0,
        });

        // Extract TaskInfo from TaskStatus message
        if (msg.task_info) {
          if (msg.phase != 0) {
            // update task info only when task is not stopped
            setTaskInfo({
              taskName: msg.task_info.task_name || '',
              robotType: msg.task_info.robot_type || '',
              taskType: msg.task_info.task_type || '',
              taskInstruction: msg.task_info.task_instruction || '',
              repoId: msg.task_info.repo_id || '',
              fps: msg.task_info.fps || 0,
              tags: msg.task_info.tags || [],
              warmupTime: msg.task_info.warmup_time_s || 0,
              episodeTime: msg.task_info.episode_time_s || 0,
              resetTime: msg.task_info.reset_time_s || 0,
              numEpisodes: msg.task_info.num_episodes || 0,
              resume: msg.task_info.resume || false,
              pushToHub: msg.task_info.push_to_hub || false,
              privateMode: msg.task_info.private_mode || false,
              useImageBuffer: msg.task_info.use_image_buffer || false,
            });
          }
        }
      });
    } catch (error) {
      console.error('Failed to subscribe to task status topic:', error);
    }
  }, [getRosConnection, topicName]);

  const cleanup = useCallback(() => {
    if (topicRef.current) {
      topicRef.current.unsubscribe();
      topicRef.current = null;
    }
    if (rosRef.current) {
      rosRef.current.close();
      rosRef.current = null;
    }
    setConnected(false);
  }, []);

  // Start connection and subscription
  useEffect(() => {
    if (!rosbridgeUrl) return;

    subscribeToTaskStatus();

    return cleanup;
  }, [rosbridgeUrl, subscribeToTaskStatus, cleanup]);

  // Helper function to get phase name
  const getPhaseName = useCallback((phase) => {
    const phaseNames = {
      0: 'NONE',
      1: 'WARMING_UP',
      2: 'RESETTING',
      3: 'RECORDING',
      4: 'STOPPED',
    };
    return phaseNames[phase] || 'UNKNOWN';
  }, []);

  // Function to manually update task status (for stop/finish commands)
  const updateTaskStatus = useCallback((updates) => {
    setTaskStatus((prevStatus) => ({
      ...prevStatus,
      ...updates,
    }));
  }, []);

  // Function to reset task to initial state
  const resetTaskToIdle = useCallback(() => {
    setTaskStatus((prevStatus) => ({
      ...prevStatus,
      running: false,
      phase: 0,
    }));
  }, []);

  return {
    taskStatus,
    taskInfo,
    connected,
    subscribeToTaskStatus,
    cleanup,
    getPhaseName,
    updateTaskStatus,
    resetTaskToIdle,
  };
}
