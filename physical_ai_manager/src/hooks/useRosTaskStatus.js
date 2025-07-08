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
import toast from 'react-hot-toast';
import { useDispatch, useSelector } from 'react-redux';
import ROSLIB from 'roslib';
import TaskPhase from '../constants/taskPhases';
import {
  setTaskStatus,
  setTaskInfo,
  setHeartbeatStatus,
  setLastHeartbeatTime,
} from '../features/tasks/taskSlice';
import rosConnectionManager from '../utils/rosConnectionManager';

export function useRosTaskStatus() {
  const taskStatusTopicRef = useRef(null);
  const heartbeatTopicRef = useRef(null);

  const dispatch = useDispatch();
  const rosbridgeUrl = useSelector((state) => state.ros.rosbridgeUrl);
  const [connected, setConnected] = useState(false);

  const cleanup = useCallback(() => {
    console.log('Starting ROS task status cleanup...');

    if (taskStatusTopicRef.current) {
      taskStatusTopicRef.current.unsubscribe();
      taskStatusTopicRef.current = null;
      console.log('Task status topic unsubscribed');
    }
    if (heartbeatTopicRef.current) {
      heartbeatTopicRef.current.unsubscribe();
      heartbeatTopicRef.current = null;
      console.log('Heartbeat topic unsubscribed');
    }
    setConnected(false);
    dispatch(setHeartbeatStatus('disconnected'));
    console.log('ROS task status cleanup completed');
  }, [dispatch]);

  const subscribeToTaskStatus = useCallback(async () => {
    try {
      const ros = await rosConnectionManager.getConnection(rosbridgeUrl);
      if (!ros) return;

      // Skip if already subscribed
      if (taskStatusTopicRef.current) {
        console.log('Task status already subscribed, skipping...');
        return;
      }

      setConnected(true);
      taskStatusTopicRef.current = new ROSLIB.Topic({
        ros,
        name: '/task/status',
        messageType: 'physical_ai_interfaces/msg/TaskStatus',
      });

      taskStatusTopicRef.current.subscribe((msg) => {
        console.log('Received task status:', msg);

        let progress = 0;

        if (msg.error !== '') {
          console.log('error:', msg.error);
          toast.error(msg.error);
          return;
        }

        // Calculate progress percentage
        if (msg.phase === TaskPhase.SAVING) {
          // Saving data phase
          progress = msg.encoding_progress || 0;
        } else {
          // all other phases
          progress = msg.total_time > 0 ? (msg.proceed_time / msg.total_time) * 100 : 0;
        }

        const isRunning =
          msg.phase === TaskPhase.WARMING_UP ||
          msg.phase === TaskPhase.RESETTING ||
          msg.phase === TaskPhase.RECORDING ||
          msg.phase === TaskPhase.SAVING ||
          msg.phase === TaskPhase.INFERENCING;

        // ROS message to React state
        dispatch(
          setTaskStatus({
            robotType: msg.robot_type || '',
            taskName: msg.task_info?.task_name || 'idle',
            running: isRunning,
            phase: msg.phase || 0,
            progress: Math.round(progress),
            totalTime: msg.total_time || 0,
            proceedTime: msg.proceed_time || 0,
            currentEpisodeNumber: msg.current_episode_number || 0,
            userId: msg.task_info?.user_id || '',
            usedStorageSize: msg.used_storage_size || 0,
            totalStorageSize: msg.total_storage_size || 0,
            usedCpu: msg.used_cpu || 0,
            usedRamSize: msg.used_ram_size || 0,
            totalRamSize: msg.total_ram_size || 0,
            error: msg.error || '',
            topicReceived: true,
          })
        );

        // Extract TaskInfo from TaskStatus message
        if (msg.task_info) {
          // update task info only when task is not stopped
          dispatch(
            setTaskInfo({
              taskName: msg.task_info.task_name || '',
              taskType: msg.task_info.task_type || '',
              taskInstruction: msg.task_info.task_instruction || '',
              policyPath: msg.task_info.policy_path || '',
              recordInferenceMode: msg.task_info.record_inference_mode || false,
              userId: msg.task_info.user_id || '',
              fps: msg.task_info.fps || 0,
              tags: msg.task_info.tags || [],
              warmupTime: msg.task_info.warmup_time_s || 0,
              episodeTime: msg.task_info.episode_time_s || 0,
              resetTime: msg.task_info.reset_time_s || 0,
              numEpisodes: msg.task_info.num_episodes || 0,
              pushToHub: msg.task_info.push_to_hub || false,
              privateMode: msg.task_info.private_mode || false,
              useOptimizedSave: msg.task_info.use_optimized_save_mode || false,
            })
          );
        }
      });
    } catch (error) {
      console.error('Failed to subscribe to task status topic:', error);
    }
  }, [dispatch, rosbridgeUrl]);

  const subscribeToHeartbeat = useCallback(async () => {
    try {
      const ros = await rosConnectionManager.getConnection(rosbridgeUrl);
      if (!ros) return;

      // Skip if already subscribed
      if (heartbeatTopicRef.current) {
        console.log('Heartbeat already subscribed, skipping...');
        return;
      }

      heartbeatTopicRef.current = new ROSLIB.Topic({
        ros,
        name: '/heartbeat',
        messageType: 'std_msgs/msg/Empty',
      });

      heartbeatTopicRef.current.subscribe(() => {
        dispatch(setHeartbeatStatus('connected'));
        dispatch(setLastHeartbeatTime(Date.now()));
      });

      console.log('Heartbeat subscription established');
    } catch (error) {
      console.error('Failed to subscribe to heartbeat topic:', error);
    }
  }, [dispatch, rosbridgeUrl]);

  // Start connection and subscription
  useEffect(() => {
    if (!rosbridgeUrl) return;

    const initializeSubscriptions = async () => {
      // Cleanup previous subscriptions before creating new ones
      cleanup();

      try {
        await subscribeToTaskStatus();
        await subscribeToHeartbeat();
      } catch (error) {
        console.error('Failed to initialize ROS subscriptions:', error);
      }
    };

    initializeSubscriptions();

    return cleanup;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rosbridgeUrl]); // Only rosbridgeUrl as dependency to prevent unnecessary re-subscriptions

  // Helper function to get phase name
  const getPhaseName = useCallback((phase) => {
    const phaseNames = {
      [TaskPhase.READY]: 'NONE',
      [TaskPhase.WARMING_UP]: 'WARMING_UP',
      [TaskPhase.RESETTING]: 'RESETTING',
      [TaskPhase.RECORDING]: 'RECORDING',
      [TaskPhase.SAVING]: 'SAVING',
      [TaskPhase.STOPPED]: 'STOPPED',
      [TaskPhase.INFERENCING]: 'INFERENCING',
    };
    return phaseNames[phase] || 'UNKNOWN';
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
    connected,
    subscribeToTaskStatus,
    cleanup,
    getPhaseName,
    resetTaskToIdle,
  };
}
