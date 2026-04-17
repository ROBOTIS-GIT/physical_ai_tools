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
  setJoystickMode,
  setRecordingMonitor,
} from '../features/tasks/taskSlice';
import {
  setIsTraining,
  setTopicReceived,
  setTrainingInfo,
  setCurrentStep,
  setLastUpdate,
  setSelectedUser,
  setSelectedDataset,
  setCurrentLoss,
} from '../features/training/trainingSlice';
import {
  setHFStatus,
  setDownloadStatus,
  setHFUserId,
  setHFRepoIdUpload,
  setHFRepoIdDownload,
  setUploadStatus,
} from '../features/editDataset/editDatasetSlice';
import HFStatus from '../constants/HFStatus';
import store from '../store/store';
import rosConnectionManager from '../utils/rosConnectionManager';

export function useRosTopicSubscription() {
  const taskStatusTopicRef = useRef(null);
  const heartbeatTopicRef = useRef(null);
  const trainingStatusTopicRef = useRef(null);
  const previousPhaseRef = useRef(null);
  const audioContextRef = useRef(null);
  const hfStatusTopicRef = useRef(null);
  const actionEventTopicRef = useRef(null);
  const recordingMonitorTopicRef = useRef(null);
  const joystickModeTopicRef = useRef(null);
  const isInitializingRef = useRef(false);
  // One-shot guard so the backend's task_info echo only seeds redux on the
  // first message; subsequent echoes would clobber whatever the user is
  // currently typing in InfoPanel.
  const initialTaskInfoSyncRef = useRef(false);
  // Dedup task-status error toasts so a sticky error field doesn't spam.
  const previousTaskStatusErrorRef = useRef('');

  const dispatch = useDispatch();
  const rosbridgeUrl = useSelector((state) => state.ros.rosbridgeUrl);
  const [connected, setConnected] = useState(false);

  const initializeAudioContext = useCallback(() => {
    if (!audioContextRef.current) {
      audioContextRef.current = new (window.AudioContext || window.webkitAudioContext)();
    }
    return audioContextRef.current;
  }, []);

  const playBeep = useCallback(
    async (frequency = 1000, duration = 400) => {
      const INITIAL_GAIN = 1.0;
      const FINAL_GAIN = 0.01;
      const FALLBACK_VIBRATION_PATTERN = [200, 100, 200];

      try {
        const audioContext = initializeAudioContext();

        if (audioContext.state === 'suspended') {
          await audioContext.resume();
        }

        const oscillator = audioContext.createOscillator();
        const gainNode = audioContext.createGain();

        oscillator.connect(gainNode);
        gainNode.connect(audioContext.destination);

        oscillator.frequency.value = frequency;
        oscillator.type = 'sine';

        gainNode.gain.setValueAtTime(INITIAL_GAIN, audioContext.currentTime);
        gainNode.gain.exponentialRampToValueAtTime(
          FINAL_GAIN,
          audioContext.currentTime + duration / 1000
        );

        oscillator.start(audioContext.currentTime);
        oscillator.stop(audioContext.currentTime + duration / 1000);

        console.log('🔊 Beep played successfully');
      } catch (error) {
        console.warn('Audio playback failed:', error);
        try {
          if (window.navigator && window.navigator.vibrate) {
            window.navigator.vibrate(FALLBACK_VIBRATION_PATTERN);
            console.log('📳 Fallback to vibration');
          }
        } catch (vibrationError) {
          console.warn('Vibration fallback also failed:', vibrationError);
        }
      }
    },
    [initializeAudioContext]
  );

  const preferredVoiceRef = useRef(null);

  useEffect(() => {
    if (!('speechSynthesis' in window)) return;
    const pickVoice = () => {
      const voices = window.speechSynthesis.getVoices();
      if (voices.length === 0) return;
      preferredVoiceRef.current = voices.find(v =>
        v.lang.startsWith('en') && v.name.toLowerCase().includes('female')
      ) || voices.find(v =>
        v.lang.startsWith('en') && /samantha|karen|victoria|fiona|moira|tessa/i.test(v.name)
      ) || voices.find(v =>
        v.lang.startsWith('en-US') && !v.name.toLowerCase().includes('male')
      ) || null;
      if (preferredVoiceRef.current) {
        console.log('TTS voice selected:', preferredVoiceRef.current.name);
      }
    };
    pickVoice();
    window.speechSynthesis.addEventListener('voiceschanged', pickVoice);
    return () => window.speechSynthesis.removeEventListener('voiceschanged', pickVoice);
  }, []);

  const speakText = useCallback(
    (text) => {
      try {
        if ('speechSynthesis' in window) {
          window.speechSynthesis.cancel();
          window.speechSynthesis.resume();
          const utterance = new SpeechSynthesisUtterance(text);
          utterance.lang = 'en-US';
          utterance.rate = 1.1;
          utterance.pitch = 1.1;
          utterance.volume = 1.0;
          if (preferredVoiceRef.current) utterance.voice = preferredVoiceRef.current;
          window.speechSynthesis.speak(utterance);
          console.log(`Speech: "${text}"`);
        } else {
          console.warn('Speech synthesis not available, falling back to beep');
          playBeep();
        }
      } catch (error) {
        console.warn('Speech failed, falling back to beep:', error);
        playBeep();
      }
    },
    [playBeep]
  );

  // Helper function to unsubscribe from a topic
  const unsubscribeFromTopic = useCallback((topicRef, topicName) => {
    if (topicRef.current) {
      topicRef.current.unsubscribe();
      topicRef.current = null;
      console.log(`${topicName} topic unsubscribed`);
    }
  }, []);

  const cleanup = useCallback(() => {
    console.log('Starting ROS subscriptions cleanup...');

    // Unsubscribe from all topics
    unsubscribeFromTopic(taskStatusTopicRef, 'Task status');
    unsubscribeFromTopic(heartbeatTopicRef, 'Heartbeat');
    unsubscribeFromTopic(trainingStatusTopicRef, 'Training status');
    unsubscribeFromTopic(hfStatusTopicRef, 'HF status');
    unsubscribeFromTopic(actionEventTopicRef, 'Action event');
    unsubscribeFromTopic(recordingMonitorTopicRef, 'Recording monitor');
    unsubscribeFromTopic(joystickModeTopicRef, 'Joystick mode');

    // Reset previous phase tracking
    previousPhaseRef.current = null;

    if (audioContextRef.current && audioContextRef.current.state !== 'closed') {
      audioContextRef.current.close();
      audioContextRef.current = null;
    }

    setConnected(false);
    dispatch(setHeartbeatStatus('disconnected'));
    console.log('ROS task status cleanup completed');
  }, [dispatch, unsubscribeFromTopic]);

  useEffect(() => {
    const enableAudioOnUserGesture = () => {
      const audioContext = initializeAudioContext();
      if (audioContext.state === 'suspended') {
        audioContext
          .resume()
          .then(() => {
            console.log('🎵 Audio enabled by user gesture');
          })
          .catch((error) => {
            console.warn('Failed to resume AudioContext on user gesture:', error);
          });
      }
    };

    const events = ['touchstart', 'touchend', 'mousedown', 'keydown', 'click'];
    events.forEach((event) => {
      document.addEventListener(event, enableAudioOnUserGesture, { once: true, passive: true });
    });

    return () => {
      events.forEach((event) => {
        document.removeEventListener(event, enableAudioOnUserGesture);
      });
    };
  }, [initializeAudioContext]);

  const subscribeToTaskStatus = useCallback(async () => {
    try {
      const RECORDING_BEEP_FREQUENCY = 1000;
      const RECORDING_BEEP_DURATION = 400;
      const BEEP_DELAY = 100;

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

        let progress = 0;

        // Show the toast once when a new error appears, but DO NOT bail out:
        // we still need to dispatch setTaskStatus below so the phase update
        // (e.g. backend resetting LOADING → READY after a failed setup) is
        // reflected in the UI. Returning early used to leave the panel
        // stuck in LOADING / Read-only forever.
        if (msg.error && msg.error !== previousTaskStatusErrorRef.current) {
          console.log('error:', msg.error);
          toast.error(msg.error);
        }
        previousTaskStatusErrorRef.current = msg.error || '';

        const currentPhase = msg.phase;
        const previousPhase = previousPhaseRef.current;

        if (currentPhase === TaskPhase.RECORDING && previousPhase !== TaskPhase.RECORDING) {
          console.log('Recording started - playing beep sound');

          setTimeout(() => {
            playBeep(RECORDING_BEEP_FREQUENCY, RECORDING_BEEP_DURATION);
          }, BEEP_DELAY);

          toast.success('Recording started!');
        }

        previousPhaseRef.current = currentPhase;

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
        // Preserve existing robotType when backend sends empty value
        const incomingRobotType = msg.robot_type && msg.robot_type.trim() !== ''
          ? msg.robot_type
          : store.getState().tasks.taskStatus.robotType;
        dispatch(
          setTaskStatus({
            robotType: incomingRobotType,
            taskName: msg.task_info?.task_name || 'idle',
            running: isRunning,
            phase: msg.phase || 0,
            progress: Math.round(progress),
            totalTime: msg.total_time || 0,
            proceedTime: msg.proceed_time || 0,
            currentEpisodeNumber: msg.current_episode_number || 0,
            currentScenarioNumber: msg.current_scenario_number || 0,
            currentTaskInstruction: msg.current_task_instruction || '',
            currentSegmentIndex: msg.current_segment_index || 0,
            segmentCount: msg.segment_count || 0,
            segmentPrimitives: msg.segment_primitives || [],
            mergeStatus: msg.merge_status || 'none',
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

        // Extract TaskInfo from TaskStatus message.
        // Skip update when server sends empty task_info (e.g. during
        // LOADING, PAUSED, INFERENCING) to preserve user-entered values.
        const hasTaskInfo = msg.task_info && (
          (msg.task_info.task_name && msg.task_info.task_name.length > 0) ||
          (msg.task_info.policy_path && msg.task_info.policy_path.length > 0)
        );
        // Only seed once: subsequent echoes would overwrite user edits.
        if (hasTaskInfo && !initialTaskInfoSyncRef.current) {
          initialTaskInfoSyncRef.current = true;
          dispatch(
            setTaskInfo({
              taskNum: msg.task_info.task_num || '',
              taskName: msg.task_info.task_name || '',
              taskType: msg.task_info.task_type || '',
              taskInstruction: msg.task_info.task_instruction || [],
              policyPath: msg.task_info.policy_path || '',
              recordInferenceMode: msg.task_info.record_inference_mode || false,
              userId: msg.task_info.user_id || '',
              controlHz: msg.task_info.control_hz || 0,
              warmupTime: msg.task_info.warmup_time_s || 0,
              episodeTime: msg.task_info.episode_time_s || 0,
              resetTime: msg.task_info.reset_time_s || 0,
              numEpisodes: msg.task_info.num_episodes || 0,
              pushToHub: msg.task_info.push_to_hub || false,
              privateMode: msg.task_info.private_mode || false,
              useOptimizedSave: msg.task_info.use_optimized_save_mode || false,
              recordRosBag2: msg.task_info.record_rosbag2 || false,
            })
          );
        }

      });
    } catch (error) {
      console.error('Failed to subscribe to task status topic:', error);
    }
  }, [dispatch, rosbridgeUrl, playBeep]);

  const subscribeToActionEvent = useCallback(async () => {
    try {
      const VOICE_DELAY = 100;
      const ACTION_VOICE_MAP = {
        start: 'Recording started',
        finish: 'Recording finished',
        cancel: 'Cancelled',
        review_on: 'Previous data needs review',
        review_off: 'Previous data review cleared',
      };

      const ros = await rosConnectionManager.getConnection(rosbridgeUrl);
      if (!ros) return;

      if (actionEventTopicRef.current) {
        console.log('Action event already subscribed, skipping...');
        return;
      }

      actionEventTopicRef.current = new ROSLIB.Topic({
        ros,
        name: '/task/action_event',
        messageType: 'std_msgs/msg/String',
      });

      actionEventTopicRef.current.subscribe((msg) => {
        const action = msg.data;
        console.log('Received action event:', action);

        const voiceText = ACTION_VOICE_MAP[action];
        if (voiceText) {
          setTimeout(() => {
            speakText(voiceText);
          }, VOICE_DELAY);
        }
      });

      console.log('Action event subscription established');
    } catch (error) {
      console.error('Failed to subscribe to action event topic:', error);
    }
  }, [rosbridgeUrl, speakText]);

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

  const subscribeToTrainingStatus = useCallback(async () => {
    try {
      const ros = await rosConnectionManager.getConnection(rosbridgeUrl);
      if (!ros) return;

      // Skip if already subscribed
      if (trainingStatusTopicRef.current) {
        console.log('Training status already subscribed, skipping...');
        return;
      }

      setConnected(true);
      trainingStatusTopicRef.current = new ROSLIB.Topic({
        ros,
        name: '/training/status',
        messageType: 'physical_ai_interfaces/msg/TrainingStatus',
      });

      trainingStatusTopicRef.current.subscribe((msg) => {
        console.log('Received training status:', msg);

        if (msg.error !== '') {
          console.log('error:', msg.error);
          toast.error(msg.error);
          return;
        }

        // ROS message to React state
        dispatch(
          setTrainingInfo({
            datasetRepoId: msg.training_info.dataset || '',
            policyType: msg.training_info.policy_type || '',
            policyDevice: msg.training_info.policy_device || '',
            outputFolderName: msg.training_info.output_folder_name || '',
            resume: msg.training_info.resume || false,
            seed: msg.training_info.seed || 0,
            numWorkers: msg.training_info.num_workers || 0,
            batchSize: msg.training_info.batch_size || 0,
            steps: msg.training_info.steps || 0,
            evalFreq: msg.training_info.eval_freq || 0,
            logFreq: msg.training_info.log_freq || 0,
            saveFreq: msg.training_info.save_freq || 0,
          })
        );

        const datasetParts = msg.training_info.dataset.split('/');
        dispatch(setSelectedUser(datasetParts[0] || ''));
        dispatch(setSelectedDataset(datasetParts[1] || ''));
        dispatch(setIsTraining(msg.is_training));
        dispatch(setCurrentStep(msg.current_step || 0));
        dispatch(setCurrentLoss(msg.current_loss));
        dispatch(setTopicReceived(true));
        dispatch(setLastUpdate(Date.now()));
      });
    } catch (error) {
      console.error('Failed to subscribe to training status topic:', error);
    }
  }, [dispatch, rosbridgeUrl]);

  const subscribeHFStatus = useCallback(async () => {
    try {
      const ros = await rosConnectionManager.getConnection(rosbridgeUrl);
      if (!ros) return;

      // Skip if already subscribed
      if (hfStatusTopicRef.current) {
        console.log('HF status already subscribed, skipping...');
        return;
      }

      hfStatusTopicRef.current = new ROSLIB.Topic({
        ros,
        name: '/huggingface/status',
        messageType: 'physical_ai_interfaces/msg/HFOperationStatus',
      });

      hfStatusTopicRef.current.subscribe((msg) => {
        console.log('Received HF status:', msg);

        const status = msg.status;
        const operation = msg.operation;
        const repoId = msg.repo_id;
        // const localPath = msg.local_path;
        const message = msg.message;
        const progressCurrent = msg.progress_current;
        const progressTotal = msg.progress_total;
        const progressPercentage = msg.progress_percentage;

        if (status === 'Failed') {
          toast.error(message);
        } else if (status === 'Success') {
          toast.success(message);
        }

        console.log('status:', status);

        // Check the current status from the store
        const currentStatus = store.getState().editDataset.hfStatus;

        if (
          (currentStatus === HFStatus.SUCCESS || currentStatus === HFStatus.FAILED) &&
          status === HFStatus.IDLE
        ) {
          console.log('Maintaining SUCCESS status, skipping IDLE update');
          // Skip updating the status
        } else {
          console.log('Updating HF status to:', status);
          dispatch(setHFStatus(status));
        }

        if (operation === 'upload') {
          dispatch(
            setUploadStatus({
              current: progressCurrent,
              total: progressTotal,
              percentage: progressPercentage.toFixed(2),
            })
          );
        } else if (operation === 'download') {
          dispatch(
            setDownloadStatus({
              current: progressCurrent,
              total: progressTotal,
              percentage: progressPercentage.toFixed(2),
            })
          );
        }
        const userId = repoId.split('/')[0];
        const repoName = repoId.split('/')[1];

        if (userId?.trim() && repoName?.trim()) {
          dispatch(setHFUserId(userId));

          if (operation === 'upload') {
            dispatch(setHFRepoIdUpload(repoName));
          } else if (operation === 'download') {
            dispatch(setHFRepoIdDownload(repoName));
          }
        }
      });

      console.log('HF status subscription established');
    } catch (error) {
      console.error('Failed to subscribe to HF status topic:', error);
    }
  }, [dispatch, rosbridgeUrl]);

  // Per-topic recording monitor (1 Hz while recording is active).
  const subscribeToRecordingMonitor = useCallback(async () => {
    try {
      const ros = await rosConnectionManager.getConnection(rosbridgeUrl);
      if (!ros) return;
      if (recordingMonitorTopicRef.current) return;

      recordingMonitorTopicRef.current = new ROSLIB.Topic({
        ros,
        name: '/rosbag_recorder/monitor',
        messageType: 'rosbag_recorder/msg/RecordingMonitor',
      });

      recordingMonitorTopicRef.current.subscribe((msg) => {
        // rosbridge serialises uint8[] as a base64 string; decode it back
        // to a plain numeric array so status[i] gives 0/1/2 not a character.
        let statusArr = msg.status || [];
        if (typeof statusArr === 'string') {
          const bin = atob(statusArr);
          statusArr = Array.from(bin, (ch) => ch.charCodeAt(0));
        }

        const topics = (msg.topic_names || []).map((name, i) => ({
          name,
          rateHz: msg.rates_hz?.[i] ?? 0,
          baselineHz: msg.baseline_hz?.[i] ?? 0,
          secondsSinceLast: msg.seconds_since_last?.[i] ?? -1,
          status: statusArr[i] ?? 0,
        }));
        dispatch(setRecordingMonitor({
          topics,
          totalReceived: msg.total_received || 0,
          totalWritten: msg.total_written || 0,
        }));
      });

      console.log('Recording monitor subscription established');
    } catch (error) {
      console.error('Failed to subscribe to recording monitor topic:', error);
    }
  }, [dispatch, rosbridgeUrl]);

  // Leader joystick operating mode (fires only on button press, not continuous).
  const subscribeToJoystickMode = useCallback(async () => {
    try {
      const ros = await rosConnectionManager.getConnection(rosbridgeUrl);
      if (!ros) return;
      if (joystickModeTopicRef.current) return;

      joystickModeTopicRef.current = new ROSLIB.Topic({
        ros,
        name: '/leader/joystick_controller_right/joystick_mode',
        messageType: 'std_msgs/msg/String',
      });

      joystickModeTopicRef.current.subscribe((msg) => {
        dispatch(setJoystickMode(msg.data));
      });

      console.log('Joystick mode subscription established');
    } catch (error) {
      console.error('Failed to subscribe to joystick mode topic:', error);
    }
  }, [dispatch, rosbridgeUrl]);

  // Manual initialization function
  const initializeSubscriptions = useCallback(async () => {
    if (!rosbridgeUrl) {
      console.warn('Cannot initialize subscriptions: rosbridgeUrl is not set');
      return;
    }

    if (isInitializingRef.current) {
      console.log('ROS subscriptions initialization already in progress, skipping...');
      return;
    }

    isInitializingRef.current = true;

    console.log('Manually initializing ROS subscriptions...');

    // Cleanup previous subscriptions before creating new ones
    cleanup();

    try {
      await subscribeToTaskStatus();
      await subscribeToHeartbeat();
      await subscribeToActionEvent();
      await subscribeToTrainingStatus();
      await subscribeHFStatus();
      await subscribeToRecordingMonitor();
      await subscribeToJoystickMode();
      console.log('ROS subscriptions initialized successfully');
    } catch (error) {
      console.error('Failed to initialize ROS subscriptions:', error);
    } finally {
      isInitializingRef.current = false;
    }
  }, [
    rosbridgeUrl,
    cleanup,
    subscribeToTaskStatus,
    subscribeToHeartbeat,
    subscribeToActionEvent,
    subscribeToTrainingStatus,
    subscribeHFStatus,
    subscribeToRecordingMonitor,
    subscribeToJoystickMode,
  ]);

  // Auto-start connection and subscription (can be disabled by not calling useRosTopicSubscription)
  useEffect(() => {
    if (!rosbridgeUrl) return;

    initializeSubscriptions();

    return cleanup;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rosbridgeUrl]); // Only rosbridgeUrl as dependency to prevent unnecessary re-subscriptions

  return {
    connected,
    subscribeToTaskStatus,
    cleanup,
    getPhaseName,
    resetTaskToIdle,
    subscribeToTrainingStatus,
    subscribeHFStatus,
    initializeSubscriptions, // Manual initialization function
  };
}
