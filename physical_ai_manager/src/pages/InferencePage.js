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

import React, { useState, useEffect, useCallback } from 'react';
import clsx from 'clsx';
import { MdKeyboardDoubleArrowLeft, MdKeyboardDoubleArrowRight } from 'react-icons/md';
import toast, { useToasterStore } from 'react-hot-toast';
import ImageGrid from '../components/ImageGrid';
import ControlPanel from '../components/ControlPanel';
import InferencePanel from '../components/InferencePanel';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import TaskPhase from '../constants/taskPhases';

export default function InferencePage({
  topics,
  setTopics,
  rosHost,
  taskStatus: propsTaskStatus,
  taskInfo: propsTaskInfo,
  isActive = true,
}) {
  const rosbridgeUrl = `ws://${rosHost.split(':')[0]}:9090`;

  // Toast limit implementation using useToasterStore
  const { toasts } = useToasterStore();
  const TOAST_LIMIT = 3;

  // Use taskStatus and taskInfo from props (received from App.js)
  const taskStatus = propsTaskStatus;
  const taskInfo = propsTaskInfo;

  const [info, setInfo] = useState(
    { ...taskInfo, taskType: 'inference' } || { taskType: 'inference' }
  );
  const [episodeStatus, setEpisodeStatus] = useState(taskStatus);
  const [isRightPanelCollapsed, setIsRightPanelCollapsed] = useState(false);
  const [isTaskStatusPaused, setIsTaskStatusPaused] = useState(false);
  const [lastTaskStatusUpdate, setLastTaskStatusUpdate] = useState(Date.now());

  useEffect(() => {
    toasts
      .filter((t) => t.visible) // Only consider visible toasts
      .filter((_, i) => i >= TOAST_LIMIT) // Is toast index over limit?
      .forEach((t) => toast.dismiss(t.id)); // Dismiss – Use toast.remove(t.id) for no exit animation
  }, [toasts]);

  // Update episodeStatus when taskStatus changes
  useEffect(() => {
    if (taskStatus) {
      setEpisodeStatus(taskStatus);

      if (taskStatus.error !== '') {
        toast.error(`${taskStatus.error}`);
      }
    }
  }, [taskStatus]);

  useEffect(() => {
    if (taskStatus.robotType !== '') {
      setInfo({ ...taskInfo, tags: [taskStatus.robotType, 'robotis'], taskType: 'inference' });
    }
  }, [taskStatus, taskInfo]);

  const { sendRecordCommand } = useRosServiceCaller(rosbridgeUrl);

  // Memoize the onChange handler to prevent recreation on every render
  const handleInfoChange = useCallback((newInfo) => {
    setInfo(newInfo);
  }, []);

  // Validation function for required fields
  const validateTaskInfo = (taskInfo) => {
    const requiredFieldsForRecordInferenceMode = [
      { key: 'taskName', label: 'Task Name' },
      { key: 'taskType', label: 'Task Type' },
      { key: 'taskInstruction', label: 'Task Instruction' },
      { key: 'recordInferenceMode', label: 'Record Inference Mode' },
      { key: 'policyPath', label: 'Policy Path' },
      { key: 'userId', label: 'User ID' },
      { key: 'fps', label: 'FPS' },
      { key: 'warmupTime', label: 'Warmup Time' },
      { key: 'episodeTime', label: 'Episode Time' },
      { key: 'resetTime', label: 'Reset Time' },
      { key: 'numEpisodes', label: 'Num Episodes' },
    ];

    const requiredFieldsForInferenceOnly = [
      { key: 'taskType', label: 'Task Type' },
      { key: 'taskInstruction', label: 'Task Instruction' },
      { key: 'policyPath', label: 'Policy Path' },
      { key: 'recordInferenceMode', label: 'Record Inference Mode' },
    ];

    const requiredFields = taskInfo.recordInferenceMode
      ? requiredFieldsForRecordInferenceMode
      : requiredFieldsForInferenceOnly;

    const missingFields = [];

    for (const field of requiredFields) {
      const value = taskInfo[field.key];

      // Check if field is empty or invalid
      if (
        value === null ||
        value === undefined ||
        value === '' ||
        (typeof value === 'string' && value.trim() === '') ||
        (typeof value === 'number' && (isNaN(value) || value <= 0))
      ) {
        missingFields.push(field.label);
      }
    }

    if (taskInfo.userId === 'Select User ID') {
      missingFields.push('User ID');
    }

    return {
      isValid: missingFields.length === 0,
      missingFields,
    };
  };

  const handleControlCommand = async (cmd) => {
    console.log('Control command received:', cmd);
    let result;

    try {
      // Execute the appropriate command
      if (cmd === 'Start') {
        // Validate info before starting
        const validation = validateTaskInfo(info);
        if (!validation.isValid) {
          toast.error(`Missing required fields: ${validation.missingFields.join(', ')}`);
          console.error('Validation failed. Missing fields:', validation.missingFields);
          return;
        }
        result = await sendRecordCommand('start_inference', info);
      } else if (cmd === 'Stop') {
        result = await sendRecordCommand('stop', info);
      } else if (cmd === 'Retry') {
        result = await sendRecordCommand('rerecord', info);
      } else if (cmd === 'Next') {
        result = await sendRecordCommand('next', info);
      } else if (cmd === 'Finish') {
        result = await sendRecordCommand('finish', info);
      } else {
        console.warn(`Unknown command: ${cmd}`);
        toast.error(`Unknown command: ${cmd}`);
        return;
      }

      console.log('Service call result:', result);

      // Handle service response
      if (result && result.success === false) {
        toast.error(`Command failed: ${result.message || 'Unknown error'}`);
        console.error(`Command '${cmd}' failed:`, result.message);
      } else if (result && result.success === true) {
        toast.success(`Command [${cmd}] executed successfully`);
        console.log(`Command '${cmd}' executed successfully`);

        // Task status will be updated automatically from ROS
      } else {
        // Handle case where result is undefined or doesn't have success field
        console.warn(`Unexpected result format for command '${cmd}':`, result);
        toast.error(`Command [${cmd}] completed with uncertain status`);
      }
    } catch (error) {
      console.error('Error handling control command:', error);

      // Show more specific error messages
      let errorMessage = error.message || error.toString();
      if (
        errorMessage.includes('ROS connection failed') ||
        errorMessage.includes('ROS connection timeout') ||
        errorMessage.includes('WebSocket')
      ) {
        toast.error(`🔌 ROS connection failed: rosbridge server is not running (${rosHost})`);
      } else if (errorMessage.includes('timeout')) {
        toast.error(`⏰ Command execution timeout [${cmd}]: Server did not respond`);
      } else {
        toast.error(`❌ Command execution failed [${cmd}]: ${errorMessage}`);
      }

      // Continue execution even after error - don't block UI
      console.log(`Continuing after error in command '${cmd}'`);
    }
  };

  // track task status update
  useEffect(() => {
    if (taskStatus) {
      setLastTaskStatusUpdate(Date.now());
      setIsTaskStatusPaused(false);
    }
  }, [taskStatus]);

  // Check if task status updates are paused (considered paused if no updates for 1 second)
  useEffect(() => {
    const UPDATE_PAUSE_THRESHOLD = 1000;
    const timer = setInterval(() => {
      const timeSinceLastUpdate = Date.now() - lastTaskStatusUpdate;
      setIsTaskStatusPaused(timeSinceLastUpdate >= UPDATE_PAUSE_THRESHOLD);
    }, 1000);

    return () => clearInterval(timer);
  }, [lastTaskStatusUpdate]);

  const classMainContainer = 'h-full flex flex-col overflow-hidden';
  const classContentsArea = 'flex-1 flex min-h-0 pt-0 px-0 justify-center items-start';
  const classImageGridContainer = clsx(
    'transition-all',
    'duration-300',
    'ease-in-out',
    'flex',
    'items-center',
    'justify-center',
    'min-h-0',
    'h-full',
    'overflow-hidden',
    'm-2',
    {
      'flex-[12]': isRightPanelCollapsed,
      'flex-[10]': !isRightPanelCollapsed,
    }
  );

  const classRightPanelArea = clsx(
    'h-full',
    'w-full',
    'transition-all',
    'duration-300',
    'ease-in-out',
    'relative',
    'overflow-scroll',
    {
      'flex-[0_0_40px]': isRightPanelCollapsed,
      'flex-[1]': !isRightPanelCollapsed,
      'min-w-[60px]': isRightPanelCollapsed,
      'min-w-[400px]': !isRightPanelCollapsed,
      'max-w-[60px]': isRightPanelCollapsed,
      'max-w-[400px]': !isRightPanelCollapsed,
    }
  );

  const classHideButton = clsx(
    'absolute',
    'top-3',
    'bg-white',
    'border',
    'border-gray-300',
    'rounded-full',
    'w-12',
    'h-12',
    'flex',
    'items-center',
    'justify-center',
    'shadow-md',
    'hover:bg-gray-50',
    'transition-all',
    'duration-200',
    'z-10',
    {
      'left-2': isRightPanelCollapsed,
      'left-[10px]': !isRightPanelCollapsed,
    }
  );

  const classRightPanel = clsx(
    'h-full',
    'flex',
    'flex-col',
    'items-center',
    'overflow-hidden',
    'transition-opacity',
    'duration-300',
    'overflow-scroll',
    {
      'opacity-0': isRightPanelCollapsed,
      'opacity-100': !isRightPanelCollapsed,
      'pointer-events-none': isRightPanelCollapsed,
      'pointer-events-auto': !isRightPanelCollapsed,
    }
  );

  const classRobotTypeContainer = clsx(
    'absolute',
    'top-4',
    'left-4',
    'z-20',
    'flex',
    'flex-row',
    'items-center',
    'bg-white/90',
    'backdrop-blur-sm',
    'rounded-full',
    'px-3',
    'py-1',
    'shadow-md',
    'border',
    'border-gray-100'
  );
  const classRobotType = clsx('ml-2 mr-1 my-2 text-gray-600 text-lg');
  const classRobotTypeValue = clsx(
    'mx-1 my-2 px-2 text-lg text-blue-600 focus:outline-none bg-blue-100 rounded-full'
  );

  return (
    <div className={classMainContainer}>
      <div className={classContentsArea}>
        <div className="w-full h-full flex flex-col relative">
          <div className={classRobotTypeContainer}>
            <div className={classRobotType}>Robot Type</div>
            <div className={classRobotTypeValue}>{taskStatus?.robotType}</div>
          </div>
          <div className={classImageGridContainer}>
            <ImageGrid
              topics={topics}
              setTopics={setTopics}
              rosHost={rosHost}
              isActive={isActive}
            />
          </div>
        </div>
        <div className={classRightPanelArea}>
          <button
            onClick={() => setIsRightPanelCollapsed(!isRightPanelCollapsed)}
            className={classHideButton}
            title="Hide"
          >
            <span className="text-gray-600 text-3xl transition-transform duration-200">
              {isRightPanelCollapsed ? (
                <MdKeyboardDoubleArrowLeft />
              ) : (
                <MdKeyboardDoubleArrowRight />
              )}
            </span>
          </button>
          <div className={classRightPanel}>
            <div className="w-full min-h-10"></div>
            <InferencePanel
              info={info}
              onChange={handleInfoChange}
              disabled={taskStatus?.phase !== TaskPhase.READY || !isTaskStatusPaused}
              rosHost={rosHost}
            />
          </div>
        </div>
      </div>
      <ControlPanel
        onCommand={handleControlCommand}
        episodeStatus={episodeStatus}
        taskInfo={taskInfo}
        page="inference"
      />
    </div>
  );
}
