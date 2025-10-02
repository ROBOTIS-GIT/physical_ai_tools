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

import React, { useState, useEffect, useRef, useCallback } from 'react';
import { useSelector } from 'react-redux';
import clsx from 'clsx';
import toast, { useToasterStore } from 'react-hot-toast';
import { MdPlayArrow, MdStop, MdReplay, MdSkipNext, MdCheck, MdNavigateNext } from 'react-icons/md';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import CompactSystemStatus from './CompactSystemStatus';
import EpisodeStatus from './EpisodeStatus';
import ProgressBar from './ProgressBar';
import SystemStatus from './SystemStatus';
import Tooltip from './Tooltip';
import PageType from '../constants/pageType';
import TaskPhase from '../constants/taskPhases';
import FullTaskStatus from './FullTaskStatus';

const phaseGuideMessages = {
  [TaskPhase.READY]: '📍 Ready to start',
  [TaskPhase.WARMING_UP]: '🔥 Warmup in progress',
  [TaskPhase.RESETTING]: '🏠 Reset in progress',
  [TaskPhase.RECORDING]: '🔴 Recording in progress',
  [TaskPhase.SAVING]: '💾 Saving...',
  [TaskPhase.STOPPED]: '◼️ Task Stopped',
  [TaskPhase.INFERENCING]: '⏳ Inferencing',
};

const requiredFieldsForRecord = [
  { key: 'taskName', label: 'Task Name' },
  { key: 'taskInstruction', label: 'Task Instruction' },
  { key: 'userId', label: 'User ID' },
  { key: 'fps', label: 'FPS' },
  { key: 'warmupTime', label: 'Warmup Time' },
  { key: 'episodeTime', label: 'Episode Time' },
  { key: 'resetTime', label: 'Reset Time' },
  { key: 'numEpisodes', label: 'Num Episodes' },
];

const requiredFieldsForRecordInferenceMode = [
  { key: 'taskName', label: 'Task Name' },
  { key: 'taskInstruction', label: 'Task Instruction' },
  { key: 'policyPath', label: 'Policy Path' },
  { key: 'userId', label: 'User ID' },
  { key: 'fps', label: 'FPS' },
  { key: 'warmupTime', label: 'Warmup Time' },
  { key: 'episodeTime', label: 'Episode Time' },
  { key: 'resetTime', label: 'Reset Time' },
  { key: 'numEpisodes', label: 'Num Episodes' },
];

const requiredFieldsForInferenceOnly = [
  { key: 'taskInstruction', label: 'Task Instruction' },
  { key: 'policyPath', label: 'Policy Path' },
];

const spinnerFrames = ['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧'];

export default function ControlPanel() {
  const taskInfo = useSelector((state) => state.tasks.taskInfo);
  const taskStatus = useSelector((state) => state.tasks.taskStatus);
  const rosHost = useSelector((state) => state.ros.rosHost);
  const page = useSelector((state) => state.ui.currentPage);
  const useMultiTaskMode = useSelector((state) => state.tasks.useMultiTaskMode);

  const [hovered, setHovered] = useState(null);
  const [pressed, setPressed] = useState(null);
  const [started, setStarted] = useState(false);
  const [expandedSystemIndex, setExpandedSystemIndex] = useState(null);
  const [spinnerIndex, setSpinnerIndex] = useState(0);
  const startedRef = useRef(started);

  const buttons = [
    {
      label: 'Start',
      icon: MdPlayArrow,
      color: '#1976d2',
      description: page === PageType.RECORD ? 'Start recording task' : 'Start inference',
      shortcut: 'Space',
    },
    {
      label: 'Stop',
      icon: MdStop,
      color: '#d32f2f',
      description:
        page === PageType.RECORD
          ? useMultiTaskMode
            ? 'Stop current task'
            : 'Stop and save current episode'
          : '',
      shortcut: 'Space',
    },
    {
      label: 'Skip\nTask',
      icon: MdNavigateNext,
      color: '#388e3c',
      description: page === PageType.RECORD ? 'Skip current task' : '',
      shortcut: 'Ctrl+Shift+N',
    },
    {
      label: 'Retry',
      icon: MdReplay,
      color: '#fbc02d',
      description:
        page === PageType.RECORD
          ? useMultiTaskMode
            ? 'Retry current task'
            : 'Retry current episode'
          : '',
      shortcut: '←',
    },
    {
      label: 'Next',
      icon: MdSkipNext,
      color: '#388e3c',
      description:
        page === PageType.RECORD
          ? useMultiTaskMode
            ? 'Move to next task'
            : 'Move to next episode'
          : '',
      shortcut: '→',
    },
    {
      label: 'Finish',
      icon: MdCheck,
      color: '#388e3c',
      description:
        page === PageType.RECORD
          ? useMultiTaskMode
            ? 'Finish and save'
            : 'Finish and save task'
          : 'Finish inference',
      shortcut: 'Ctrl+Shift+X',
    },
  ];

  const buttonEnabled = {
    Start: true,
    Stop: true,
    Retry: true,
    Next: true,
    Finish: true,
    'Skip\nTask': useMultiTaskMode && page === PageType.RECORD,
  };

  const { sendRecordCommand } = useRosServiceCaller();

  const { toasts } = useToasterStore();
  const TOAST_LIMIT = 3;

  useEffect(() => {
    toasts
      .filter((t) => t.visible) // Only consider visible toasts
      .filter((_, i) => i >= TOAST_LIMIT) // Is toast index over limit?
      .forEach((t) => toast.dismiss(t.id)); // Dismiss – Use toast.remove(t.id) for no exit animation
  }, [toasts]);

  useEffect(() => {
    startedRef.current = started;
  }, [started]);

  // Spinner animation effect - update whenever taskStatus changes (ROS topic received)
  useEffect(() => {
    // Update spinner index whenever taskStatus changes (regardless of phase)
    updateSpinnerFrame();
  }, [taskStatus]);

  const isReadyState = (phase) => {
    return phase === TaskPhase.READY;
  };

  const updateSpinnerFrame = () => {
    setSpinnerIndex((prevIndex) => (prevIndex + 1) % spinnerFrames.length);
  };

  // Check if button should be enabled based on phase
  const isButtonEnabled = useCallback(
    (label) => {
      if (taskStatus.running) {
        if (page === PageType.RECORD) {
          if (taskInfo?.taskType !== 'record' && taskInfo?.taskType !== '') {
            return false;
          } // disable buttons in Record page when inference task is running
        } else if (page === PageType.INFERENCE) {
          if (taskInfo?.taskType !== 'inference' && taskInfo?.taskType !== '') {
            return false;
          } // disable buttons in Inference page when record task is running
        }
      }

      const isRecordTaskType = taskInfo?.taskType === 'record';
      const isInferenceTaskType = taskInfo?.taskType === 'inference';

      switch (label) {
        case 'Start':
          // Start button disabled when task is running or when running flag is true
          return !taskStatus.running;
        case 'Stop':
          if (isInferenceTaskType) {
            return taskStatus.running && taskInfo.recordInferenceMode;
          }
          // Stop button enabled only when task is running
          return taskStatus.running;
        case 'Retry':
          if (isRecordTaskType && useMultiTaskMode) {
            return taskStatus.running;
          }

          if (isInferenceTaskType) {
            return !isReadyState(taskStatus.phase) && taskInfo.recordInferenceMode;
          }
          // Retry button enabled only when task is stopped
          return !isReadyState(taskStatus.phase);
        case 'Next':
          if (isRecordTaskType && useMultiTaskMode) {
            return taskStatus.running;
          }

          if (isInferenceTaskType) {
            return !isReadyState(taskStatus.phase) && taskInfo.recordInferenceMode;
          }
          // Next button enabled only when task is stopped
          return !isReadyState(taskStatus.phase);
        case 'Skip\nTask':
          if (page === PageType.RECORD) {
            return !isReadyState(taskStatus.phase) && taskStatus.running;
          }
          return false;
        case 'Finish':
          // Finish button enabled only when task is stopped
          return true; // Always enabled
        default:
          return false;
      }
    },
    [
      taskStatus.phase,
      taskStatus.running,
      taskInfo.recordInferenceMode,
      taskInfo.taskType,
      page,
      useMultiTaskMode,
    ]
  );

  const validateTaskInfo = useCallback(() => {
    const requiredFields =
      page === PageType.RECORD
        ? requiredFieldsForRecord
        : page === PageType.INFERENCE
        ? taskInfo.recordInferenceMode
          ? requiredFieldsForRecordInferenceMode
          : requiredFieldsForInferenceOnly
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
        (typeof value === 'number' && (isNaN(value) || value <= 0)) ||
        (Array.isArray(value) && value.length === 0) ||
        (Array.isArray(value) && value.every((item) => item.trim() === ''))
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
  }, [taskInfo, page]);

  const handleControlCommand = useCallback(
    async (cmd) => {
      console.log('Control command received:', cmd);
      let result;

      try {
        // Execute the appropriate command
        if (cmd === 'Start') {
          // Validate info before starting
          const validation = validateTaskInfo();
          if (!validation.isValid) {
            toast.error(`Missing required fields: ${validation.missingFields.join(', ')}`);
            console.error('Validation failed. Missing fields:', validation.missingFields);
            return;
          }
          if (page === PageType.RECORD) {
            result = await sendRecordCommand('start_record');
          } else if (page === PageType.INFERENCE) {
            result = await sendRecordCommand('start_inference');
          } else {
            console.warn(`Unknown page: ${page}`);
            toast.error(`Unknown page: ${page}`);
            return;
          }
        } else if (cmd === 'Stop') {
          result = await sendRecordCommand('stop');
        } else if (cmd === 'Retry') {
          result = await sendRecordCommand('rerecord');
        } else if (cmd === 'Next') {
          result = await sendRecordCommand('next');
        } else if (cmd === 'Skip\nTask') {
          result = await sendRecordCommand('skip_task');
        } else if (cmd === 'Finish') {
          result = await sendRecordCommand('finish');
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
    },
    [sendRecordCommand, validateTaskInfo, page, rosHost]
  );

  const handleCommand = useCallback(
    (label) => {
      handleControlCommand(label);
      console.log(label + ' command executed');
      if (label === 'Start') setStarted(true);
      if (label === 'Stop' || label === 'Finish') setStarted(false);
    },
    [handleControlCommand]
  );

  // Helper function to get button label from keyboard event
  const getButtonFromKey = useCallback(
    (e) => {
      if (e.key === 'ArrowLeft' && isButtonEnabled('Retry')) {
        return 'Retry';
      } else if (e.key === 'ArrowRight' && isButtonEnabled('Next')) {
        return 'Next';
      } else if (e.key === ' ' || e.key === 'Spacebar' || e.code === 'Space') {
        if (isButtonEnabled('Start')) {
          return 'Start';
        } else if (isButtonEnabled('Stop')) {
          return 'Stop';
        }
      } else if (
        (e.ctrlKey || e.metaKey) &&
        e.shiftKey &&
        (e.key === 'x' || e.key === 'X') &&
        isButtonEnabled('Finish')
      ) {
        return 'Finish';
      }
      return null;
    },
    [isButtonEnabled]
  );

  // Add keyboard press visual feedback
  const handleKeyboardPress = useCallback((buttonLabel) => {
    if (buttonLabel) {
      setPressed(buttonLabel);
    }
  }, []);

  // Add keyboard release visual feedback
  const handleKeyboardRelease = useCallback(() => {
    setPressed(null);
  }, []);

  useEffect(() => {
    const isInputFocused = () => {
      const activeElement = document.activeElement;
      if (!activeElement) return false;

      const tagName = activeElement.tagName.toLowerCase();
      const isEditable = activeElement.contentEditable === 'true';

      return tagName === 'input' || tagName === 'textarea' || tagName === 'select' || isEditable;
    };

    const handleKeyDown = (e) => {
      // Ignore repeated keydown events when holding the key
      if (e.repeat) return;

      // Ignore keyboard shortcuts when user is typing in input fields
      if (isInputFocused()) return;

      const buttonLabel = getButtonFromKey(e);
      if (buttonLabel) {
        handleKeyboardPress(buttonLabel);
      }
    };

    const handleKeyUp = (e) => {
      // Always release pressed state on keyup
      handleKeyboardRelease();

      // Ignore keyboard shortcuts when user is typing in input fields
      if (isInputFocused()) return;

      // Get the button label and execute the command
      const buttonLabel = getButtonFromKey(e);
      if (buttonLabel) {
        handleCommand(buttonLabel);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [
    handleCommand,
    isButtonEnabled,
    getButtonFromKey,
    handleKeyboardPress,
    handleKeyboardRelease,
  ]);

  const classControlPanelBody = clsx(
    'h-56',
    'bg-gray-300',
    'rounded-3xl',
    'mx-8',
    'mt-2',
    'mb-4',
    'p-4',
    'flex',
    'flex-row',
    'items-center',
    'gap-2',
    'shadow-lg'
  );

  const classControlPanelButtons = (label, isDisabled) =>
    clsx(
      'text-4xl',
      'font-extrabold',
      'w-full',
      'h-full',
      'min-w-0',
      'rounded-2xl',
      'border-none',
      'cursor-pointer',
      'px-2',
      'flex',
      'items-center',
      'justify-center',
      'flex-col',
      'bg-gray-100',
      'transition-all',
      'duration-200',
      'overflow-hidden',
      {
        'bg-gray-300': pressed === label && !isDisabled,
        'bg-gray-200': hovered === label && pressed !== label && !isDisabled,
        'opacity-30': isDisabled,
        'cursor-not-allowed': isDisabled,
        'bg-gray-50': isDisabled,
      }
    );

  const classControlPanelButtonIcon = clsx(
    'bg-transparent',
    'rounded-full',
    'w-20',
    'h-20',
    'flex',
    'items-center',
    'justify-center',
    'mb-1'
  );

  const handleButtonKeyUp = (e, label, isDisabled) => {
    if (isDisabled) return;
    if (e.key === 'Enter') {
      handleCommand(label);
    }
  };

  const handleButtonKeyDown = (e, label, isDisabled) => {
    if (isDisabled) return;
    if (e.key === ' ') {
      e.preventDefault(); // Prevent scrolling
    }
  };

  const handleMouseEnter = (label, isDisabled) => {
    if (!isDisabled) {
      setHovered(label);
    }
  };

  const handleMouseLeave = () => {
    setHovered(null);
    setPressed(null);
  };

  const handleMouseDown = (label, isDisabled) => {
    if (!isDisabled) {
      setPressed(label);
    }
  };

  const handleMouseUp = () => {
    setPressed(null);
  };

  return (
    <div className={classControlPanelBody}>
      <div className="flex flex-[2] w-full h-full gap-4">
        {buttons.map(({ label, icon: Icon, color, description, shortcut }) => {
          const isDisabled = !isButtonEnabled(label);

          const tooltipContent = (
            <div className="text-center">
              <div className="font-semibold text-lg">{description}</div>
              {!isDisabled && (
                <div className="text-md mt-1 text-gray-300">
                  Press <span className="font-mono bg-gray-700 px-1 rounded">{shortcut}</span>
                </div>
              )}
              {isDisabled && <div className="text-xs mt-1 text-red-300">Currently disabled</div>}
            </div>
          );

          if (!buttonEnabled[label]) {
            return null;
          }

          return (
            <Tooltip
              key={label}
              content={tooltipContent}
              disabled={false}
              className="relative h-full flex-1 min-w-0"
            >
              <button
                className={classControlPanelButtons(label, isDisabled)}
                style={{
                  fontFamily: 'Pretendard Variable',
                  fontSize: 'clamp(1rem, 1.5vw, 2.2rem)',
                }}
                tabIndex={isDisabled ? -1 : 0}
                onClick={() => !isDisabled && handleCommand(label)}
                onKeyUp={(e) => handleButtonKeyUp(e, label, isDisabled)}
                onKeyDown={(e) => handleButtonKeyDown(e, label, isDisabled)}
                onMouseEnter={() => handleMouseEnter(label, isDisabled)}
                onMouseLeave={handleMouseLeave}
                onMouseDown={() => handleMouseDown(label, isDisabled)}
                onMouseUp={handleMouseUp}
                disabled={isDisabled}
              >
                <span className="h-[30%] w-full flex items-center justify-center"></span>
                <span className={classControlPanelButtonIcon}>
                  <Icon
                    style={{ fontSize: 'clamp(1rem, 4vw, 4rem)' }}
                    color={isDisabled ? '#9ca3af' : color}
                  />
                </span>
                <span className="text-center whitespace-pre-line leading-tight text-ellipsis overflow-hidden block w-full h-full flex items-center justify-center">
                  {label}
                </span>
              </button>
            </Tooltip>
          );
        })}
      </div>
      <div className="w-full h-full rounded-2xl flex flex-1 flex-col justify-center items-center gap-2">
        <div className="flex items-center justify-center gap-5">
          <div
            className="flex min-w-0 text-center items-center gap-2"
            style={{ fontSize: 'clamp(1rem, 2vw, 2.5rem)' }}
          >
            {phaseGuideMessages[taskStatus.phase]}
          </div>
          <div>
            {/* Spinner */}
            {taskStatus.running && (
              <span className="font-mono text-blue-500 text-4xl">
                {spinnerFrames[spinnerIndex]}
              </span>
            )}
          </div>
        </div>
        {!useMultiTaskMode && (
          <div className="w-full flex flex-col items-center gap-1">
            <div className="w-full max-w-xl flex flex-col items-center gap-1">
              <div className="flex px-3 w-full justify-end text-xl text-gray-500 font-bold whitespace-nowrap ">
                {taskStatus.proceedTime} / {taskStatus.totalTime} (s)
              </div>
              <ProgressBar percent={taskStatus.progress} />
            </div>
          </div>
        )}
        {useMultiTaskMode && (
          <>
            <div className="h-3"></div>
            <div className="flex items-center justify-center gap-2">
              <div className="flex w-full justify-center text-2xl text-gray-900 font-semibold whitespace-nowrap">
                {taskStatus.proceedTime}
              </div>
              <div className="w-full justify-center text-2xl text-gray-500 font-semibold whitespace-nowrap">
                seconds passed
              </div>
            </div>
          </>
        )}
      </div>
      <div className="flex justify-end flex-[0.4] min-w-30 h-full p-1 gap-2">
        {useMultiTaskMode ? (
          <div className="flex flex-col gap-2">
            <FullTaskStatus />
            <EpisodeStatus />
          </div>
        ) : (
          <EpisodeStatus />
        )}
      </div>
      <div className="flex flex-col gap-2">
        {expandedSystemIndex !== null ? (
          /* Expanded System View */
          <div onClick={() => setExpandedSystemIndex(null)} className="cursor-pointer">
            {expandedSystemIndex === 0 ? (
              /* CPU Details */
              <SystemStatus label="CPU" type="cpu" />
            ) : expandedSystemIndex === 1 ? (
              /* RAM Details */
              <SystemStatus label="RAM" type="ram" />
            ) : (
              /* Storage Details */
              <SystemStatus label="Storage" type="storage" />
            )}
          </div>
        ) : (
          /* Compact System List */
          <>
            {/* CPU */}
            <div onClick={() => setExpandedSystemIndex(0)} className="cursor-pointer">
              <CompactSystemStatus label="CPU" type="cpu" />
            </div>

            {/* RAM */}
            <div onClick={() => setExpandedSystemIndex(1)} className="cursor-pointer">
              <CompactSystemStatus label="RAM" type="ram" />
            </div>

            {/* Storage */}
            <div onClick={() => setExpandedSystemIndex(2)} className="cursor-pointer">
              <CompactSystemStatus label="Storage" type="storage" />
            </div>
          </>
        )}
      </div>
    </div>
  );
}
