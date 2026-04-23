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
// Author: Dongyun Kim

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import clsx from 'clsx';
import toast, { useToasterStore } from 'react-hot-toast';
import {
  MdPlayArrow,
  MdStop,
  MdDeleteOutline,
  MdFiberManualRecord,
  MdSave,
  MdClose,
} from 'react-icons/md';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import Tooltip from './Tooltip';
import TaskPhase from '../constants/taskPhases';
import { setTaskStatus } from '../features/tasks/taskSlice';

const phaseGuideMessages = {
  [TaskPhase.READY]: 'Ready to start',
  [TaskPhase.LOADING]: 'Loading model...',
  [TaskPhase.INFERENCING]: 'Inferencing',
  [TaskPhase.PAUSED]: 'Paused',
};

const requiredFields = [
  { key: 'taskInstruction', label: 'Task Instruction' },
  { key: 'policyPath', label: 'Policy Path' },
];

const spinnerFrames = ['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧'];

export default function InferenceControlPanel() {
  const dispatch = useDispatch();
  const taskInfo = useSelector((state) => state.tasks.taskInfo);
  const taskStatus = useSelector((state) => state.tasks.taskStatus);
  const rosHost = useSelector((state) => state.ros.rosHost);

  const [hovered, setHovered] = useState(null);
  const [pressed, setPressed] = useState(null);
  const [isRecording, setIsRecording] = useState(false);
  const [lastPolicyPath, setLastPolicyPath] = useState('');
  const [spinnerIndex, setSpinnerIndex] = useState(0);

  const isRecordingRef = useRef(isRecording);

  const { sendRecordCommand } = useRosServiceCaller();

  const { toasts } = useToasterStore();
  const TOAST_LIMIT = 3;

  const phase = taskStatus.phase;
  const isIdle = phase === TaskPhase.READY || phase === TaskPhase.IDLE;
  const isLoading = phase === TaskPhase.LOADING;
  const isInferencing = phase === TaskPhase.INFERENCING;
  const isPaused = phase === TaskPhase.PAUSED;
  const isModelLoaded = isInferencing || isPaused;

  useEffect(() => {
    isRecordingRef.current = isRecording;
  }, [isRecording]);

  useEffect(() => {
    toasts
      .filter((t) => t.visible)
      .filter((_, i) => i >= TOAST_LIMIT)
      .forEach((t) => toast.dismiss(t.id));
  }, [toasts]);

  useEffect(() => {
    setSpinnerIndex((prev) => (prev + 1) % spinnerFrames.length);
  }, [taskStatus]);

  useEffect(() => {
    if (isIdle && isRecordingRef.current) {
      setIsRecording(false);
    }
  }, [isIdle]);

  const validateTaskInfo = useCallback(() => {
    const missingFields = [];
    for (const field of requiredFields) {
      const value = taskInfo[field.key];
      if (
        value === null ||
        value === undefined ||
        value === '' ||
        (typeof value === 'string' && value.trim() === '') ||
        (Array.isArray(value) && value.length === 0) ||
        (Array.isArray(value) && value.every((item) => item.trim() === ''))
      ) {
        missingFields.push(field.label);
      }
    }
    return { isValid: missingFields.length === 0, missingFields };
  }, [taskInfo]);

  const executeCommand = useCallback(
    async (commandName, commandString) => {
      try {
        const result = await sendRecordCommand(commandString);
        if (result && result.success === false) {
          toast.error(`Command failed: ${result.message || 'Unknown error'}`);
          // Backend may have left phase in LOADING/INFERENCING after a failed
          // setup; force the local phase back to READY so the panel becomes
          // editable and the user can retry.
          dispatch(setTaskStatus({ phase: TaskPhase.READY, running: false }));
        } else if (result && result.success === true) {
          toast.success(`${commandName} executed successfully`);
        } else {
          toast.error(`${commandName} completed with uncertain status`);
          dispatch(setTaskStatus({ phase: TaskPhase.READY, running: false }));
        }
        return result;
      } catch (error) {
        let errorMessage = error.message || error.toString();
        if (
          errorMessage.includes('ROS connection failed') ||
          errorMessage.includes('WebSocket')
        ) {
          toast.error(`ROS connection failed: rosbridge server is not running (${rosHost})`);
        } else if (errorMessage.includes('timeout')) {
          toast.error(`Command timeout [${commandName}]: Server did not respond`);
        } else {
          toast.error(`Command failed [${commandName}]: ${errorMessage}`);
        }
        // Same reasoning as the success===false branch above.
        dispatch(setTaskStatus({ phase: TaskPhase.READY, running: false }));
        return null;
      }
    },
    [sendRecordCommand, rosHost, dispatch]
  );

  const handleStart = useCallback(async () => {
    if (isPaused && taskInfo.policyPath === lastPolicyPath) {
      await executeCommand('Resume', 'resume_inference');
    } else {
      const validation = validateTaskInfo();
      if (!validation.isValid) {
        toast.error(`Missing required fields: ${validation.missingFields.join(', ')}`);
        return;
      }
      setLastPolicyPath(taskInfo.policyPath);
      await executeCommand('Start Inference', 'start_inference');
    }
  }, [isPaused, taskInfo.policyPath, lastPolicyPath, executeCommand, validateTaskInfo]);

  const handleStop = useCallback(async () => {
    await executeCommand('Stop', 'stop_inference');
  }, [executeCommand]);

  const handleClear = useCallback(async () => {
    if (isRecording) {
      await executeCommand('Cancel Recording', 'cancel_inference_record');
      setIsRecording(false);
    }
    await executeCommand('Clear', 'finish');
    setLastPolicyPath('');
  }, [executeCommand, isRecording]);

  const handleRecordStart = useCallback(async () => {
    const result = await executeCommand('Record Start', 'start_inference_record');
    if (result && result.success) {
      setIsRecording(true);
    }
  }, [executeCommand]);

  const handleRecordSave = useCallback(async () => {
    const result = await executeCommand('Record Save', 'stop_inference_record');
    if (result && result.success) {
      setIsRecording(false);
    }
  }, [executeCommand]);

  const handleRecordDiscard = useCallback(async () => {
    const result = await executeCommand('Record Discard', 'cancel_inference_record');
    if (result && result.success) {
      setIsRecording(false);
    }
  }, [executeCommand]);

  const startEnabled = isIdle || isPaused;
  const stopEnabled = isInferencing;
  const clearEnabled = isModelLoaded;
  const recordEnabled = isModelLoaded && !isRecording && !!taskInfo.recordInferenceMode;

  const handleKeyAction = useCallback(
    (e) => {
      if (e.key === ' ' || e.key === 'Spacebar' || e.code === 'Space') {
        if (startEnabled) return 'Start';
      }
      if (
        (e.ctrlKey || e.metaKey) &&
        e.shiftKey &&
        (e.key === 's' || e.key === 'S')
      ) {
        if (stopEnabled) return 'Stop';
      }
      if (e.key === 'Escape') {
        if (clearEnabled) return 'Clear';
      }
      if (e.key === 'r' || e.key === 'R') {
        if (!e.ctrlKey && !e.metaKey && !e.altKey) {
          if (isRecording) return 'RecordSave';
          if (recordEnabled) return 'RecordStart';
        }
      }
      return null;
    },
    [startEnabled, stopEnabled, clearEnabled, recordEnabled, isRecording]
  );

  useEffect(() => {
    const isInputFocused = () => {
      const el = document.activeElement;
      if (!el) return false;
      const tag = el.tagName.toLowerCase();
      return tag === 'input' || tag === 'textarea' || tag === 'select' ||
        el.contentEditable === 'true';
    };

    const handleKeyDown = (e) => {
      if (e.repeat || isInputFocused()) return;
      const action = handleKeyAction(e);
      if (action) {
        e.preventDefault();
        setPressed(action);
      }
    };

    const handleKeyUp = (e) => {
      setPressed(null);
      if (isInputFocused()) return;
      const action = handleKeyAction(e);
      if (action === 'Start') handleStart();
      else if (action === 'Stop') handleStop();
      else if (action === 'Clear') handleClear();
      else if (action === 'RecordStart') handleRecordStart();
      else if (action === 'RecordSave') handleRecordSave();
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [handleKeyAction, handleStart, handleStop, handleClear, handleRecordStart, handleRecordSave]);

  const classBody = clsx(
    'bg-white/90',
    'backdrop-blur-sm',
    'rounded-full',
    'px-3',
    'py-1',
    'flex',
    'flex-row',
    'items-center',
    'gap-1.5',
    'shadow-md',
    'border',
    'border-gray-100'
  );

  const classBtn = (label, isDisabled) =>
    clsx(
      'h-full',
      'rounded-lg',
      'border-none',
      'cursor-pointer',
      'px-2.5',
      'flex',
      'items-center',
      'justify-center',
      'gap-1',
      'bg-gray-100',
      'transition-all',
      'duration-150',
      'font-semibold',
      'text-lg',
      'shrink-0',
      {
        'bg-gray-400': pressed === label && !isDisabled,
        'bg-gray-200': hovered === label && pressed !== label && !isDisabled,
        'opacity-30 cursor-not-allowed bg-gray-50': isDisabled,
      }
    );

  const classRecBtn = (variant, isDisabled) =>
    clsx(
      'h-full',
      'rounded-lg',
      'border-none',
      'cursor-pointer',
      'px-2.5',
      'flex',
      'items-center',
      'justify-center',
      'gap-1',
      'font-semibold',
      'text-lg',
      'transition-all',
      'duration-150',
      'shrink-0',
      {
        'bg-red-500 text-white hover:bg-red-600': variant === 'record' && !isDisabled,
        'bg-green-500 text-white hover:bg-green-600': variant === 'save',
        'bg-gray-500 text-white hover:bg-gray-600': variant === 'discard',
        'opacity-30 cursor-not-allowed bg-gray-200 text-gray-400': isDisabled,
      }
    );

  const controlButtons = [
    {
      label: 'Start',
      icon: MdPlayArrow,
      color: '#1976d2',
      enabled: startEnabled,
      handler: handleStart,
      description: isPaused ? 'Resume inference' : 'Start inference',
      shortcut: 'Space',
    },
    {
      label: 'Stop',
      icon: MdStop,
      color: '#f57c00',
      enabled: stopEnabled,
      handler: handleStop,
      description: 'Pause inference (model stays loaded)',
      shortcut: 'Ctrl+Shift+S',
    },
    {
      label: 'Clear',
      icon: MdDeleteOutline,
      color: '#d32f2f',
      enabled: clearEnabled,
      handler: handleClear,
      description: 'Stop inference and unload model',
      shortcut: 'Escape',
    },
  ];

  return (
    <div className={classBody}>
      <span className="text-lg font-semibold text-gray-500 whitespace-nowrap px-1 shrink-0">Inference</span>
      <div className="w-px h-2/3 bg-gray-300 shrink-0"></div>
      {controlButtons.map(({ label, icon: Icon, color, enabled, handler, description, shortcut }) => {
        const isDisabled = !enabled;
        return (
          <Tooltip
            key={label}
            position="bottom"
            content={
              <div className="text-center">
                <div className="font-semibold">{description}</div>
                {!isDisabled && (
                  <div className="text-sm mt-1 text-gray-300">
                    <span className="font-mono bg-gray-700 px-1 rounded">{shortcut}</span>
                  </div>
                )}
              </div>
            }
            disabled={false}
            className="relative h-full shrink-0"
          >
            <button
              className={classBtn(label, isDisabled)}
              onClick={() => !isDisabled && handler()}
              onMouseEnter={() => !isDisabled && setHovered(label)}
              onMouseLeave={() => { setHovered(null); setPressed(null); }}
              onMouseDown={() => !isDisabled && setPressed(label)}
              onMouseUp={() => setPressed(null)}
              disabled={isDisabled}
              aria-label={description}
            >
              <Icon
                style={{ fontSize: '1.1rem' }}
                color={isDisabled ? '#9ca3af' : color}
              />
              {label}
            </button>
          </Tooltip>
        );
      })}

      <div className="w-px h-2/3 bg-gray-400 shrink-0"></div>

      {!isRecording ? (
        <Tooltip
          position="bottom"
          content={
            <div className="text-center">
              <div className="font-semibold">Start recording</div>
              <div className="text-sm mt-1 text-gray-300">
                <span className="font-mono bg-gray-700 px-1 rounded">R</span>
              </div>
            </div>
          }
          disabled={!recordEnabled}
          className="relative h-full shrink-0"
        >
          <button
            className={classRecBtn('record', !recordEnabled)}
            onClick={() => recordEnabled && handleRecordStart()}
            disabled={!recordEnabled}
            aria-label="Start recording"
          >
            <MdFiberManualRecord style={{ fontSize: '0.8rem' }} />
            Record
          </button>
        </Tooltip>
      ) : (
        <>
          <div className="flex items-center gap-0.5 text-red-500 font-bold text-lg animate-pulse shrink-0">
            <MdFiberManualRecord style={{ fontSize: '0.5rem' }} />
            REC
          </div>
          <Tooltip
            position="bottom"
            content={
              <div className="text-center">
                <div className="font-semibold">Save recording</div>
                <div className="text-sm mt-1 text-gray-300">
                  <span className="font-mono bg-gray-700 px-1 rounded">R</span>
                </div>
              </div>
            }
            className="relative h-full shrink-0"
          >
            <button
              className={classRecBtn('save', false)}
              onClick={handleRecordSave}
              aria-label="Save recording"
            >
              <MdSave style={{ fontSize: '1rem' }} />
              Save
            </button>
          </Tooltip>
          <Tooltip
            position="bottom"
            content={
              <div className="text-center">
                <div className="font-semibold">Discard recording</div>
              </div>
            }
            className="relative h-full shrink-0"
          >
            <button
              className={classRecBtn('discard', false)}
              onClick={handleRecordDiscard}
              aria-label="Discard recording"
            >
              <MdClose style={{ fontSize: '1rem' }} />
              Discard
            </button>
          </Tooltip>
        </>
      )}

      {(phaseGuideMessages[phase] || isInferencing || isLoading) && (
        <>
          <div className="w-px h-2/3 bg-gray-400 shrink-0"></div>
          <div className="flex items-center gap-1 shrink-0">
            <span className="text-gray-600 font-semibold text-lg whitespace-nowrap">
              {phaseGuideMessages[phase] || ''}
            </span>
            {(isInferencing || isLoading) && (
              <span className="font-mono text-blue-500 text-sm">
                {spinnerFrames[spinnerIndex]}
              </span>
            )}
          </div>
        </>
      )}
    </div>
  );
}
