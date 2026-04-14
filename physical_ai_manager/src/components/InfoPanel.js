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
import { useSelector, useDispatch } from 'react-redux';
import clsx from 'clsx';
import TaskPhase from '../constants/taskPhases';
import { setTaskInfo } from '../features/tasks/taskSlice';

const InfoPanel = () => {
  const dispatch = useDispatch();

  const info = useSelector((state) => state.tasks.taskInfo);
  const taskStatus = useSelector((state) => state.tasks.taskStatus);

  const [isTaskStatusPaused, setIsTaskStatusPaused] = useState(false);
  const [lastTaskStatusUpdate, setLastTaskStatusUpdate] = useState(Date.now());

  // Black-list approach: only lock the panel while a task is actually
  // running. Any other phase (including STOPPED, an unknown initial value,
  // or no recent backend echo) leaves the inputs editable.
  const isTaskRunning =
    taskStatus.phase === TaskPhase.WARMING_UP ||
    taskStatus.phase === TaskPhase.RESETTING ||
    taskStatus.phase === TaskPhase.RECORDING ||
    taskStatus.phase === TaskPhase.SAVING ||
    taskStatus.phase === TaskPhase.LOADING ||
    taskStatus.phase === TaskPhase.INFERENCING ||
    taskStatus.phase === TaskPhase.CONVERTING ||
    taskStatus.phase === TaskPhase.PAUSED;
  const disabled = isTaskRunning;
  const [isEditable, setIsEditable] = useState(!disabled);

  const handleChange = useCallback(
    (field, value) => {
      if (!isEditable) return; // Block changes when not editable
      dispatch(setTaskInfo({ ...info, [field]: value }));
    },
    [isEditable, info, dispatch]
  );

  // Update isEditable state when the disabled prop changes
  useEffect(() => {
    setIsEditable(!disabled);
  }, [disabled]);

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
      const isPaused = timeSinceLastUpdate >= UPDATE_PAUSE_THRESHOLD;
      if (isPaused !== isTaskStatusPaused) {
        setIsTaskStatusPaused(isPaused);
      }
    }, 1000);

    return () => clearInterval(timer);
  }, [lastTaskStatusUpdate, isTaskStatusPaused]);

  const classLabel = clsx('text-sm', 'text-gray-600', 'w-28', 'flex-shrink-0', 'font-medium');

  const classInfoPanel = clsx(
    'bg-white',
    'border',
    'border-gray-200',
    'rounded-2xl',
    'shadow-md',
    'p-4',
    'w-full',
    'max-w-[350px]',
    'relative',
    'overflow-y-auto',
    'scrollbar-thin'
  );

  const classTaskNameTextarea = clsx(
    'text-sm',
    'resize-y',
    'min-h-8',
    'max-h-20',
    'h-10',
    'w-full',
    'p-2',
    'border',
    'border-gray-300',
    'rounded-md',
    'focus:outline-none',
    'focus:ring-2',
    'focus:ring-blue-500',
    'focus:border-transparent',
    {
      'bg-gray-100 cursor-not-allowed': !isEditable,
      'bg-white': isEditable,
    }
  );

  const classTaskInstructionTextarea = clsx(
    'text-sm',
    'resize-y',
    'min-h-16',
    'max-h-24',
    'w-full',
    'p-2',
    'border',
    'border-gray-300',
    'rounded-md',
    'focus:outline-none',
    'focus:ring-2',
    'focus:ring-blue-500',
    'focus:border-transparent',
    {
      'bg-gray-100 cursor-not-allowed': !isEditable,
      'bg-white': isEditable,
    }
  );

  return (
    <div className={classInfoPanel}>
      <div className={clsx('text-lg', 'font-semibold', 'mb-3', 'text-gray-800')}>
        Task Information
      </div>

      {/* Edit mode indicator */}
      <div
        className={clsx('mb-3', 'p-2', 'rounded-md', 'text-sm', 'font-medium', {
          'bg-green-100 text-green-800': isEditable,
          'bg-gray-100 text-gray-600': !isEditable,
        })}
      >
        {isEditable ? (
          'Edit mode'
        ) : (
          <div className="leading-tight">
            <div>Read only</div>
            <div className="text-xs mt-1 opacity-80">task is running or robot is not connected</div>
          </div>
        )}
      </div>

      {/* Task Num */}
      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>Task Num</span>
        <textarea
          className={classTaskNameTextarea}
          value={info.taskNum || ''}
          onChange={(e) => handleChange('taskNum', e.target.value)}
          disabled={!isEditable}
          placeholder="Enter Task Num"
        />
      </div>

      {/* Task Name */}
      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>Task Name</span>
        <textarea
          className={classTaskNameTextarea}
          value={info.taskName || ''}
          onChange={(e) => handleChange('taskName', e.target.value)}
          disabled={!isEditable}
          placeholder="Enter Task Name"
        />
      </div>

      {/* Task Instruction */}
      <div className={clsx('flex', 'items-start', 'mb-2.5')}>
        <span className={clsx(classLabel, 'pt-2')}>Task Instruction</span>
        <textarea
          className={classTaskInstructionTextarea}
          value={(info.taskInstruction && info.taskInstruction[0]) || ''}
          onChange={(e) => handleChange('taskInstruction', [e.target.value])}
          disabled={!isEditable}
          placeholder="Enter Task Instruction"
        />
      </div>

      {/* Dataset save path indicator */}
      <div className="flex flex-col items-center text-xs text-gray-500 mt-3 leading-relaxed bg-gray-100 p-2 rounded-md">
        <div>Dataset will be saved as:</div>
        <div className="text-blue-500 font-bold break-all">
          Task_{info.taskNum}_{info.taskName}_MCAP
        </div>
      </div>

    </div>
  );
};

export default InfoPanel;
