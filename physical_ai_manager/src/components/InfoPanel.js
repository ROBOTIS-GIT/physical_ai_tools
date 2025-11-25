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
import TaskInstructionInput from './TaskInstructionInput';
import toast from 'react-hot-toast';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import TagInput from './TagInput';
import TokenInputPopup from './TokenInputPopup';
import TaskPhase from '../constants/taskPhases';
import { setTaskInfo, setUseMultiTaskMode } from '../features/tasks/taskSlice';

const taskInfos = [
  {
    taskName: 'Task 1',
    robotType: 'Type A',
    taskType: 'Type X',
    taskInstruction: 'Do X',
    userId: 'repo1',
    fps: 30,
    tags: ['tag1'],
    warmupTime: '10',
    episodeTime: '60',
    resetTime: '5',
    numEpisodes: 3,
    pushToHub: false,
  },
  {
    taskName: 'Task 2',
    robotType: 'Type B',
    taskType: 'Type Y',
    taskInstruction: 'Do Y',
    userId: 'repo2',
    fps: 20,
    tags: ['tag2'],
    warmupTime: '5',
    episodeTime: '30',
    resetTime: '2',
    numEpisodes: 5,
    pushToHub: true,
  },
];

const InfoPanel = () => {
  const dispatch = useDispatch();

  const info = useSelector((state) => state.tasks.taskInfo);
  const taskStatus = useSelector((state) => state.tasks.taskStatus);

  const [isTaskStatusPaused, setIsTaskStatusPaused] = useState(false);
  const [lastTaskStatusUpdate, setLastTaskStatusUpdate] = useState(Date.now());

  const useMultiTaskMode = useSelector((state) => state.tasks.useMultiTaskMode);

  const [showPopup, setShowPopup] = useState(false);
  const [taskInfoList] = useState(taskInfos);
  const disabled = taskStatus.phase !== TaskPhase.READY || !isTaskStatusPaused;
  const [isEditable, setIsEditable] = useState(!disabled);

  // User ID list for dropdown
  const [userIdList, setUserIdList] = useState([]);

  // Token popup states
  const [showTokenPopup, setShowTokenPopup] = useState(false);
  const [isLoading, setIsLoading] = useState(false);

  // User ID selection states
  const [showUserIdDropdown, setShowUserIdDropdown] = useState(false);

  const { registerHFUser, getRegisteredHFUser } = useRosServiceCaller();

  const handleChange = useCallback(
    (field, value) => {
      if (!isEditable) return; // Block changes when not editable
      dispatch(setTaskInfo({ ...info, [field]: value }));
    },
    [isEditable, info, dispatch]
  );

  const handleSelect = (selected) => {
    dispatch(setTaskInfo(selected));
    setShowPopup(false);
  };

  const handleTokenSubmit = async (token) => {
    if (!token || !token.trim()) {
      toast.error('Please enter a token');
      return;
    }

    setIsLoading(true);
    try {
      const result = await registerHFUser(token);
      console.log('registerHFUser result:', result);

      if (result && result.user_id_list) {
        setUserIdList(result.user_id_list);
        setShowTokenPopup(false);
        toast.success('User ID list updated successfully!');
      } else {
        toast.error('Failed to get user ID list from response');
      }
    } catch (error) {
      console.error('Error registering HF user:', error);
      toast.error(`Failed to register user: ${error.message}`);
    } finally {
      setIsLoading(false);
    }
  };

  const handleLoadUserId = useCallback(async () => {
    setIsLoading(true);
    try {
      const result = await getRegisteredHFUser();
      console.log('getRegisteredHFUser result:', result);

      if (result && result.user_id_list) {
        if (result.success) {
          setUserIdList(result.user_id_list);
          toast.success('User ID list loaded successfully!');
          setShowUserIdDropdown(true);
        } else {
          toast.error('Failed to get user ID list:\n' + result.message);
        }
      } else {
        toast.error('Failed to get user ID list from response');
      }
    } catch (error) {
      console.error('Error loading HF user list:', error);
      toast.error(`Failed to load user ID list: ${error.message}`);
    } finally {
      setIsLoading(false);
    }
  }, [getRegisteredHFUser]);

  const handleUserIdSelect = useCallback(
    (selectedUserId) => {
      handleChange('userId', selectedUserId);
      setShowUserIdDropdown(false);
    },
    [handleChange]
  );

  // Update isEditable state when the disabled prop changes
  useEffect(() => {
    setIsEditable(!disabled);
  }, [disabled]);

  // Reset dropdown state when Push to Hub is unchecked
  useEffect(() => {
    if (!info.pushToHub) {
      setShowUserIdDropdown(false);
    }
  }, [info.pushToHub]);

  // Auto-enable optimized save when multi-task mode is enabled
  useEffect(() => {
    if (useMultiTaskMode && !info.useOptimizedSave) {
      dispatch(setTaskInfo({ ...info, useOptimizedSave: true }));
    }

    if (useMultiTaskMode && info.recordRosBag2) {
      dispatch(setTaskInfo({ ...info, recordRosBag2: false }));
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [useMultiTaskMode, info.useOptimizedSave, dispatch]);

  useEffect(() => {
    if (info.pushToHub) {
      handleLoadUserId();
    }
  }, [handleLoadUserId, info.pushToHub]);

  useEffect(() => {
    if (userIdList.length > 0 && info.userId === undefined) {
      handleUserIdSelect(userIdList[0]);
    }
  }, [userIdList, info.userId, handleUserIdSelect]);

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

  const classSingleTaskButton = clsx(
    'px-3',
    'py-1',
    'text-sm',
    'rounded-xl',
    'font-medium',
    'transition-colors',
    !useMultiTaskMode ? 'bg-blue-500 text-white' : 'bg-gray-200 text-gray-700',
    !isEditable && 'cursor-not-allowed opacity-60'
  );

  const classMultiTaskButton = clsx(
    'px-3',
    'py-1',
    'text-sm',
    'rounded-xl',
    'font-medium',
    'transition-colors',
    useMultiTaskMode ? 'bg-blue-500 text-white' : 'bg-gray-200 text-gray-700',
    !isEditable && 'cursor-not-allowed opacity-60'
  );

  const classRepoIdTextarea = clsx(
    'text-sm',
    'resize-y',
    'min-h-10',
    'max-h-24',
    'h-14',
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
      'bg-gray-100 cursor-not-allowed': !isEditable || info.pushToHub,
      'bg-white': isEditable && !info.pushToHub,
    }
  );

  const classTextInput = clsx(
    'text-sm',
    'w-full',
    'h-8',
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

  const classSelect = clsx(
    'text-sm',
    'w-full',
    'h-8',
    'px-2',
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

  const classCheckbox = clsx(
    'w-4',
    'h-4',
    'text-blue-600',
    'bg-gray-100',
    'border-gray-300',
    'rounded',
    'focus:ring-blue-500',
    'focus:ring-2',
    {
      'cursor-not-allowed opacity-50': !isEditable,
      'cursor-pointer': isEditable,
    }
  );

  // Common button base styles
  const classButtonBase = clsx(
    'px-3',
    'py-1',
    'text-s',
    'font-medium',
    'rounded-xl',
    'transition-colors'
  );

  // Button variants
  const getButtonVariant = (variant, isActive = true, isLoading = false) => {
    const variants = {
      blue: {
        active: 'bg-blue-200 text-blue-800 hover:bg-blue-300',
        disabled: 'bg-gray-200 text-gray-500 cursor-not-allowed',
      },
      red: {
        active: 'bg-red-200 text-red-800 hover:bg-red-300',
        disabled: 'bg-gray-200 text-gray-500 cursor-not-allowed',
      },
      green: {
        active: 'bg-green-200 text-green-800 hover:bg-green-300',
        disabled: 'bg-gray-200 text-gray-500 cursor-not-allowed',
      },
    };

    const isDisabled = !isActive || isLoading;
    return variants[variant]?.[isDisabled ? 'disabled' : 'active'] || '';
  };

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
          '✏️ Edit mode'
        ) : (
          <div className="leading-tight">
            <div>🔒 Read only</div>
            <div className="text-xs mt-1 opacity-80">task is running or robot is not connected</div>
          </div>
        )}
      </div>

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

      <div className={clsx('flex', 'items-start', 'mb-2.5')}>
        <span
          className={clsx(
            'text-sm',
            'text-gray-600',
            'w-28',
            'flex-shrink-0',
            'font-medium',
            'pt-2'
          )}
        >
          Task Instruction
        </span>

        <div>
          {/* Single/Multi Task Mode Toggle */}
          <div className={clsx('flex', 'justify-start', 'mb-3', 'gap-3')}>
            <button
              type="button"
              className={classSingleTaskButton}
              onClick={() => isEditable && dispatch(setUseMultiTaskMode(false))}
              disabled={!isEditable}
            >
              Single Task
            </button>
            <button
              type="button"
              className={classMultiTaskButton}
              onClick={() => isEditable && dispatch(setUseMultiTaskMode(true))}
              disabled={!isEditable}
            >
              Multi Task
            </button>
          </div>

          {useMultiTaskMode && (
            <div className="flex-1 min-w-0">
              <TaskInstructionInput
                instructions={info.taskInstruction || []}
                onChange={(newInstructions) => handleChange('taskInstruction', newInstructions)}
                disabled={!isEditable}
              />
            </div>
          )}
          {!useMultiTaskMode && (
            <textarea
              className={classTaskInstructionTextarea}
              value={info.taskInstruction || ''}
              onChange={(e) => handleChange('taskInstruction', [e.target.value])}
              disabled={!isEditable}
              placeholder="Enter Task Instruction"
            />
          )}
        </div>
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2')}>
        <span className={classLabel}>Push to Hub</span>
        <div className={clsx('flex', 'items-center')}>
          <input
            className={classCheckbox}
            type="checkbox"
            checked={!!info.pushToHub}
            onChange={(e) => handleChange('pushToHub', e.target.checked)}
            disabled={!isEditable}
          />
          <span className={clsx('ml-2', 'text-sm', 'text-gray-500')}>
            {info.pushToHub ? 'Enabled' : 'Disabled'}
          </span>
        </div>
      </div>

      {info.pushToHub && (
        <div className={clsx('flex', 'items-center', 'mb-2')}>
          <span className={classLabel}>Private Mode</span>
          <div className={clsx('flex', 'items-center')}>
            <input
              className={classCheckbox}
              type="checkbox"
              checked={!!info.privateMode}
              onChange={(e) => handleChange('privateMode', e.target.checked)}
              disabled={!isEditable}
            />
            <span className={clsx('ml-2', 'text-sm', 'text-gray-500')}>
              {info.privateMode ? 'Enabled' : 'Disabled'}
            </span>
          </div>
        </div>
      )}

      <div className={clsx('flex', 'items-start', 'mb-2.5')}>
        <span
          className={clsx(
            'text-sm',
            'text-gray-600',
            'w-28',
            'flex-shrink-0',
            'font-medium',
            'pt-2'
          )}
        >
          User ID
        </span>

        <div className="flex-1 min-w-0">
          {/* Common Load button for both modes */}
          <div className="flex gap-2 mb-2">
            <button
              className={clsx(classButtonBase, getButtonVariant('blue', isEditable, isLoading))}
              onClick={() => {
                if (isEditable && !isLoading) {
                  handleLoadUserId();
                }
              }}
              disabled={!isEditable || isLoading}
            >
              {isLoading ? 'Loading...' : 'Load'}
            </button>
            {!info.pushToHub && showUserIdDropdown && (
              <button
                className={clsx(classButtonBase, getButtonVariant('red', isEditable))}
                onClick={() => setShowUserIdDropdown(false)}
                disabled={!isEditable}
              >
                Manual Input
              </button>
            )}
            {info.pushToHub && (
              <button
                className={clsx(classButtonBase, getButtonVariant('green', isEditable, isLoading))}
                onClick={() => {
                  if (isEditable && !isLoading) {
                    setShowTokenPopup(true);
                  }
                }}
                disabled={!isEditable || isLoading}
              >
                Change
              </button>
            )}
          </div>

          {info.pushToHub ? (
            /* Dropdown selection only when Push to Hub is enabled */
            <>
              <select
                className={classSelect}
                value={info.userId || ''}
                onChange={(e) => handleChange('userId', e.target.value)}
                disabled={!isEditable}
              >
                <option value="">Select User ID</option>
                {userIdList.map((userId) => (
                  <option key={userId} value={userId}>
                    {userId}
                  </option>
                ))}
              </select>
              <div className="text-xs text-gray-500 mt-1 leading-relaxed">
                Select from registered User IDs (required for Hub upload)
              </div>
            </>
          ) : (
            /* Text input with optional registered ID selection when Push to Hub is disabled */
            <>
              {!showUserIdDropdown ? (
                <>
                  <textarea
                    className={classRepoIdTextarea}
                    value={info.userId || ''}
                    onChange={(e) => handleChange('userId', e.target.value)}
                    disabled={!isEditable}
                    placeholder="Enter User ID or load from registered ID"
                  />
                  <div className="text-xs text-gray-500 mt-1 leading-relaxed">
                    Enter any User ID manually or load from registered IDs
                  </div>
                </>
              ) : (
                <>
                  <select
                    className={classSelect}
                    value=""
                    onChange={(e) => {
                      if (e.target.value) {
                        handleUserIdSelect(e.target.value);
                      }
                    }}
                    disabled={!isEditable}
                  >
                    <option value="">Select from registered User IDs</option>
                    {userIdList.map((userId) => (
                      <option key={userId} value={userId}>
                        {userId}
                      </option>
                    ))}
                  </select>
                  <div className="text-xs text-gray-500 mt-1 leading-relaxed">
                    Select a registered User ID or use Cancel button above
                  </div>
                </>
              )}
            </>
          )}
        </div>
      </div>

      <div className="flex flex-col items-center text-xs text-gray-500 mt-1 leading-relaxed bg-gray-100 p-2 rounded-md mb-2">
        <div>Dataset will be saved with the following repo id</div>
        <div className="text-blue-500 font-bold break-all">
          {info.userId}/{taskStatus?.robotType}_{info.taskName}
        </div>
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>FPS</span>
        <input
          className={classTextInput}
          type="number"
          step="5"
          value={info.fps || ''}
          onChange={(e) => handleChange('fps', Number(e.target.value))}
          disabled={!isEditable}
        />
      </div>

      <div className={clsx('flex', 'items-start', 'mb-2.5')}>
        <span className={clsx(classLabel, 'pt-2')}>Tags</span>
        <div className="flex-1 min-w-0">
          <TagInput
            tags={info.tags || []}
            onChange={(newTags) => handleChange('tags', newTags)}
            disabled={!isEditable}
          />
          <div className="text-xs text-gray-500 mt-1 leading-relaxed">
            Press Enter or use comma to add tags
          </div>
        </div>
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>Warmup Time (s)</span>
        <input
          className={classTextInput}
          type="number"
          step="5"
          min={0}
          max={65535}
          value={info.warmupTime || ''}
          onChange={(e) => handleChange('warmupTime', Number(e.target.value) || 0)}
          disabled={!isEditable}
        />
      </div>

      {!useMultiTaskMode && (
        <>
          <div className={clsx('flex', 'items-center', 'mb-2.5')}>
            <span className={classLabel}>Episode Time (s)</span>
            <input
              className={classTextInput}
              type="number"
              step="5"
              min={0}
              max={65535}
              value={info.episodeTime || ''}
              onChange={(e) => handleChange('episodeTime', Number(e.target.value) || 0)}
              disabled={!isEditable}
            />
          </div>

          <div className={clsx('flex', 'items-center', 'mb-2.5')}>
            <span className={classLabel}>Reset Time (s)</span>
            <input
              className={classTextInput}
              type="number"
              step="5"
              min={0}
              max={65535}
              value={info.resetTime || ''}
              onChange={(e) => handleChange('resetTime', Number(e.target.value) || 0)}
              disabled={!isEditable || useMultiTaskMode}
            />
          </div>

          <div className={clsx('flex', 'items-center', 'mb-2.5')}>
            <span className={classLabel}>Num Episodes</span>
            <input
              className={classTextInput}
              type="number"
              step="1"
              min={0}
              max={65535}
              value={info.numEpisodes || ''}
              onChange={(e) => handleChange('numEpisodes', Number(e.target.value) || 0)}
              disabled={!isEditable || useMultiTaskMode}
            />
          </div>
        </>
      )}

      <div className={clsx('flex', 'items-center', 'mb-2')}>
        <span className={classLabel}>Optimized Save</span>
        <div className="flex flex-col">
          <div className={clsx('flex', 'items-center')}>
            <input
              className={clsx(classCheckbox, {
                'cursor-not-allowed opacity-50': useMultiTaskMode,
              })}
              type="checkbox"
              checked={!!info.useOptimizedSave}
              onChange={(e) => handleChange('useOptimizedSave', e.target.checked)}
              disabled={!isEditable || useMultiTaskMode}
            />
            <span className={clsx('ml-2', 'text-sm', 'text-gray-500')}>
              {info.useOptimizedSave ? 'Enabled' : 'Disabled'}
            </span>
          </div>
          {useMultiTaskMode && (
            <span className="text-xs text-blue-600 ml-1">(Auto-enabled in Multi-Task mode)</span>
          )}
        </div>
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2')}>
        <span className={classLabel}>Record Rosbag2</span>
        <div className="flex flex-col">
          <div className={clsx('flex', 'items-center')}>
            <input
              className={clsx(classCheckbox, {
                'cursor-not-allowed opacity-50': useMultiTaskMode,
              })}
              type="checkbox"
              checked={!!info.recordRosBag2}
              onChange={(e) => handleChange('recordRosBag2', e.target.checked)}
              disabled={!isEditable || useMultiTaskMode}
            />
            <span className={clsx('ml-2', 'text-sm', 'text-gray-500')}>
              {info.recordRosBag2 ? 'Enabled' : 'Disabled'}
            </span>
          </div>
          {useMultiTaskMode && (
            <span className="text-xs text-blue-600 ml-1">(Auto-disabled in Multi-Task mode)</span>
          )}
        </div>
      </div>

      <div className="mt-4 space-y-2">
        <button
          className={clsx(
            'px-4',
            'py-2',
            'rounded',
            'w-full',
            'text-sm',
            'font-medium',
            'transition-colors',
            {
              'bg-blue-500 text-white hover:bg-blue-600': isEditable,
              'bg-gray-400 text-gray-600 cursor-not-allowed': !isEditable,
            },
            'hidden'
          )}
          onClick={() => setShowPopup(true)}
          disabled={!isEditable}
        >
          Load Previous Task Info
        </button>
      </div>

      {showPopup && (
        <div className="fixed inset-0 bg-black bg-opacity-40 flex items-center justify-center z-50">
          <div className="bg-white p-6 rounded-lg shadow-lg max-w-2xl w-full">
            <div className="mb-4 font-bold text-lg">Select Task Info</div>
            <div className="grid grid-cols-2 gap-4">
              {taskInfoList.map((item, idx) => (
                <div
                  key={idx}
                  className="p-4 border rounded cursor-pointer hover:bg-blue-100"
                  onClick={() => handleSelect(item)}
                >
                  <div className="font-semibold">{item.taskName}</div>
                  <div className="text-sm text-gray-600">{item.taskType}</div>
                  <div className="text-xs text-gray-400">{item.repoId}</div>
                </div>
              ))}
            </div>
            <button
              className="mt-6 px-4 py-2 bg-gray-400 text-white rounded"
              onClick={() => setShowPopup(false)}
            >
              Cancel
            </button>
          </div>
        </div>
      )}

      {/* Token Input Popup */}
      <TokenInputPopup
        isOpen={showTokenPopup}
        onClose={() => setShowTokenPopup(false)}
        onSubmit={handleTokenSubmit}
        isLoading={isLoading}
      />
    </div>
  );
};

export default InfoPanel;
