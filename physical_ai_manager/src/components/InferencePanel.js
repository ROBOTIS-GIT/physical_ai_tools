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
import toast from 'react-hot-toast';
import { MdVisibility, MdVisibilityOff, MdFolderOpen, MdDownload } from 'react-icons/md';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import TagInput from './TagInput';
import FileBrowserModal from './FileBrowserModal';
import PolicyDownloadModal from './PolicyDownloadModal';
import TaskPhase from '../constants/taskPhases';
import { DEFAULT_PATHS, TARGET_FILES } from '../constants/paths';
import { setTaskInfo } from '../features/tasks/taskSlice';

const InferencePanel = () => {
  const dispatch = useDispatch();

  const info = useSelector((state) => state.tasks.taskInfo);
  const taskStatus = useSelector((state) => state.tasks.taskStatus);

  const [isTaskStatusPaused, setIsTaskStatusPaused] = useState(false);
  const [lastTaskStatusUpdate, setLastTaskStatusUpdate] = useState(Date.now());

  // Calculate if the panel should be disabled based on task status
  const disabled = taskStatus.phase !== TaskPhase.READY || !isTaskStatusPaused;
  const [isEditable, setIsEditable] = useState(!disabled);

  // User ID list for dropdown
  const [userIdList, setUserIdList] = useState([]);

  // Token popup states
  const [showTokenPopup, setShowTokenPopup] = useState(false);
  const [tokenInput, setTokenInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [showPassword, setShowPassword] = useState(false);

  // User ID selection states
  const [showUserIdDropdown, setShowUserIdDropdown] = useState(false);

  // File browser modal states
  const [showPolicyPathModal, setShowPolicyPathModal] = useState(false);

  // Policy download modal states
  const [showPolicyDownloadModal, setShowPolicyDownloadModal] = useState(false);

  const { registerHFUser, getRegisteredHFUser } = useRosServiceCaller();

  const handleChange = useCallback(
    (field, value) => {
      if (!isEditable) return; // Block changes when not editable
      dispatch(setTaskInfo({ ...info, [field]: value }));
    },
    [isEditable, info, dispatch]
  );

  const handlePolicyPathSelect = useCallback(
    (item) => {
      if (!isEditable) return;
      handleChange('policyPath', item.full_path);
      setShowPolicyPathModal(false);
    },
    [isEditable, handleChange]
  );

  const handleTokenSubmit = async () => {
    if (!tokenInput.trim()) {
      toast.error('Please enter a token');
      return;
    }

    setIsLoading(true);
    try {
      const result = await registerHFUser(tokenInput.trim());
      console.log('registerHFUser result:', result);

      if (result && result.user_id_list) {
        setUserIdList(result.user_id_list);
        setShowTokenPopup(false);
        setTokenInput('');
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
        setUserIdList(result.user_id_list);
        toast.success('User ID list loaded successfully!');
        setShowUserIdDropdown(true);
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

  const handleDownloadPolicyComplete = useCallback(
    (repoId) => {
      // Update the policy path with the local cache path
      const localPath = `${DEFAULT_PATHS.POLICY_MODEL_PATH}${repoId}`;
      handleChange('policyPath', localPath);
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
    'min-h-10',
    'max-h-20',
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

  const classPolicyPathTextarea = clsx(
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

  const classRepoIdTextarea = clsx(
    'text-sm',
    'resize-y',
    'min-h-10',
    'max-h-24',
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
        <textarea
          className={classTaskInstructionTextarea}
          value={info.taskInstruction || ''}
          onChange={(e) => handleChange('taskInstruction', [e.target.value])}
          disabled={!isEditable}
          placeholder="Enter Task Instruction"
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
          Policy Path
        </span>
        <div className={clsx('flex', 'flex-col', 'flex-1', 'gap-2')}>
          {/* Browse Policy Path Button */}
          <button
            onClick={() => setShowPolicyPathModal(true)}
            disabled={!isEditable}
            className={clsx(
              'flex',
              'items-center',
              'gap-2',
              'px-3',
              'py-2',
              'text-sm',
              'bg-blue-50',
              'text-blue-700',
              'border',
              'border-blue-200',
              'rounded-lg',
              'hover:bg-blue-100',
              'transition-colors',
              'disabled:bg-gray-100',
              'disabled:text-gray-400',
              'disabled:cursor-not-allowed',
              'w-fit'
            )}
          >
            <MdFolderOpen size={16} />
            Browse Policy Path
          </button>
          {/* Download Policy Button */}
          <button
            disabled={!isEditable}
            onClick={() => setShowPolicyDownloadModal(true)}
            className={clsx(
              'flex',
              'items-center',
              'gap-2',
              'px-3',
              'py-2',
              'text-sm',
              'bg-green-50',
              'text-green-700',
              'border',
              'border-green-200',
              'rounded-lg',
              'hover:bg-green-100',
              'transition-colors',
              'disabled:bg-gray-100',
              'disabled:text-gray-400',
              'disabled:cursor-not-allowed',
              'w-fit'
            )}
          >
            <MdDownload size={16} />
            Download Policy
          </button>
          <textarea
            className={classPolicyPathTextarea}
            value={info.policyPath || ''}
            onChange={(e) => handleChange('policyPath', e.target.value)}
            disabled={!isEditable}
            placeholder="Enter Policy Path or Repo ID"
          />
        </div>
      </div>

      <div className="w-full h-1 my-2 border-t border-gray-300"></div>

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

      <div className="text-xs text-gray-400 mt-1 ml-2">
        Recording during inference will be supported in a future update
      </div>

      {false && (
        <>
          <div className="h-3 w-full"></div>
          <div className={clsx('flex', 'items-center', 'mb-2')}>
            <span className={classLabel}>Record</span>
            <div className={clsx('flex', 'items-center')}>
              <input
                className={classCheckbox}
                type="checkbox"
                checked={info.recordInferenceMode}
                onChange={(e) => handleChange('recordInferenceMode', e.target.checked)}
                disabled={true}
              />
              <span className={clsx('ml-2', 'text-sm', 'text-gray-500')}>
                {info.recordInferenceMode ? 'Enabled' : 'Disabled'}
              </span>
            </div>
          </div>

          <div className={clsx('flex', 'items-center', 'mb-2.5')}>
            <span className={classLabel}>Task Name</span>
            <textarea
              className={classTaskNameTextarea}
              value={info.taskName || ''}
              onChange={(e) => handleChange('taskName', e.target.value)}
              disabled={!isEditable || !info.recordInferenceMode}
              placeholder="Enter Task Name"
            />
          </div>

          <div className={clsx('flex', 'items-center', 'mb-2')}>
            <span className={classLabel}>Push to Hub</span>
            <div className={clsx('flex', 'items-center')}>
              <input
                className={classCheckbox}
                type="checkbox"
                checked={!!info.pushToHub}
                onChange={(e) => handleChange('pushToHub', e.target.checked)}
                disabled={!isEditable || !info.recordInferenceMode}
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
                  disabled={!isEditable || !info.recordInferenceMode}
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
                  className={clsx(
                    classButtonBase,
                    getButtonVariant('blue', isEditable && info.recordInferenceMode, isLoading)
                  )}
                  onClick={() => {
                    if (isEditable && !isLoading && info.recordInferenceMode) {
                      handleLoadUserId();
                    }
                  }}
                  disabled={!isEditable || isLoading || !info.recordInferenceMode}
                >
                  {isLoading ? 'Loading...' : 'Load'}
                </button>
                {!info.pushToHub && showUserIdDropdown && (
                  <button
                    className={clsx(
                      classButtonBase,
                      getButtonVariant('red', isEditable && info.recordInferenceMode)
                    )}
                    onClick={() => setShowUserIdDropdown(false)}
                    disabled={!isEditable || !info.recordInferenceMode}
                  >
                    Manual Input
                  </button>
                )}
                {info.pushToHub && (
                  <button
                    className={clsx(
                      classButtonBase,
                      getButtonVariant('green', isEditable && info.recordInferenceMode, isLoading)
                    )}
                    onClick={() => {
                      if (isEditable && !isLoading && info.recordInferenceMode) {
                        setShowTokenPopup(true);
                      }
                    }}
                    disabled={!isEditable || isLoading || !info.recordInferenceMode}
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
                    disabled={!isEditable || !info.recordInferenceMode}
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
                        disabled={!isEditable || !info.recordInferenceMode}
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
                        disabled={!isEditable || !info.recordInferenceMode}
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

          <div className={clsx('flex', 'items-start', 'mb-2.5')}>
            <span className={clsx(classLabel, 'pt-2')}>Tags</span>
            <div className="flex-1 min-w-0">
              <TagInput
                tags={info.tags || []}
                onChange={(newTags) => handleChange('tags', newTags)}
                disabled={!isEditable || !info.recordInferenceMode}
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
              disabled={!isEditable || !info.recordInferenceMode}
            />
          </div>

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
              disabled={!isEditable || !info.recordInferenceMode}
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
              disabled={!isEditable || !info.recordInferenceMode}
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
              disabled={!isEditable || !info.recordInferenceMode}
            />
          </div>

          <div className={clsx('flex', 'items-center', 'mb-2')}>
            <span className={classLabel}>Optimized Save</span>
            <div className={clsx('flex', 'items-center')}>
              <input
                className={classCheckbox}
                type="checkbox"
                checked={!!info.useOptimizedSave}
                onChange={(e) => handleChange('useOptimizedSave', e.target.checked)}
                disabled={!isEditable || !info.recordInferenceMode}
              />
              <span className={clsx('ml-2', 'text-sm', 'text-gray-500')}>
                {info.useOptimizedSave ? 'Enabled' : 'Disabled'}
              </span>
            </div>
          </div>
        </>
      )}

      {/* Input Hugging Face Token Popup */}
      {showTokenPopup && (
        <div className="fixed inset-0 bg-black bg-opacity-40 flex items-center justify-center z-50">
          <div className="bg-white p-6 rounded-lg shadow-lg max-w-md w-full">
            <div className="mb-4 font-bold text-lg">Enter Hugging Face Token</div>
            <div className="mb-4">
              <label className="block text-sm font-medium text-gray-700 mb-2">Token</label>
              <div className="relative">
                <input
                  type={showPassword ? 'text' : 'password'}
                  className={clsx(
                    'w-full',
                    'p-3',
                    'pr-10',
                    'border',
                    'border-gray-300',
                    'rounded-md',
                    'focus:outline-none',
                    'focus:ring-2',
                    'focus:ring-blue-500',
                    'focus:border-transparent'
                  )}
                  value={tokenInput}
                  onChange={(e) => setTokenInput(e.target.value)}
                  placeholder="Enter your Hugging Face token"
                  disabled={isLoading}
                />
                <button
                  type="button"
                  className="absolute inset-y-0 right-0 pr-3 flex items-center"
                  onClick={() => setShowPassword(!showPassword)}
                  disabled={isLoading}
                >
                  {showPassword ? (
                    <MdVisibilityOff className="h-5 w-5 text-gray-400 hover:text-gray-600" />
                  ) : (
                    <MdVisibility className="h-5 w-5 text-gray-400 hover:text-gray-600" />
                  )}
                </button>
              </div>
              <div className="text-xs text-gray-500 mt-1">
                This token will be used to fetch your available User IDs
              </div>
            </div>
            <div className="flex gap-3">
              <button
                className={clsx(
                  'flex-1',
                  'px-4',
                  'py-2',
                  'rounded',
                  'font-medium',
                  'transition-colors',
                  {
                    'bg-blue-500 text-white hover:bg-blue-600': !isLoading,
                    'bg-gray-400 text-gray-600 cursor-not-allowed': isLoading,
                  }
                )}
                onClick={handleTokenSubmit}
                disabled={isLoading}
              >
                {isLoading ? 'Loading...' : 'Submit'}
              </button>
              <button
                className="flex-1 px-4 py-2 bg-gray-400 text-white rounded hover:bg-gray-500 transition-colors"
                onClick={() => {
                  setShowTokenPopup(false);
                  setTokenInput('');
                }}
                disabled={isLoading}
              >
                Cancel
              </button>
            </div>
          </div>
        </div>
      )}

      <FileBrowserModal
        isOpen={showPolicyPathModal}
        onClose={() => setShowPolicyPathModal(false)}
        onFileSelect={handlePolicyPathSelect}
        title="Select Policy Path"
        selectButtonText="Select"
        allowDirectorySelect={true}
        targetFileName={[TARGET_FILES.POLICY_MODEL]}
        targetFileLabel="Policy file found! 🎯"
        initialPath={DEFAULT_PATHS.POLICY_MODEL_PATH}
        defaultPath={DEFAULT_PATHS.POLICY_MODEL_PATH}
        homePath=""
      />

      <PolicyDownloadModal
        isOpen={showPolicyDownloadModal}
        onClose={() => setShowPolicyDownloadModal(false)}
        onDownloadComplete={handleDownloadPolicyComplete}
      />
    </div>
  );
};

export default InferencePanel;
