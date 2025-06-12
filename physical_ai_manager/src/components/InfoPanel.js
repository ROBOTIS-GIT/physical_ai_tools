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

import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import TagInput from './TagInput';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';

const dummyTaskInfoList = [
  {
    taskName: 'Task 1',
    robotType: 'Type A',
    taskType: 'Type X',
    taskInstruction: 'Do X',
    repoId: 'repo1',
    fps: 30,
    tags: ['tag1'],
    warmupTime: '10',
    episodeTime: '60',
    resetTime: '5',
    numEpisodes: 3,
    resume: false,
    pushToHub: false,
  },
  {
    taskName: 'Task 2',
    robotType: 'Type B',
    taskType: 'Type Y',
    taskInstruction: 'Do Y',
    repoId: 'repo2',
    fps: 20,
    tags: ['tag2'],
    warmupTime: '5',
    episodeTime: '30',
    resetTime: '2',
    numEpisodes: 5,
    resume: true,
    pushToHub: true,
  },
];

const InfoPanel = ({ info, onChange, disabled = false }) => {
  const [showPopup, setShowPopup] = useState(false);
  const [taskInfoList] = useState(dummyTaskInfoList);
  const [isEditable, setIsEditable] = useState(!disabled);

  // User ID list for dropdown
  const [userIdList, setUserIdList] = useState([
    'user123',
    'robotics_team',
    'ai_researcher',
    'demo_user',
    'test_account',
  ]);

  // Token popup states
  const [showTokenPopup, setShowTokenPopup] = useState(false);
  const [tokenInput, setTokenInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  // User ID selection states
  const [showUserIdDropdown, setShowUserIdDropdown] = useState(false);

  // ROS service caller
  const { registerHFUser, getRegisteredHFUser } = useRosServiceCaller();

  const handleChange = (field, value) => {
    if (!isEditable) return; // Block changes when not editable
    onChange({ ...info, [field]: value });
  };

  const handleSelect = (selected) => {
    onChange(selected);
    setShowPopup(false);
  };

  const handleTokenSubmit = async () => {
    if (!tokenInput.trim()) {
      alert('Please enter a token');
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
        alert('User ID list updated successfully!');
      } else {
        alert('Failed to get user ID list from response');
      }
    } catch (error) {
      console.error('Error registering HF user:', error);
      alert(`Failed to register user: ${error.message}`);
    } finally {
      setIsLoading(false);
    }
  };

  const handleLoadUserId = async () => {
    setIsLoading(true);
    try {
      const result = await getRegisteredHFUser();
      console.log('getRegisteredHFUser result:', result);

      if (result && result.user_id_list) {
        setUserIdList(result.user_id_list);
        if (info.pushToHub) {
          alert('User ID list loaded successfully!');
        } else {
          // For non-push-to-hub mode, show dropdown for selection
          setShowUserIdDropdown(true);
        }
      } else {
        alert('Failed to get user ID list from response');
      }
    } catch (error) {
      console.error('Error loading HF user list:', error);
      alert(`Failed to load user ID list: ${error.message}`);
    } finally {
      setIsLoading(false);
    }
  };

  const handleUserIdSelect = (selectedUserId) => {
    handleChange('repoId', selectedUserId);
    setShowUserIdDropdown(false);
  };

  // Update isEditable state when the disabled prop changes
  useEffect(() => {
    setIsEditable(!disabled);
  }, [disabled]);

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
    'relative'
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

  const classTokenTextarea = clsx(
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
      'bg-gray-100 cursor-not-allowed': !isEditable,
      'bg-white': isEditable,
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

  // Task type options
  const taskTypeOptions = [
    { value: 'record', label: 'Record' },
    { value: 'inference', label: 'Inference' },
  ];

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
        <textarea
          className={classTaskInstructionTextarea}
          value={info.taskInstruction || ''}
          onChange={(e) => handleChange('taskInstruction', e.target.value)}
          disabled={!isEditable}
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
            disabled={!isEditable}
          />
          <span className={clsx('ml-2', 'text-sm', 'text-gray-500')}>
            {info.pushToHub ? 'Enabled' : 'Disabled'}
          </span>
        </div>
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
          User ID
        </span>

        <div className="flex-1 min-w-0">
          {info.pushToHub ? (
            /* Dropdown selection only when Push to Hub is enabled */
            <>
              <div className="flex flex-row gap-2 mb-2">
                <button
                  className={clsx(
                    'px-3',
                    'py-1',
                    'text-xs',
                    'font-medium',
                    'rounded',
                    'transition-colors',
                    {
                      'bg-blue-500 text-white hover:bg-blue-600': isEditable && !isLoading,
                      'bg-gray-400 text-gray-600 cursor-not-allowed': !isEditable || isLoading,
                    }
                  )}
                  onClick={() => {
                    if (isEditable && !isLoading) {
                      handleLoadUserId();
                    }
                  }}
                  disabled={!isEditable || isLoading}
                >
                  {isLoading ? 'Loading...' : 'Load User ID'}
                </button>
                <button
                  className={clsx(
                    'px-3',
                    'py-1',
                    'text-xs',
                    'font-medium',
                    'rounded',
                    'transition-colors',
                    {
                      'bg-green-500 text-white hover:bg-green-600': isEditable && !isLoading,
                      'bg-gray-400 text-gray-600 cursor-not-allowed': !isEditable || isLoading,
                    }
                  )}
                  onClick={() => {
                    if (isEditable && !isLoading) {
                      setShowTokenPopup(true);
                    }
                  }}
                  disabled={!isEditable || isLoading}
                >
                  Set
                </button>
              </div>
              <select
                className={classSelect}
                value={info.repoId || ''}
                onChange={(e) => handleChange('repoId', e.target.value)}
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
                    value={info.repoId || ''}
                    onChange={(e) => handleChange('repoId', e.target.value)}
                    disabled={!isEditable}
                    placeholder="Enter User ID or load from registered IDs"
                  />
                  <button
                    className={clsx(
                      'mt-2',
                      'px-3',
                      'py-1',
                      'text-xs',
                      'font-medium',
                      'rounded',
                      'transition-colors',
                      {
                        'bg-blue-500 text-white hover:bg-blue-600': isEditable && !isLoading,
                        'bg-gray-400 text-gray-600 cursor-not-allowed': !isEditable || isLoading,
                      }
                    )}
                    onClick={() => {
                      if (isEditable && !isLoading) {
                        handleLoadUserId();
                      }
                    }}
                    disabled={!isEditable || isLoading}
                  >
                    {isLoading ? 'Loading...' : 'Load from Registered IDs'}
                  </button>
                  <div className="text-xs text-gray-500 mt-1 leading-relaxed">
                    Enter any User ID or load from registered IDs
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
                  <div className="flex gap-2 mt-2">
                    <button
                      className="px-3 py-1 text-xs font-medium rounded bg-gray-500 text-white hover:bg-gray-600 transition-colors"
                      onClick={() => setShowUserIdDropdown(false)}
                      disabled={!isEditable}
                    >
                      Cancel
                    </button>
                  </div>
                  <div className="text-xs text-gray-500 mt-1 leading-relaxed">
                    Select a registered User ID or cancel to enter manually
                  </div>
                </>
              )}
            </>
          )}
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
          value={info.warmupTime || ''}
          onChange={(e) => handleChange('warmupTime', e.target.value)}
          disabled={!isEditable}
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>Episode Time (s)</span>
        <input
          className={classTextInput}
          type="number"
          step="5"
          value={info.episodeTime || ''}
          onChange={(e) => handleChange('episodeTime', e.target.value)}
          disabled={!isEditable}
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>Reset Time (s)</span>
        <input
          className={classTextInput}
          type="number"
          step="5"
          value={info.resetTime || ''}
          onChange={(e) => handleChange('resetTime', e.target.value)}
          disabled={!isEditable}
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>Num Episodes</span>
        <input
          className={classTextInput}
          type="number"
          step="1"
          value={info.numEpisodes || ''}
          onChange={(e) => handleChange('numEpisodes', Number(e.target.value))}
          disabled={!isEditable}
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2')}>
        <span className={classLabel}>Use Image Buffer</span>
        <div className={clsx('flex', 'items-center')}>
          <input
            className={classCheckbox}
            type="checkbox"
            checked={!!info.useImageBuffer}
            onChange={(e) => handleChange('useImageBuffer', e.target.checked)}
            disabled={!isEditable}
          />
          <span className={clsx('ml-2', 'text-sm', 'text-gray-500')}>
            {info.pushToHub ? 'Enabled' : 'Disabled'}
          </span>
        </div>
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>Resume</span>
        <div className={clsx('flex', 'items-center')}>
          <input
            className={classCheckbox}
            type="checkbox"
            checked={!!info.resume}
            onChange={(e) => handleChange('resume', e.target.checked)}
            disabled={!isEditable}
          />
          <span className={clsx('ml-2', 'text-sm', 'text-gray-500')}>
            {info.resume ? 'Enabled' : 'Disabled'}
          </span>
        </div>
      </div>

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
            {info.pushToHub ? 'Enabled' : 'Disabled'}
          </span>
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

      {showTokenPopup && (
        <div className="fixed inset-0 bg-black bg-opacity-40 flex items-center justify-center z-50">
          <div className="bg-white p-6 rounded-lg shadow-lg max-w-md w-full">
            <div className="mb-4 font-bold text-lg">Enter Hugging Face Token</div>
            <div className="mb-4">
              <label className="block text-sm font-medium text-gray-700 mb-2">Token</label>
              <input
                type="password"
                className={clsx(
                  'w-full',
                  'p-3',
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
    </div>
  );
};

export default InfoPanel;
