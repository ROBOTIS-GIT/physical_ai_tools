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

  const handleChange = (field, value) => {
    if (!isEditable) return; // Block changes when not editable
    onChange({ ...info, [field]: value });
  };

  const handleSelect = (selected) => {
    onChange(selected);
    setShowPopup(false);
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
            <div className="text-xs mt-1 opacity-80">
              (task is running or robot is not connected)
            </div>
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

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>Robot Type</span>
        <input
          className={classTextInput}
          type="text"
          value={info.robotType || ''}
          onChange={(e) => handleChange('robotType', e.target.value)}
          disabled={!isEditable}
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>Task Type</span>
        <input
          className={classTextInput}
          type="text"
          value={info.taskType || ''}
          onChange={(e) => handleChange('taskType', e.target.value)}
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
          Repo ID
        </span>
        <textarea
          className={classRepoIdTextarea}
          value={info.repoId || ''}
          onChange={(e) => handleChange('repoId', e.target.value)}
          disabled={!isEditable}
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>FPS</span>
        <input
          className={classTextInput}
          type="number"
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
          value={info.numEpisodes || ''}
          onChange={(e) => handleChange('numEpisodes', Number(e.target.value))}
          disabled={!isEditable}
        />
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
    </div>
  );
};

export default InfoPanel;
