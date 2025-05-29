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

import React from 'react';
import clsx from 'clsx';

const InfoPanel = ({ info, onChange }) => {
  const handleChange = (field, value) => {
    onChange({ ...info, [field]: value });
  };

  return (
    <div
      className={clsx(
        'bg-white',
        'border',
        'border-gray-200',
        'rounded-2xl',
        'shadow-md',
        'p-4',
        'w-full',
        'max-w-[350px]'
      )}
    >
      <div className={clsx('text-lg', 'font-semibold', 'mb-3', 'text-gray-800')}>
        Task Information
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={clsx('text-sm', 'text-gray-600', 'w-28', 'flex-shrink-0', 'font-medium')}>
          Task Name
        </span>
        <textarea
          className={clsx(
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
            'focus:border-transparent'
          )}
          value={info.taskName || ''}
          onChange={(e) => handleChange('taskName', e.target.value)}
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={clsx('text-sm', 'text-gray-600', 'w-28', 'flex-shrink-0', 'font-medium')}>
          Robot Type
        </span>
        <input
          className={clsx(
            'text-sm',
            'w-full',
            'p-2',
            'border',
            'border-gray-300',
            'rounded-md',
            'focus:outline-none',
            'focus:ring-2',
            'focus:ring-blue-500',
            'focus:border-transparent'
          )}
          type="text"
          value={info.robotType || ''}
          onChange={(e) => handleChange('robotType', e.target.value)}
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={clsx('text-sm', 'text-gray-600', 'w-28', 'flex-shrink-0', 'font-medium')}>
          Task Type
        </span>
        <input
          className={clsx(
            'text-sm',
            'w-full',
            'p-2',
            'border',
            'border-gray-300',
            'rounded-md',
            'focus:outline-none',
            'focus:ring-2',
            'focus:ring-blue-500',
            'focus:border-transparent'
          )}
          type="text"
          value={info.taskType || ''}
          onChange={(e) => handleChange('taskType', e.target.value)}
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
          className={clsx(
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
            'focus:border-transparent'
          )}
          value={info.taskInstruction || ''}
          onChange={(e) => handleChange('taskInstruction', e.target.value)}
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
          className={clsx(
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
            'focus:border-transparent'
          )}
          value={info.repoId || ''}
          onChange={(e) => handleChange('repoId', e.target.value)}
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={clsx('text-sm', 'text-gray-600', 'w-28', 'flex-shrink-0', 'font-medium')}>
          FPS
        </span>
        <input
          className={clsx(
            'text-sm',
            'w-full',
            'p-2',
            'border',
            'border-gray-300',
            'rounded-md',
            'focus:outline-none',
            'focus:ring-2',
            'focus:ring-blue-500',
            'focus:border-transparent'
          )}
          type="number"
          value={info.fps || ''}
          onChange={(e) => handleChange('fps', Number(e.target.value))}
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={clsx('text-sm', 'text-gray-600', 'w-28', 'flex-shrink-0', 'font-medium')}>
          Tags
        </span>
        <input
          className={clsx(
            'text-sm',
            'w-full',
            'p-2',
            'border',
            'border-gray-300',
            'rounded-md',
            'focus:outline-none',
            'focus:ring-2',
            'focus:ring-blue-500',
            'focus:border-transparent',
            'bg-gray-100', // TODO: remove this when enable tags
            'cursor-not-allowed'
          )}
          type="text"
          value={Array.isArray(info.tags) ? info.tags.join(', ') : ''}
          onChange={(e) =>
            handleChange(
              'tags',
              e.target.value.split(',').map((s) => s.trim())
            )
          }
          disabled // TODO: remove this when enable tags
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={clsx('text-sm', 'text-gray-600', 'w-28', 'flex-shrink-0', 'font-medium')}>
          Warmup Time (s)
        </span>
        <input
          className={clsx(
            'text-sm',
            'w-full',
            'p-2',
            'border',
            'border-gray-300',
            'rounded-md',
            'focus:outline-none',
            'focus:ring-2',
            'focus:ring-blue-500',
            'focus:border-transparent'
          )}
          type="text"
          value={info.warmupTime || ''}
          onChange={(e) => handleChange('warmupTime', e.target.value)}
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={clsx('text-sm', 'text-gray-600', 'w-28', 'flex-shrink-0', 'font-medium')}>
          Episode Time (s)
        </span>
        <input
          className={clsx(
            'text-sm',
            'w-full',
            'p-2',
            'border',
            'border-gray-300',
            'rounded-md',
            'focus:outline-none',
            'focus:ring-2',
            'focus:ring-blue-500',
            'focus:border-transparent'
          )}
          type="text"
          value={info.episodeTime || ''}
          onChange={(e) => handleChange('episodeTime', e.target.value)}
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={clsx('text-sm', 'text-gray-600', 'w-28', 'flex-shrink-0', 'font-medium')}>
          Reset Time (s)
        </span>
        <input
          className={clsx(
            'text-sm',
            'w-full',
            'p-2',
            'border',
            'border-gray-300',
            'rounded-md',
            'focus:outline-none',
            'focus:ring-2',
            'focus:ring-blue-500',
            'focus:border-transparent'
          )}
          type="text"
          value={info.resetTime || ''}
          onChange={(e) => handleChange('resetTime', e.target.value)}
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={clsx('text-sm', 'text-gray-600', 'w-28', 'flex-shrink-0', 'font-medium')}>
          Num Episodes
        </span>
        <input
          className={clsx(
            'text-sm',
            'w-full',
            'p-2',
            'border',
            'border-gray-300',
            'rounded-md',
            'focus:outline-none',
            'focus:ring-2',
            'focus:ring-blue-500',
            'focus:border-transparent'
          )}
          type="number"
          value={info.numEpisodes || ''}
          onChange={(e) => handleChange('numEpisodes', Number(e.target.value))}
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={clsx('text-sm', 'text-gray-600', 'w-28', 'flex-shrink-0', 'font-medium')}>
          Resume
        </span>
        <div className={clsx('flex', 'items-center')}>
          <input
            className={clsx(
              'w-4',
              'h-4',
              'text-blue-600',
              'bg-gray-100',
              'border-gray-300',
              'rounded',
              'focus:ring-blue-500',
              'focus:ring-2'
            )}
            type="checkbox"
            checked={!!info.resume}
            onChange={(e) => handleChange('resume', e.target.checked)}
          />
          <span className={clsx('ml-2', 'text-sm', 'text-gray-500')}>
            {info.resume ? 'Enabled' : 'Disabled'}
          </span>
        </div>
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2')}>
        <span className={clsx('text-sm', 'text-gray-600', 'w-28', 'flex-shrink-0', 'font-medium')}>
          Push to Hub
        </span>
        <div className={clsx('flex', 'items-center')}>
          <input
            className={clsx(
              'w-4',
              'h-4',
              'text-blue-600',
              'bg-gray-100',
              'border-gray-300',
              'rounded',
              'focus:ring-blue-500',
              'focus:ring-2'
            )}
            type="checkbox"
            checked={!!info.pushToHub}
            onChange={(e) => handleChange('pushToHub', e.target.checked)}
          />
          <span className={clsx('ml-2', 'text-sm', 'text-gray-500')}>
            {info.pushToHub ? 'Enabled' : 'Disabled'}
          </span>
        </div>
      </div>
    </div>
  );
};

export default InfoPanel;
