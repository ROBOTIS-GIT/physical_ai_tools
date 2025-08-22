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

const TaskInstructionInput = ({ instructions = [''], onChange, disabled, className }) => {
  const [localInstructions, setLocalInstructions] = useState(() => {
    // Ensure instructions is always an array
    if (Array.isArray(instructions) && instructions.length > 0) {
      return instructions;
    }
    return [''];
  });

  useEffect(() => {
    // Add proper type checking to ensure instructions is an array
    if (Array.isArray(instructions)) {
      setLocalInstructions(instructions.length > 0 ? instructions : ['']);
    } else {
      setLocalInstructions(['']);
    }
  }, [instructions]);

  const addInstruction = () => {
    const newInstructions = [...localInstructions, ''];
    setLocalInstructions(newInstructions);
    onChange(newInstructions);
  };

  const removeInstruction = (index) => {
    if (localInstructions.length > 1) {
      const newInstructions = localInstructions.filter((_, i) => i !== index);
      setLocalInstructions(newInstructions);
      onChange(newInstructions);
    }
  };

  const updateInstruction = (index, value) => {
    const newInstructions = [...localInstructions];
    newInstructions[index] = value;
    setLocalInstructions(newInstructions);
    onChange(newInstructions);
  };

  return (
    <div className={clsx('w-full', className)}>
      <div className="max-h-48 overflow-y-auto border border-gray-300 rounded-md bg-white scrollbar-thin">
        <div className="p-2 space-y-2">
          {localInstructions.map((instruction, index) => (
            <div key={index} className="relative">
              <textarea
                value={instruction}
                onChange={(e) => updateInstruction(index, e.target.value)}
                disabled={disabled}
                placeholder={`Task instruction ${index + 1}`}
                className={clsx(
                  'w-full',
                  'p-2',
                  'border',
                  'border-gray-200',
                  'rounded',
                  'text-sm',
                  'resize-none',
                  'min-h-8',
                  'h-14',
                  'focus:ring-2',
                  'focus:ring-blue-500',
                  'focus:border-transparent',
                  {
                    'bg-gray-100 cursor-not-allowed': disabled,
                    'bg-white': !disabled,
                    'pr-10': !disabled && localInstructions.length > 1,
                  }
                )}
                rows={2}
              />
              {!disabled && localInstructions.length > 1 && (
                <button
                  type="button"
                  onClick={() => removeInstruction(index)}
                  className={clsx(
                    'absolute',
                    'top-2',
                    'right-2',
                    'w-6',
                    'h-6',
                    'bg-red-100',
                    'text-red-600',
                    'rounded-full',
                    'hover:bg-red-200',
                    'focus:outline-none',
                    'focus:ring-2',
                    'focus:ring-red-500',
                    'flex',
                    'items-center',
                    'justify-center',
                    'text-sm',
                    'font-medium'
                  )}
                >
                  ×
                </button>
              )}
            </div>
          ))}
        </div>
      </div>

      {!disabled && (
        <div className="mt-3 flex justify-between items-center">
          <button
            type="button"
            onClick={addInstruction}
            className={clsx(
              'px-3',
              'py-1',
              'bg-blue-500',
              'text-white',
              'rounded',
              'hover:bg-blue-600',
              'focus:outline-none',
              'focus:ring-2',
              'focus:ring-blue-500',
              'flex',
              'items-center',
              'gap-2',
              'text-sm',
              'font-medium'
            )}
          >
            <span className="text-base font-bold">+</span>
            Add Instruction
          </button>
          <span className="block text-xs text-gray-600 mb-0.5 px-0 select-none">
            <span role="img" aria-label="task">
              📝
            </span>{' '}
            {localInstructions.length}
          </span>
        </div>
      )}
    </div>
  );
};

export default TaskInstructionInput;
