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

import React, { useState, useRef } from 'react';
import clsx from 'clsx';

const TagInput = ({ tags, onChange, disabled, className }) => {
  const [inputValue, setInputValue] = useState('');
  const inputRef = useRef(null);

  const addTag = (tag) => {
    const trimmedTag = tag.trim();
    if (trimmedTag && !tags.includes(trimmedTag)) {
      onChange([...tags, trimmedTag]);
    }
    setInputValue('');
  };

  const removeTag = (indexToRemove) => {
    onChange(tags.filter((_, index) => index !== indexToRemove));
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter') {
      e.preventDefault();
      addTag(inputValue);
    } else if (e.key === 'Backspace' && inputValue === '' && tags.length > 0) {
      // Remove last tag when backspace is pressed and input is empty
      removeTag(tags.length - 1);
    }
  };

  const handleInputChange = (e) => {
    const value = e.target.value;
    // Allow comma-separated input
    if (value.includes(',')) {
      const newTags = value
        .split(',')
        .map((t) => t.trim())
        .filter((t) => t && !tags.includes(t));
      if (newTags.length > 0) {
        onChange([...tags, ...newTags]);
      }
      setInputValue('');
    } else {
      setInputValue(value);
    }
  };

  return (
    <div
      className={clsx(
        'flex',
        'flex-wrap',
        'gap-1',
        'p-2',
        'border',
        'border-gray-300',
        'rounded-md',
        'min-h-10',
        'max-w-full',
        'w-full',
        'text-sm',
        'focus-within:ring-2',
        'focus-within:ring-blue-500',
        'focus-within:border-transparent',
        'max-h-24',
        'overflow-y-auto',
        {
          'bg-gray-100 cursor-not-allowed': disabled,
          'bg-white': !disabled,
        },
        className
      )}
      onClick={() => {
        if (!disabled && inputRef.current) {
          inputRef.current.focus();
        }
      }}
    >
      {/* Render existing tags */}
      {tags.map((tag, index) => (
        <span
          key={index}
          className={clsx(
            'inline-flex',
            'items-center',
            'px-2',
            'py-1',
            'bg-blue-100',
            'text-blue-800',
            'text-xs',
            'font-medium',
            'rounded-full',
            'max-w-[80%]',
            'break-all',
            {
              'cursor-default': disabled,
              'cursor-pointer': !disabled,
            }
          )}
        >
          {tag}
          {!disabled && (
            <button
              type="button"
              onClick={() => removeTag(index)}
              className={clsx('ml-1', 'text-blue-600', 'hover:text-blue-800', 'focus:outline-none')}
            >
              ×
            </button>
          )}
        </span>
      ))}

      {/* Input for new tags */}
      <input
        ref={inputRef}
        type="text"
        value={inputValue}
        onChange={handleInputChange}
        onKeyDown={handleKeyDown}
        disabled={disabled}
        placeholder={tags.length === 0 ? 'Add tags' : ''}
        className={clsx(
          'w-auto',
          'min-w-12',
          'max-w-[40%]',
          'break-all',
          'bg-transparent',
          'border-none',
          'outline-none',
          'text-sm',
          {
            'cursor-not-allowed': disabled,
            'cursor-text': !disabled,
          }
        )}
      />
    </div>
  );
};

export default TagInput;
