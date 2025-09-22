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

import React, { useState } from 'react';
import clsx from 'clsx';
import { MdVisibility, MdVisibilityOff } from 'react-icons/md';

const TokenInputPopup = ({
  isOpen,
  onClose,
  onSubmit,
  isLoading = false,
  title = 'Enter Hugging Face Token',
}) => {
  const [tokenInput, setTokenInput] = useState('');
  const [showPassword, setShowPassword] = useState(false);

  // Handle popup close and reset state
  const handleClose = () => {
    setTokenInput('');
    setShowPassword(false);
    onClose();
  };

  // Handle token submission
  const handleSubmit = () => {
    onSubmit(tokenInput.trim());
    setTokenInput('');
    setShowPassword(false);
  };

  // Handle Enter key press
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !isLoading) {
      handleSubmit();
    }
  };

  if (!isOpen) {
    return null;
  }

  return (
    <div className="fixed inset-0 bg-black bg-opacity-40 flex items-center justify-center z-50">
      <div className="bg-white p-6 rounded-lg shadow-lg max-w-md w-full">
        <div className="mb-4 font-bold text-lg">{title}</div>
        {/* Hidden dummy forms to confuse password managers */}
        <form style={{ display: 'none' }} autoComplete="off">
          <input type="text" name="fake-username" autoComplete="username" value="dummy" readOnly />
          <input
            type="password"
            name="fake-password"
            autoComplete="current-password"
            value="dummy"
            readOnly
          />
        </form>
        <div style={{ display: 'none' }}>
          <input type="text" name="decoy1" />
          <input type="password" name="decoy2" />
        </div>
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
              onKeyPress={handleKeyPress}
              placeholder="Enter your Hugging Face token"
              disabled={isLoading}
              autoFocus
              autoComplete="off"
              data-lpignore="true"
              name="hf-token"
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
            onClick={handleSubmit}
            disabled={isLoading}
          >
            {isLoading ? 'Loading...' : 'Submit'}
          </button>
          <button
            className="flex-1 px-4 py-2 bg-gray-400 text-white rounded hover:bg-gray-500 transition-colors"
            onClick={handleClose}
            disabled={isLoading}
          >
            Cancel
          </button>
        </div>
      </div>
    </div>
  );
};

export default TokenInputPopup;
