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
// Author: Physical AI Team

import React, { useState, useCallback, useEffect } from 'react';
import clsx from 'clsx';
import { useSelector } from 'react-redux';
import toast from 'react-hot-toast';
import {
  MdOutlineFileDownload,
  MdClose,
  MdDataset,
  MdKey,
  MdExpandMore,
  MdExpandLess,
} from 'react-icons/md';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import TokenInputPopup from './TokenInputPopup';
import HFStatus from '../constants/HFStatus';

// HuggingFace repository ID validation (includes username/repo format)
const validateHfRepoId = (repoId) => {
  if (!repoId) return { isValid: false, message: '' };

  const trimmed = repoId.trim();

  // Check for username/repo format
  if (!trimmed.includes('/')) {
    return {
      isValid: false,
      message: 'Format: username/dataset-name (e.g., lerobot/pusht)',
    };
  }

  const parts = trimmed.split('/');
  if (parts.length !== 2 || !parts[0] || !parts[1]) {
    return {
      isValid: false,
      message: 'Format: username/dataset-name (e.g., lerobot/pusht)',
    };
  }

  const [username, repoName] = parts;

  // Check length (max 96 characters for repo name)
  if (repoName.length > 96) {
    return {
      isValid: false,
      message: 'Repository name must be 96 characters or less',
    };
  }

  // Check if starts or ends with '-' or '.'
  if (
    repoName.startsWith('-') ||
    repoName.startsWith('.') ||
    repoName.endsWith('-') ||
    repoName.endsWith('.')
  ) {
    return {
      isValid: false,
      message: 'Repository name cannot start or end with "-" or "."',
    };
  }

  // Check for forbidden patterns '--' and '..'
  if (repoName.includes('--') || repoName.includes('..')) {
    return {
      isValid: false,
      message: 'Repository name cannot contain "--" or ".."',
    };
  }

  // Check for allowed characters only (alphanumeric, '-', '_', '.')
  const allowedPattern = /^[a-zA-Z0-9._-]+$/;
  if (!allowedPattern.test(repoName) || !allowedPattern.test(username)) {
    return {
      isValid: false,
      message: 'Names can only contain letters, numbers, "-", "_", and "."',
    };
  }

  return { isValid: true, message: '' };
};

// Dataset save path
const DATASET_SAVE_PATH = '/root/.cache/huggingface/lerobot';

// Style Classes
const STYLES = {
  textInput: clsx(
    'text-sm',
    'w-full',
    'h-10',
    'p-2',
    'border',
    'border-gray-300',
    'rounded-md',
    'focus:outline-none',
    'focus:ring-2',
    'focus:ring-green-500',
    'focus:border-transparent'
  ),
  cancelButton: clsx('px-6', 'py-2', 'text-sm', 'font-medium', 'rounded-lg', 'transition-colors'),
};

const DatasetDownloadModal = ({ isOpen, onClose, onDownloadComplete }) => {
  const hfStatus = useSelector((state) => state.editDataset.hfStatus);

  const { controlHfServer, registerHFUser } = useRosServiceCaller();

  // Local states
  const [repoId, setRepoId] = useState('');
  const [isDownloading, setIsDownloading] = useState(false);
  const [finalStatus, setFinalStatus] = useState(null);
  const [showTokenSection, setShowTokenSection] = useState(false);
  const [showTokenPopup, setShowTokenPopup] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [tokenRegistered, setTokenRegistered] = useState(false);

  // Validation states
  const [repoValidation, setRepoValidation] = useState({ isValid: false, message: '' });

  // Computed values
  const isHfStatusReady =
    hfStatus === HFStatus.IDLE || hfStatus === HFStatus.SUCCESS || hfStatus === HFStatus.FAILED;

  const downloadButtonEnabled =
    !isDownloading && repoId?.trim() && repoValidation.isValid && isHfStatusReady;

  const canCloseModal = !isDownloading;

  // Token related handlers
  const handleTokenSubmit = async (token) => {
    if (!token || !token.trim()) {
      toast.error('Please enter a token');
      return;
    }

    setIsLoading(true);
    try {
      const result = await registerHFUser(token);
      console.log('registerHFUser result:', result);

      if (result && result.success) {
        setTokenRegistered(true);
        setShowTokenPopup(false);
        toast.success('Token registered successfully!');
      } else {
        toast.error('Failed to register token: ' + (result?.message || 'Unknown error'));
      }
    } catch (error) {
      console.error('Error registering HF token:', error);
      toast.error(`Failed to register token: ${error.message}`);
    } finally {
      setIsLoading(false);
    }
  };

  // Input handlers with validation
  const handleRepoIdChange = (value) => {
    setRepoId(value);
    const validation = validateHfRepoId(value.trim());
    setRepoValidation(validation);
  };

  // Download operation
  const handleDownloadDataset = async () => {
    if (!repoId || repoId.trim() === '') {
      toast.error('Please enter a Repository ID');
      return;
    }

    // Additional validation check
    const validation = validateHfRepoId(repoId.trim());
    if (!validation.isValid) {
      toast.error(`Invalid repository ID: ${validation.message}`);
      return;
    }

    setIsDownloading(true);
    try {
      const trimmedRepoId = repoId.trim();
      // Use 'dataset' as repo_type for dataset download
      const result = await controlHfServer('download', trimmedRepoId, 'dataset');
      console.log('Download dataset result:', result);

      toast.success(`Dataset download started for ${trimmedRepoId}!`);

      // Call the completion callback if provided with the full path
      if (onDownloadComplete) {
        const downloadedPath = `${DATASET_SAVE_PATH}/${trimmedRepoId}`;
        onDownloadComplete(downloadedPath);
      }
    } catch (error) {
      console.error('Error downloading dataset:', error);
      toast.error(`Failed to download dataset: ${error.message}`);
    } finally {
      setIsDownloading(false);
    }
  };

  const handleCancelDownload = async () => {
    try {
      const result = await controlHfServer('cancel', repoId, 'dataset');
      console.log('Cancel download result:', result);
      toast.success(`Cancelling download...`);
    } catch (error) {
      console.error('Error canceling download:', error);
      toast.error(`Failed to cancel download: ${error.message}`);
    }
  };

  // Handle finish button click
  const handleFinish = () => {
    if (onDownloadComplete && finalStatus === HFStatus.SUCCESS) {
      const trimmedRepoId = repoId.trim();
      const downloadedPath = `${DATASET_SAVE_PATH}/${trimmedRepoId}`;
      onDownloadComplete(downloadedPath);
    }
    onClose();
  };

  // Track HF status updates
  useEffect(() => {
    if (hfStatus === HFStatus.DOWNLOADING) {
      setIsDownloading(true);
      setFinalStatus(null);
    } else if (hfStatus === HFStatus.SUCCESS || hfStatus === HFStatus.FAILED) {
      setIsDownloading(false);
      setFinalStatus(hfStatus);
    }
  }, [hfStatus]);

  // Reset form when modal closes
  useEffect(() => {
    if (!isOpen) {
      setRepoId('');
      setRepoValidation({ isValid: false, message: '' });
      setIsDownloading(false);
      setFinalStatus(null);
      setShowTokenSection(false);
    }
  }, [isOpen]);

  if (!isOpen) return null;

  return (
    <>
      {/* Modal Overlay */}
      <div className="fixed inset-0 z-50 overflow-y-auto">
        <div className="fixed inset-0 bg-black bg-opacity-50 transition-opacity" />

        {/* Modal Container */}
        <div className="flex min-h-full items-center justify-center p-4">
          <div className="relative bg-white rounded-lg shadow-xl max-w-2xl w-full max-h-[90vh] flex flex-col">
            {/* Modal Header */}
            <div className="flex items-center justify-between px-6 py-4 border-b border-gray-200">
              <div className="flex items-center gap-3">
                <MdDataset className="text-2xl text-green-600" />
                <h2 className="text-xl font-semibold text-gray-900">
                  Download Dataset from Hugging Face
                </h2>
              </div>
              <button
                onClick={onClose}
                className="p-2 text-gray-400 hover:text-gray-600 hover:bg-gray-100 rounded-lg transition-colors"
                disabled={!canCloseModal}
              >
                <MdClose className="w-6 h-6" />
              </button>
            </div>

            {/* Modal Content */}
            <div className="flex-1 p-6 overflow-y-auto">
              {/* Download Dataset Section */}
              <div className="space-y-4">
                {/* Repository ID Input */}
                <div className="w-full bg-white p-4 rounded-md flex flex-col gap-3 border border-gray-200">
                  <div className="flex flex-col gap-2">
                    <span className="text-lg font-bold text-gray-800">Dataset Repository ID</span>
                    <p className="text-sm text-gray-500">
                      Enter the full repository path (e.g., lerobot/pusht, ROBOTIS/test1)
                    </p>
                  </div>

                  <div className="relative">
                    <input
                      className={clsx(STYLES.textInput, {
                        'border-red-300 focus:ring-red-500': !repoValidation.isValid && repoId,
                        'border-green-300 focus:ring-green-500': repoValidation.isValid,
                        'bg-gray-100 cursor-not-allowed': isDownloading,
                      })}
                      type="text"
                      placeholder="username/dataset-name"
                      value={repoId}
                      onChange={(e) => handleRepoIdChange(e.target.value)}
                      disabled={isDownloading}
                    />
                    {repoId && (
                      <div className="mt-1 text-xs">
                        {repoValidation.isValid ? (
                          <span className="text-green-600">✓ Valid repository ID</span>
                        ) : (
                          <span className="text-red-500">{repoValidation.message}</span>
                        )}
                      </div>
                    )}
                  </div>

                  {/* Save path info */}
                  {repoValidation.isValid && (
                    <div className="flex items-center gap-2 text-xs text-gray-600 bg-gray-50 p-2 rounded">
                      <MdOutlineFileDownload className="text-green-600" />
                      <span>
                        Will be saved to:{' '}
                        <span className="font-mono text-green-700">
                          {DATASET_SAVE_PATH}/{repoId.trim()}
                        </span>
                      </span>
                    </div>
                  )}

                  {/* Download Button and Status */}
                  <div className="flex items-center gap-3 mt-2">
                    <button
                      className={clsx(
                        'px-6',
                        'py-2',
                        'text-sm',
                        'font-medium',
                        'rounded-lg',
                        'transition-colors',
                        'flex',
                        'items-center',
                        'gap-2',
                        {
                          'bg-green-500 text-white hover:bg-green-600': downloadButtonEnabled,
                          'bg-gray-300 text-gray-500 cursor-not-allowed': !downloadButtonEnabled,
                        }
                      )}
                      onClick={handleDownloadDataset}
                      disabled={!downloadButtonEnabled}
                    >
                      <MdOutlineFileDownload className="w-5 h-5" />
                      Download
                    </button>

                    {/* Cancel Button */}
                    {isDownloading && (
                      <button
                        className={clsx(
                          STYLES.cancelButton,
                          'bg-red-500 text-white hover:bg-red-600'
                        )}
                        onClick={handleCancelDownload}
                      >
                        Cancel
                      </button>
                    )}

                    {/* Status */}
                    {isDownloading && (
                      <div className="flex items-center gap-2">
                        <div className="animate-spin rounded-full h-4 w-4 border-b-2 border-green-600"></div>
                        <span className="text-sm text-gray-500">Downloading...</span>
                      </div>
                    )}
                  </div>
                </div>

                {/* Token Registration Section (Collapsible) */}
                <div className="border border-gray-200 rounded-md overflow-hidden">
                  <button
                    className="w-full px-4 py-3 flex items-center justify-between bg-gray-50 hover:bg-gray-100 transition-colors"
                    onClick={() => setShowTokenSection(!showTokenSection)}
                  >
                    <div className="flex items-center gap-2">
                      <MdKey className="text-gray-500" />
                      <span className="text-sm font-medium text-gray-700">
                        Token Registration (Optional)
                      </span>
                      {tokenRegistered && (
                        <span className="text-xs bg-green-100 text-green-700 px-2 py-0.5 rounded">
                          Registered
                        </span>
                      )}
                    </div>
                    {showTokenSection ? (
                      <MdExpandLess className="text-gray-500" />
                    ) : (
                      <MdExpandMore className="text-gray-500" />
                    )}
                  </button>

                  {showTokenSection && (
                    <div className="px-4 py-3 bg-white border-t border-gray-200">
                      <p className="text-xs text-gray-500 mb-3">
                        Required only for private datasets. Public datasets (like lerobot/pusht) don't
                        need a token.
                      </p>
                      <button
                        className={clsx(
                          'px-4',
                          'py-2',
                          'text-sm',
                          'font-medium',
                          'rounded-lg',
                          'transition-colors',
                          'flex',
                          'items-center',
                          'gap-2',
                          {
                            'bg-blue-500 text-white hover:bg-blue-600': !isLoading,
                            'bg-gray-300 text-gray-500 cursor-not-allowed': isLoading,
                          }
                        )}
                        onClick={() => setShowTokenPopup(true)}
                        disabled={isLoading}
                      >
                        <MdKey className="w-4 h-4" />
                        {tokenRegistered ? 'Update Token' : 'Register Token'}
                      </button>
                    </div>
                  )}
                </div>
              </div>
            </div>

            {/* Modal Footer */}
            <div className="flex items-center justify-end gap-3 px-6 py-4 border-t border-gray-200">
              {finalStatus === HFStatus.SUCCESS && (
                <button
                  onClick={handleFinish}
                  className="px-4 py-2 text-sm font-medium rounded-md transition-colors bg-green-500 text-white hover:bg-green-600"
                >
                  Use This Dataset
                </button>
              )}
              <button
                onClick={onClose}
                className="px-4 py-2 text-sm font-medium text-gray-700 bg-gray-100 border border-gray-300 rounded-md hover:bg-gray-200 transition-colors"
                disabled={!canCloseModal}
              >
                {isDownloading ? 'Downloading...' : 'Close'}
              </button>
            </div>
          </div>
        </div>
      </div>

      {/* Token Input Popup */}
      <TokenInputPopup
        isOpen={showTokenPopup}
        onClose={() => setShowTokenPopup(false)}
        onSubmit={handleTokenSubmit}
        isLoading={isLoading}
      />
    </>
  );
};

export default DatasetDownloadModal;
