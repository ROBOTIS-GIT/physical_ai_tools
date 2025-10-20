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

import React, { useState, useCallback, useEffect } from 'react';
import clsx from 'clsx';
import { useSelector } from 'react-redux';
import toast from 'react-hot-toast';
import { MdOutlineFileDownload, MdClose } from 'react-icons/md';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import TokenInputPopup from './TokenInputPopup';
import HFStatus from '../constants/HFStatus';
import { DEFAULT_PATHS } from '../constants/paths';

// HuggingFace repository name validation
const validateHfRepoName = (repoName) => {
  if (!repoName) return { isValid: false, message: '' };

  // Check length (max 96 characters)
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
  if (!allowedPattern.test(repoName)) {
    return {
      isValid: false,
      message: 'Repository name can only contain letters, numbers, "-", "_", and "."',
    };
  }

  return { isValid: true, message: '' };
};

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
    'focus:ring-blue-500',
    'focus:border-transparent'
  ),
  selectUserID: clsx(
    'text-md',
    'w-full',
    'max-w-120',
    'min-w-60',
    'h-8',
    'px-2',
    'border',
    'border-gray-300',
    'rounded-md',
    'focus:outline-none',
    'focus:ring-2',
    'focus:ring-blue-500',
    'focus:border-transparent'
  ),
  loadUserButton: clsx('px-3', 'py-1', 'text-md', 'font-medium', 'rounded-xl', 'transition-colors'),
  cancelButton: clsx('px-6', 'py-2', 'text-sm', 'font-medium', 'rounded-lg', 'transition-colors'),
};

const PolicyDownloadModal = ({ isOpen, onClose, onDownloadComplete }) => {
  const hfStatus = useSelector((state) => state.editDataset.hfStatus);
  // const downloadStatus = useSelector((state) => state.editDataset.downloadStatus);

  const { controlHfServer, registerHFUser, getRegisteredHFUser } = useRosServiceCaller();

  // Local states
  const [hfRepoId, setHfRepoId] = useState('');
  const [userId, setUserId] = useState('');
  const [isDownloading, setIsDownloading] = useState(false);
  const [finalStatus, setFinalStatus] = useState(null); // Store final SUCCESS/FAILED status
  const [userIdList, setUserIdList] = useState([]);
  const [showTokenPopup, setShowTokenPopup] = useState(false);
  const [isLoading, setIsLoading] = useState(false);

  // Validation states
  const [repoValidation, setRepoValidation] = useState({ isValid: true, message: '' });

  // Computed values
  const isHfStatusReady =
    hfStatus === HFStatus.IDLE || hfStatus === HFStatus.SUCCESS || hfStatus === HFStatus.FAILED;

  const downloadButtonEnabled =
    !isDownloading &&
    hfRepoId?.trim() &&
    userId?.trim() &&
    repoValidation.isValid &&
    isHfStatusReady;

  const canCloseModal = !isDownloading;

  // Button variants helper function
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

  // Input handlers with validation
  const handleRepoIdChange = (value) => {
    let repo_id = '';

    if (value.includes('/')) {
      const head = value.split('/')[0];
      const tail = value.split('/')[1];

      if (userIdList.includes(head)) {
        setUserId(head);
        repo_id = tail;
      } else {
        // If the head is not in userIdList, treat the whole value as repo_id
        repo_id = value;
      }
    } else {
      repo_id = value;
    }

    setHfRepoId(repo_id);
    const validation = validateHfRepoName(repo_id.trim());
    setRepoValidation(validation);
  };

  // Download operation
  const handleDownloadPolicy = async () => {
    if (!hfRepoId || hfRepoId.trim() === '') {
      toast.error('Please enter a Repository ID first');
      return;
    }

    if (!userId || userId.trim() === '') {
      toast.error('Please select a User ID first');
      return;
    }

    // Additional validation check
    const validation = validateHfRepoName(hfRepoId.trim());
    if (!validation.isValid) {
      toast.error(`Invalid repository name: ${validation.message}`);
      return;
    }

    setIsDownloading(true);
    try {
      const repoId = userId + '/' + hfRepoId.trim();
      const result = await controlHfServer('download', repoId, 'model');
      console.log('Download policy result:', result);

      toast.success(`Policy download started successfully for ${repoId}!`);

      // Call the completion callback if provided
      if (onDownloadComplete) {
        onDownloadComplete(repoId);
      }
    } catch (error) {
      console.error('Error downloading policy:', error);
      toast.error(`Failed to download policy: ${error.message}`);
    } finally {
      setIsDownloading(false);
    }
  };

  const handleCancelDownload = async () => {
    try {
      const result = await controlHfServer('cancel', hfRepoId, 'model');
      console.log('Cancel download result:', result);
      toast.success(`Cancelling... (${hfRepoId})`);
    } catch (error) {
      console.error('Error canceling download:', error);
      toast.error(`Failed to cancel download: ${error.message}`);
    }
  };

  // Handle finish button click
  const handleFinish = () => {
    onClose();
  };

  // Auto-load User ID list on component mount
  useEffect(() => {
    if (isOpen) {
      handleLoadUserId();
    }
  }, [isOpen, handleLoadUserId]);

  // track hf status update
  useEffect(() => {
    if (hfStatus === HFStatus.DOWNLOADING) {
      setIsDownloading(true);
    } else {
      setIsDownloading(false);
    }
  }, [hfStatus]);

  // Track HF status updates
  useEffect(() => {
    if (hfStatus === HFStatus.DOWNLOADING) {
      setIsDownloading(true);
      setFinalStatus(null);
    } else if (hfStatus === HFStatus.SUCCESS || hfStatus === HFStatus.FAILED) {
      // Download completed (success or failed) - only process once
      setIsDownloading(false);
      setFinalStatus(hfStatus); // Store the final status
    }
  }, [hfStatus, isDownloading]);

  // Reset form when modal closes
  useEffect(() => {
    if (!isOpen) {
      setHfRepoId('');
      setUserId('');
      setRepoValidation({ isValid: true, message: '' });
      setIsDownloading(false);
      setFinalStatus(null);
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
                <MdOutlineFileDownload className="text-2xl text-blue-600" />
                <h2 className="text-xl font-semibold text-gray-900">
                  Download Policy from Hugging Face
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
              {/* Download Policy Section Header */}
              <div className="w-full flex flex-col items-start justify-start gap-2 bg-gray-50 border border-gray-200 p-3 mb-2 rounded-md">
                <div className="w-full flex items-center rounded-md font-medium gap-2">
                  <MdOutlineFileDownload className="text-lg text-blue-600" />
                  Download Policy
                </div>
                <div className="text-sm text-gray-600">
                  <div className="mb-1">
                    Downloads policy model from Hugging Face hub to local directory
                  </div>
                </div>
              </div>

              <div className="space-y-3">
                {/* User ID Selection */}
                <div className="bg-white p-4 rounded-md flex flex-col items-start justify-center gap-4 shadow-md">
                  <div className="w-full flex items-center justify-start">
                    <span className="text-lg font-bold">User ID Configuration</span>
                  </div>
                  <div
                    className={clsx('w-full flex flex-row gap-3', {
                      'opacity-50': isDownloading,
                    })}
                  >
                    <select
                      className={STYLES.selectUserID}
                      value={userId || ''}
                      onChange={(e) => setUserId(e.target.value)}
                      disabled={isDownloading}
                    >
                      <option value="">Select User ID</option>
                      {userIdList.map((id) => (
                        <option key={id} value={id}>
                          {id}
                        </option>
                      ))}
                    </select>
                    <div className="flex gap-2">
                      <button
                        className={clsx(
                          STYLES.loadUserButton,
                          getButtonVariant('blue', true, isLoading)
                        )}
                        onClick={() => {
                          if (!isLoading) {
                            handleLoadUserId();
                          }
                        }}
                        disabled={isLoading}
                      >
                        {isLoading ? 'Loading...' : 'Load'}
                      </button>
                      <button
                        className={clsx(
                          STYLES.loadUserButton,
                          getButtonVariant('green', true, isLoading)
                        )}
                        onClick={() => {
                          if (!isLoading) {
                            setShowTokenPopup(true);
                          }
                        }}
                        disabled={isLoading}
                      >
                        Change
                      </button>
                    </div>
                  </div>
                </div>

                {/* Repository ID Input */}
                <div className="w-full bg-white p-4 rounded-md flex flex-col items-start justify-center gap-2 shadow-md">
                  <div className="w-full flex flex-col gap-3">
                    <div className="w-full flex flex-col gap-2">
                      <span className="text-lg font-bold">Repository ID</span>
                      <div className="relative">
                        <div
                          className={clsx(
                            'flex items-center border rounded-md overflow-hidden bg-white focus-within:ring-2',
                            {
                              'border-gray-300 focus-within:ring-blue-500 focus-within:border-transparent':
                                repoValidation.isValid || !hfRepoId,
                              'border-red-300 focus-within:ring-red-500 focus-within:border-transparent':
                                !repoValidation.isValid && hfRepoId,
                            }
                          )}
                        >
                          <div className="px-3 py-2 bg-gray-50 border-r border-gray-300 text-gray-700 font-medium flex items-center">
                            <span className="text-sm">{userId || 'username'}</span>
                            <span className="mx-1 text-gray-400">/</span>
                          </div>
                          <input
                            className={clsx(
                              'flex-1 px-3 py-2 text-sm bg-transparent border-none outline-none',
                              {
                                'bg-gray-100 cursor-not-allowed text-gray-500': isDownloading,
                                'text-gray-900': !isDownloading,
                              }
                            )}
                            type="text"
                            placeholder="Enter repository id or username/repo"
                            value={hfRepoId || ''}
                            onChange={(e) => handleRepoIdChange(e.target.value)}
                            disabled={isDownloading}
                          />
                        </div>
                        <div className="mt-1 text-xs">
                          <div className="text-gray-500">
                            Full repository path:{' '}
                            <span className="font-mono text-blue-600">
                              {userId || ''}/{hfRepoId || ''}
                            </span>
                          </div>
                          {!repoValidation.isValid && hfRepoId && (
                            <div className="text-red-500 mt-1">⚠️ {repoValidation.message}</div>
                          )}
                        </div>
                      </div>
                    </div>

                    {/* Info message: Policy save path with folder icon */}
                    <div className="w-full flex flex-row items-center mt-1">
                      <span className="text-xs text-gray-600 flex items-center gap-1">
                        <MdOutlineFileDownload className="inline-block w-4 h-4 text-blue-700 mr-1" />
                        The policy will be saved in{' '}
                        <span className="font-mono text-blue-700">
                          {DEFAULT_PATHS.POLICY_MODEL_PATH}
                        </span>
                      </span>
                    </div>

                    {/* Download Button and Status */}
                    <div className="w-full flex flex-row items-center justify-start gap-3 mt-2">
                      <button
                        className={clsx(
                          'px-6',
                          'py-2',
                          'text-sm',
                          'font-medium',
                          'rounded-lg',
                          'transition-colors',
                          {
                            'bg-blue-500 text-white hover:bg-blue-600': downloadButtonEnabled,
                            'bg-gray-300 text-gray-500 cursor-not-allowed': !downloadButtonEnabled,
                          }
                        )}
                        onClick={handleDownloadPolicy}
                        disabled={!downloadButtonEnabled}
                      >
                        <div className="flex items-center justify-center gap-2">
                          <MdOutlineFileDownload className="w-6 h-6" />
                          Download
                        </div>
                      </button>

                      {/* Cancel Button */}
                      <button
                        className={clsx(STYLES.cancelButton, {
                          'bg-red-500 text-white hover:bg-red-600': isDownloading,
                          'bg-gray-300 text-gray-500 cursor-not-allowed': !isDownloading,
                        })}
                        onClick={handleCancelDownload}
                        disabled={!isDownloading}
                      >
                        Cancel
                      </button>

                      {/* Status */}
                      <div className="flex flex-row items-center justify-start gap-2">
                        <span className="text-sm text-gray-500">
                          {isDownloading && '⏳ Downloading...'}
                          {!isDownloading && hfStatus}{' '}
                        </span>
                        {/* Spinner for model downloads - right next to status text */}
                        {isDownloading && (
                          <div className="animate-spin rounded-full h-4 w-4 border-b-2 border-blue-600"></div>
                        )}
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </div>

            {/* Modal Footer */}
            <div className="flex items-center justify-end gap-3 px-6 py-4 border-t border-gray-200">
              <button
                onClick={handleFinish}
                className={clsx('px-4 py-2 text-sm font-medium rounded-md transition-colors', {
                  'bg-green-500 text-white hover:bg-green-600': finalStatus === HFStatus.SUCCESS,
                  'bg-gray-500 text-white hover:bg-gray-600 cursor-not-allowed':
                    finalStatus === HFStatus.FAILED,
                  'bg-gray-300 text-gray-500 cursor-not-allowed': !finalStatus,
                })}
                disabled={!finalStatus}
              >
                Finish
              </button>
              <button
                onClick={onClose}
                className="px-4 py-2 text-sm font-medium text-gray-700 bg-gray-100 border border-gray-300 rounded-md hover:bg-gray-200 transition-colors"
                disabled={!canCloseModal}
              >
                {isDownloading ? 'Downloading...' : 'Cancel'}
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

export default PolicyDownloadModal;
