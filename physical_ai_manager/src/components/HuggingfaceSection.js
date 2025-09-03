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
import { useSelector, useDispatch } from 'react-redux';
import toast from 'react-hot-toast';
import { MdFolderOpen, MdOutlineFileUpload, MdOutlineFileDownload } from 'react-icons/md';
import { setUserId } from '../features/editDataset/editDatasetSlice';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import FileBrowserModal from './FileBrowserModal';
import TokenInputPopup from './TokenInputPopup';
import { DEFAULT_PATHS, TARGET_FOLDERS } from '../constants/paths';

// Style Classes
const STYLES = {
  textInput: clsx(
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
    'focus:border-transparent'
  ),
  selectUserID: clsx(
    'text-md',
    'w-full',
    'max-w-80',
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
};

const HuggingfaceSection = ({ isEditable = true }) => {
  const dispatch = useDispatch();
  const { userId } = useSelector((state) => state.editDataset);
  const { browseFile, controlHfServer, registerHFUser, getRegisteredHFUser } =
    useRosServiceCaller();

  // Local states
  const [hfRepoIdUpload, setHfRepoIdUpload] = useState('');
  const [hfRepoIdDownload, setHfRepoIdDownload] = useState('');
  const [hfLocalDirUpload, setHfLocalDirUpload] = useState('');
  const [hfLocalDirDownload, setHfLocalDirDownload] = useState('');
  const [isUploading, setIsUploading] = useState(false);
  const [isDownloading, setIsDownloading] = useState(false);
  const [showHfLocalDirBrowserModal, setShowHfLocalDirBrowserModal] = useState(false);
  const [showHfLocalDirDownloadBrowserModal, setShowHfLocalDirDownloadBrowserModal] =
    useState(false);
  const [userIdList, setUserIdList] = useState([]);
  const [showTokenPopup, setShowTokenPopup] = useState(false);
  const [isLoading, setIsLoading] = useState(false);

  // Computed values
  const uploadButtonEnabled =
    !isUploading && isEditable && hfRepoIdUpload?.trim() && hfLocalDirUpload?.trim();
  const downloadButtonEnabled =
    !isDownloading && isEditable && hfRepoIdDownload?.trim() && hfLocalDirDownload?.trim();

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
          toast.error('Failed to get user ID list: ' + result.message);
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

  // File browser handlers
  const handleHfLocalDirSelect = useCallback((item) => {
    setHfLocalDirUpload(item.full_path);
    setShowHfLocalDirBrowserModal(false);
  }, []);

  const handleHfLocalDirDownloadSelect = useCallback((item) => {
    setHfLocalDirDownload(item.full_path);
    setShowHfLocalDirDownloadBrowserModal(false);
  }, []);

  // Operations
  const operations = {
    uploadDataset: async () => {
      if (!hfRepoIdUpload || hfRepoIdUpload.trim() === '') {
        toast.error('Please enter a Repo ID first');
        return;
      }

      if (!hfLocalDirUpload || hfLocalDirUpload.trim() === '') {
        toast.error('Please select a Local Directory first');
        return;
      }

      setIsUploading(true);
      try {
        const repoId = hfRepoIdUpload.trim();
        const localDir = hfLocalDirUpload.trim();
        const result = await controlHfServer('upload', repoId, 'dataset', localDir);
        console.log('Upload dataset result:', result);
        toast.success(`Dataset upload started successfully for ${repoId}!`);
      } catch (error) {
        console.error('Error uploading dataset:', error);
        toast.error(`Failed to upload dataset: ${error.message}`);
      } finally {
        setIsUploading(false);
      }
    },

    downloadDataset: async () => {
      if (!hfRepoIdDownload || hfRepoIdDownload.trim() === '') {
        toast.error('Please enter a Repo ID first');
        return;
      }

      setIsDownloading(true);
      try {
        const repoId = hfRepoIdDownload.trim();
        // Update the local dir text box with the local cache path
        const localPath = `/root/.cache/huggingface/lerobot/${repoId}`;
        setHfLocalDirDownload(localPath);
        const result = await controlHfServer('download', repoId, 'dataset');
        console.log('Download dataset result:', result);

        toast.success(`Dataset download started successfully for ${repoId}!`);
      } catch (error) {
        console.error('Error downloading dataset:', error);
        toast.error(`Failed to download dataset: ${error.message}`);
      } finally {
        setIsDownloading(false);
      }
    },
  };

  // Auto-load User ID list on component mount
  useEffect(() => {
    handleLoadUserId();
  }, [handleLoadUserId]);

  return (
    <div className="w-full flex flex-col items-center justify-start bg-gray-100 p-10 gap-8 rounded-xl">
      <div className="w-full flex items-center justify-start">
        <span className="text-2xl font-bold mb-4">Huggingface Upload & Download</span>
      </div>

      {/* User ID Selection */}
      <div className="w-full bg-white p-5 rounded-md flex flex-col items-start justify-center gap-4 shadow-md">
        <div className="w-full flex items-center justify-start">
          <span className="text-lg font-bold">User ID Configuration</span>
        </div>
        <div className="w-full flex flex-row gap-3">
          <select
            className={STYLES.selectUserID}
            value={userId || ''}
            onChange={(e) => dispatch(setUserId(e.target.value))}
          >
            <option value="">Select User ID</option>
            {userIdList.map((userId) => (
              <option key={userId} value={userId}>
                {userId}
              </option>
            ))}
          </select>
          <div className="flex gap-2">
            <button
              className={clsx(
                STYLES.loadUserButton,
                getButtonVariant('blue', isEditable, isLoading)
              )}
              onClick={() => {
                if (isEditable && !isLoading) {
                  handleLoadUserId();
                }
              }}
              disabled={!isEditable || isLoading}
            >
              {isLoading ? 'Loading...' : 'Load'}
            </button>
            <button
              className={clsx(
                STYLES.loadUserButton,
                getButtonVariant('green', isEditable, isLoading)
              )}
              onClick={() => {
                if (isEditable && !isLoading) {
                  setShowTokenPopup(true);
                }
              }}
              disabled={!isEditable || isLoading}
            >
              Change
            </button>
          </div>
        </div>
      </div>

      <div className="w-full flex gap-5">
        {/* Upload */}
        <div className="w-full bg-white p-5 rounded-md flex flex-col items-start justify-center gap-4 shadow-md">
          <div className="w-full flex items-center justify-start">
            <span className="text-lg font-bold">Upload Dataset</span>
          </div>
          <div className="w-full flex flex-col gap-3">
            {/* Local Directory Input */}
            <div className="w-full flex flex-col gap-2">
              <span className="text-lg font-bold">Local Directory</span>
              <div className="w-full flex flex-row items-center justify-start gap-2">
                <input
                  className={clsx(STYLES.textInput, 'flex-1', {
                    'bg-gray-100 cursor-not-allowed': !isEditable,
                    'bg-white': isEditable,
                  })}
                  type="text"
                  placeholder="Enter local directory path or browse"
                  value={hfLocalDirUpload || ''}
                  onChange={(e) => setHfLocalDirUpload(e.target.value)}
                  disabled={!isEditable}
                />
                <button
                  type="button"
                  onClick={() => setShowHfLocalDirBrowserModal(true)}
                  className="flex items-center justify-center w-8 h-8 text-blue-500 bg-gray-200 rounded-md hover:text-blue-700"
                  aria-label="Browse files for local directory"
                >
                  <MdFolderOpen className="w-6 h-6" />
                </button>
              </div>
            </div>

            {/* Repo ID Input */}
            <div className="w-full flex flex-col gap-2">
              <span className="text-lg font-bold">Repository ID</span>
              <div className="relative">
                <div className="flex items-center border border-gray-300 rounded-md overflow-hidden bg-white focus-within:ring-2 focus-within:ring-blue-500 focus-within:border-transparent">
                  <div className="px-3 py-2 bg-gray-50 border-r border-gray-300 text-gray-700 font-medium flex items-center">
                    <span className="text-sm">{userId || 'username'}</span>
                    <span className="mx-1 text-gray-400">/</span>
                  </div>
                  <input
                    className={clsx(
                      'flex-1 px-3 py-2 text-sm bg-transparent border-none outline-none',
                      {
                        'bg-gray-100 cursor-not-allowed text-gray-500': !isEditable,
                        'text-gray-900': isEditable,
                      }
                    )}
                    type="text"
                    placeholder="Enter repository id"
                    value={hfRepoIdUpload || ''}
                    onChange={(e) => setHfRepoIdUpload(e.target.value)}
                    disabled={!isEditable}
                  />
                </div>
                <div className="mt-1 text-xs text-gray-500">
                  Full repository path:{' '}
                  <span className="font-mono text-blue-600">
                    {userId || ''}/{hfRepoIdUpload || ''}
                  </span>
                </div>
              </div>
            </div>

            {/* Action Buttons */}
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
                    'bg-green-500 text-white hover:bg-green-600': uploadButtonEnabled,
                    'bg-gray-300 text-gray-500 cursor-not-allowed': !uploadButtonEnabled,
                  }
                )}
                onClick={operations.uploadDataset}
                disabled={!uploadButtonEnabled}
              >
                <div className="flex items-center justify-center gap-2">
                  <MdOutlineFileUpload className="w-6 h-6" />
                  {isUploading ? 'Uploading...' : 'Upload'}
                </div>
              </button>
            </div>
          </div>

          <div className="text-sm text-gray-600 mt-2">
            <div className="mb-1">
              • <strong>Upload:</strong> Uploads dataset from local directory to HuggingFace Hub
            </div>
          </div>
        </div>

        {/* Download */}
        <div className="w-full bg-white p-5 rounded-md flex flex-col items-start justify-center gap-4 shadow-md">
          <div className="w-full flex items-center justify-start">
            <span className="text-lg font-bold">Download Dataset</span>
          </div>
          <div className="w-full flex flex-col gap-3">
            {/* Repo ID Input */}
            <div className="w-full flex flex-col gap-2">
              <span className="text-lg font-bold">Repository ID</span>
              <div className="relative">
                <div className="flex items-center border border-gray-300 rounded-md overflow-hidden bg-white focus-within:ring-2 focus-within:ring-blue-500 focus-within:border-transparent">
                  <div className="px-3 py-2 bg-gray-50 border-r border-gray-300 text-gray-700 font-medium flex items-center">
                    <span className="text-sm">{userId || 'username'}</span>
                    <span className="mx-1 text-gray-400">/</span>
                  </div>
                  <input
                    className={clsx(
                      'flex-1 px-3 py-2 text-sm bg-transparent border-none outline-none',
                      {
                        'bg-gray-100 cursor-not-allowed text-gray-500': !isEditable,
                        'text-gray-900': isEditable,
                      }
                    )}
                    type="text"
                    placeholder="Enter repository id"
                    value={hfRepoIdDownload || ''}
                    onChange={(e) => setHfRepoIdDownload(e.target.value)}
                    disabled={!isEditable}
                  />
                </div>
                <div className="mt-1 text-xs text-gray-500">
                  Full repository path:{' '}
                  <span className="font-mono text-blue-600">
                    {userId || ''}/{hfRepoIdDownload || ''}
                  </span>
                </div>
              </div>
            </div>

            {/* Local Directory Input */}
            <div className="w-full flex flex-col gap-2">
              <span className="text-lg font-bold">Local Directory</span>
              <div className="w-full flex flex-row items-center justify-start gap-2">
                <input
                  className={clsx(STYLES.textInput, 'flex-1', {
                    'bg-gray-100 cursor-not-allowed': !isEditable,
                    'bg-white': isEditable,
                  })}
                  type="text"
                  placeholder="Enter local directory path or browse"
                  value={hfLocalDirDownload || ''}
                  onChange={(e) => setHfLocalDirDownload(e.target.value)}
                  disabled={!isEditable}
                />
                <button
                  type="button"
                  onClick={() => setShowHfLocalDirDownloadBrowserModal(true)}
                  className="flex items-center justify-center w-8 h-8 text-blue-500 bg-gray-200 rounded-md hover:text-blue-700"
                  aria-label="Browse files for local directory"
                >
                  <MdFolderOpen className="w-6 h-6" />
                </button>
              </div>
            </div>

            {/* Action Buttons */}
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
                onClick={operations.downloadDataset}
                disabled={!downloadButtonEnabled}
              >
                <div className="flex items-center justify-center gap-2">
                  <MdOutlineFileDownload className="w-6 h-6" />
                  {isDownloading ? 'Downloading...' : 'Download'}
                </div>
              </button>
            </div>
          </div>

          <div className="text-sm text-gray-600 mt-2">
            <div className="mb-1">
              • <strong>Download:</strong> Downloads dataset from HuggingFace Hub to local cache
              directory
            </div>
          </div>
        </div>
      </div>

      {/* File Browser Modals */}
      <FileBrowserModal
        isOpen={showHfLocalDirBrowserModal}
        onClose={() => setShowHfLocalDirBrowserModal(false)}
        onFileSelect={handleHfLocalDirSelect}
        title="Select Local Directory for Upload"
        selectButtonText="Select"
        allowDirectorySelect={false}
        targetFolderName={[
          TARGET_FOLDERS.DATASET_METADATA,
          TARGET_FOLDERS.DATASET_VIDEO,
          TARGET_FOLDERS.DATASET_DATA,
        ]}
        targetFileLabel="Dataset folder found! 🎯"
        initialPath={DEFAULT_PATHS.DATASET_PATH}
        defaultPath={DEFAULT_PATHS.DATASET_PATH}
        homePath=""
      />

      <FileBrowserModal
        isOpen={showHfLocalDirDownloadBrowserModal}
        onClose={() => setShowHfLocalDirDownloadBrowserModal(false)}
        onFileSelect={handleHfLocalDirDownloadSelect}
        title="Select Local Directory for Download"
        selectButtonText="Select"
        allowDirectorySelect={true}
        allowFileSelect={false}
        targetFolderName={[]}
        targetFileLabel="Local directory found! 🎯"
        initialPath={DEFAULT_PATHS.DATASET_PATH}
        defaultPath={DEFAULT_PATHS.DATASET_PATH}
        homePath=""
      />

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

export default HuggingfaceSection;
