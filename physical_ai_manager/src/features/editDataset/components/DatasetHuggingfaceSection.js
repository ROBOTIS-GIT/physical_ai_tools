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
import { setUserId } from '../editDatasetSlice';
import { useRosServiceCaller } from '../../../hooks/useRosServiceCaller';
import FileBrowserModal from '../../../components/FileBrowserModal';
import TokenInputPopup from '../../../components/TokenInputPopup';
import SectionSelector from './SectionSelector';
import { DEFAULT_PATHS, TARGET_FOLDERS } from '../../../constants/paths';
import HFStatus from '../../../constants/HFStatus';

// Constants
const SECTION_NAME = {
  UPLOAD: 'upload',
  DOWNLOAD: 'download',
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
};

// Folder Browse Button Component
const FolderBrowseButton = ({ onClick, disabled = false, ariaLabel }) => {
  return (
    <button
      type="button"
      onClick={onClick}
      className={clsx('flex items-center justify-center w-10 h-10 rounded-md transition-colors', {
        'text-blue-500 bg-gray-200 hover:text-blue-700': !disabled,
        'text-gray-400 bg-gray-100 cursor-not-allowed': disabled,
      })}
      aria-label={ariaLabel}
      disabled={disabled}
    >
      <MdFolderOpen className="w-8 h-8" />
    </button>
  );
};

const HuggingfaceSection = ({ isEditable = true }) => {
  const dispatch = useDispatch();
  const userId = useSelector((state) => state.editDataset.userId);
  const hfStatus = useSelector((state) => state.editDataset.hfStatus);

  const { controlHfServer, registerHFUser, getRegisteredHFUser } = useRosServiceCaller();

  // Local states
  const [activeSection, setActiveSection] = useState(SECTION_NAME.UPLOAD);
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
  const isProcessing = isUploading || isDownloading;

  // Section availability
  const canChangeSection = !isProcessing;

  const uploadButtonEnabled =
    !isUploading &&
    !isDownloading &&
    isEditable &&
    hfRepoIdUpload?.trim() &&
    hfLocalDirUpload?.trim();
  const downloadButtonEnabled =
    !isUploading &&
    !isDownloading &&
    isEditable &&
    hfRepoIdDownload?.trim() &&
    hfLocalDirDownload?.trim();

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
        const repoId = userId + '/' + hfRepoIdUpload.trim();
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
        const repoId = userId + '/' + hfRepoIdDownload.trim();
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

  // track hf status update
  useEffect(() => {
    if (hfStatus === HFStatus.UPLOADING) {
      setActiveSection(SECTION_NAME.UPLOAD);
      setIsUploading(true);
    } else if (hfStatus === HFStatus.DOWNLOADING) {
      setActiveSection(SECTION_NAME.DOWNLOAD);
      setIsDownloading(true);
    } else {
      setIsUploading(false);
      setIsDownloading(false);
    }
  }, [hfStatus]);

  return (
    <div className="w-full flex flex-col items-start justify-start bg-gray-100 p-10 gap-4 rounded-xl">
      <div className="w-full flex items-center justify-start">
        <span className="text-2xl font-bold mb-4">Hugging Face Upload & Download</span>
      </div>

      <div className="w-full flex flex-row items-start justify-start gap-4">
        {/* Progress Status Bar */}
        {/* {isProcessing && (
        <div className="w-full bg-blue-50 border border-blue-200 p-4 rounded-lg">
          <div className="flex items-center gap-3">
            <div className="animate-spin w-5 h-5 border-2 border-blue-500 border-t-transparent rounded-full"></div>
            <span className="text-blue-800 font-medium">
              {isUploading && 'Upload in progress...'}
              {isDownloading && 'Download in progress...'}
            </span>
            <span className="text-blue-600 text-sm ml-auto">
              Section switching disabled during transfer
            </span>
          </div>
        </div>
      )} */}

        {/* User ID Selection */}
        <div className="flex flex-col items-center justify-start gap-4">
          <div className="bg-white p-5 rounded-md flex flex-col items-start justify-center gap-4 shadow-md">
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
          {/* Section Selector */}
          <div className="flex items-center justify-start">
            <SectionSelector
              activeSection={activeSection}
              onSectionChange={setActiveSection}
              canChangeSection={canChangeSection}
            />
          </div>
        </div>

        {/* Active Section Content */}
        <div className="w-full">
          {activeSection === SECTION_NAME.UPLOAD && (
            <div className="w-full bg-white p-5 rounded-md flex flex-col items-start justify-center gap-2 shadow-md">
              {/* Upload Dataset Section Header */}
              <div className="w-full flex flex-col items-start justify-start gap-2 bg-gray-50 border border-gray-200 p-3 rounded-md">
                <div className="w-full flex items-center rounded-md font-medium gap-2">
                  <span className="text-lg">⬆️</span>
                  Upload Dataset
                </div>
                <div className="text-sm text-gray-600">
                  <div className="mb-1">
                    Uploads dataset from local directory to Hugging Face hub
                  </div>
                </div>
              </div>

              {/* Upload Dataset Section Content */}
              <div className="w-full flex flex-col gap-3">
                {/* Local Directory Input */}
                <div className="w-full flex flex-col gap-2">
                  <span className="text-lg font-bold">Local Directory</span>
                  <div className="w-full flex flex-row items-center justify-start gap-2">
                    <FolderBrowseButton
                      onClick={() => setShowHfLocalDirBrowserModal(true)}
                      disabled={isDownloading}
                      ariaLabel="Browse files for local directory"
                    />
                    <input
                      className={clsx(STYLES.textInput, 'flex-1', {
                        'bg-gray-100 cursor-not-allowed': !isEditable || isDownloading,
                        'bg-white': isEditable && !isDownloading,
                      })}
                      type="text"
                      placeholder="Enter local directory path or browse"
                      value={hfLocalDirUpload || ''}
                      onChange={(e) => setHfLocalDirUpload(e.target.value)}
                      disabled={!isEditable || isDownloading}
                    />
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
                            'bg-gray-100 cursor-not-allowed text-gray-500':
                              !isEditable || isDownloading,
                            'text-gray-900': isEditable && !isDownloading,
                          }
                        )}
                        type="text"
                        placeholder="Enter repository id"
                        value={hfRepoIdUpload || ''}
                        onChange={(e) => setHfRepoIdUpload(e.target.value)}
                        disabled={!isEditable || isDownloading}
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

                {/* Upload Button */}
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

                  {/* Status */}
                  <div className="flex flex-row items-center justify-start">
                    <span className="text-sm text-gray-500">
                      {isUploading && '⏳ Uploading...'}
                      {!isUploading && hfStatus}
                    </span>
                  </div>
                </div>
              </div>
            </div>
          )}

          {activeSection === SECTION_NAME.DOWNLOAD && (
            <div className="w-full bg-white p-5 rounded-md flex flex-col items-start justify-center gap-4 shadow-md">
              {/* Download Dataset Section Header */}
              <div className="w-full flex flex-col items-start justify-start gap-2 bg-gray-50 border border-gray-200 p-3 rounded-md">
                <div className="w-full flex items-center rounded-md font-medium gap-2">
                  <span className="text-lg">⬇️</span>
                  Download Dataset
                </div>
                <div className="text-sm text-gray-600">
                  <div className="mb-1">
                    Downloads dataset from Hugging Face hub to local cache directory
                  </div>
                </div>
              </div>

              {/* Download Dataset Section Content */}
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
                            'bg-gray-100 cursor-not-allowed text-gray-500':
                              !isEditable || isUploading,
                            'text-gray-900': isEditable && !isUploading,
                          }
                        )}
                        type="text"
                        placeholder="Enter repository id"
                        value={hfRepoIdDownload || ''}
                        onChange={(e) => setHfRepoIdDownload(e.target.value)}
                        disabled={!isEditable || isUploading}
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
                    <FolderBrowseButton
                      onClick={() => setShowHfLocalDirDownloadBrowserModal(true)}
                      disabled={isUploading}
                      ariaLabel="Browse files for local directory"
                    />
                    <input
                      className={clsx(STYLES.textInput, 'flex-1', {
                        'bg-gray-100 cursor-not-allowed': !isEditable || isUploading,
                        'bg-white': isEditable && !isUploading,
                      })}
                      type="text"
                      placeholder="Enter local directory path or browse"
                      value={hfLocalDirDownload || ''}
                      onChange={(e) => setHfLocalDirDownload(e.target.value)}
                      disabled={!isEditable || isUploading}
                    />
                  </div>
                </div>

                {/* Download Button */}
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

                  {/* Status */}
                  <div className="flex flex-row items-center justify-start">
                    <span className="text-sm text-gray-500">
                      {isDownloading && '⏳ Downloading...'}
                      {!isDownloading && hfStatus}
                    </span>
                  </div>
                </div>
              </div>
            </div>
          )}
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
