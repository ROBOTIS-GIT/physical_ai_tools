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

import React, { useState, useCallback, useEffect } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import { MdCloudUpload, MdFolder, MdClose } from 'react-icons/md';
import TokenInputPopup from '../components/TokenInputPopup';
import FileBrowserModal from '../components/FileBrowserModal';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import { DEFAULT_PATHS } from '../constants/paths';
import {
  setHfUserId,
  setHfToken,
  setSelectedFolders,
  clearSelectedFolders,
  startUpload,
  completeUpload,
  setError,
} from '../features/dataUploader/dataUploaderSlice';

// Constants
const ROSBAG_BASE_PATH = DEFAULT_PATHS.ROSBAG2_PATH;

// Style Classes
const STYLES = {
  container: clsx(
    'w-full',
    'h-full',
    'flex',
    'flex-col',
    'items-start',
    'justify-start',
    'overflow-scroll',
    'p-6'
  ),
  card: clsx(
    'w-full',
    'bg-white',
    'rounded-xl',
    'shadow-sm',
    'border',
    'border-gray-200',
    'p-6',
    'mb-4'
  ),
  sectionTitle: clsx(
    'text-lg',
    'font-semibold',
    'text-gray-900',
    'mb-4',
    'flex',
    'items-center',
    'gap-2'
  ),
  button: clsx(
    'px-4',
    'py-2',
    'rounded-lg',
    'font-medium',
    'transition-all',
    'duration-200',
    'focus:outline-none',
    'focus:ring-2',
    'focus:ring-opacity-50'
  ),
  buttonPrimary: clsx(
    'bg-blue-500',
    'text-white',
    'hover:bg-blue-600',
    'focus:ring-blue-400',
    'disabled:bg-gray-300',
    'disabled:cursor-not-allowed'
  ),
  buttonSecondary: clsx(
    'bg-gray-100',
    'text-gray-700',
    'hover:bg-gray-200',
    'focus:ring-gray-400'
  ),
};

export default function DataUploaderPage() {
  const dispatch = useDispatch();
  const { hfUserId, hfToken, selectedFolders, uploadProgress, isUploading } = useSelector(
    (state) => state.dataUploader
  );

  // Local state
  const [showTokenPopup, setShowTokenPopup] = useState(false);
  const [showFileBrowser, setShowFileBrowser] = useState(false);

  // ROS service caller
  const { registerHFUser, getRegisteredHFUser, uploadRosbagFolders } = useRosServiceCaller();

  // Load saved token on mount
  useEffect(() => {
    const loadSavedToken = async () => {
      try {
        const result = await getRegisteredHFUser();

        if (result.success && result.user_id_list && result.user_id_list.length > 0) {
          const userId = result.user_id_list[0];
          dispatch(setHfUserId(userId));
          // Note: We don't store the actual token in state for security
          // The token is stored on the server and used automatically
          console.log(`Auto-loaded saved token for user: ${userId}`);
        }
      } catch (error) {
        // No saved token or error loading - this is normal on first visit
        console.log('No saved token found');
      }
    };

    loadSavedToken();
  }, [getRegisteredHFUser, dispatch]);

  // Token registration handler
  const handleTokenSubmit = useCallback(async (token) => {
    try {
      const result = await registerHFUser(token);

      if (result.success && result.user_id_list && result.user_id_list.length > 0) {
        const userId = result.user_id_list[0];
        dispatch(setHfUserId(userId));
        dispatch(setHfToken(token));
        toast.success(`Logged in as ${userId}`);
        setShowTokenPopup(false);
      } else {
        toast.error('Failed to register token');
      }
    } catch (error) {
      console.error('Token registration error:', error);
      toast.error(`Token registration failed: ${error.message}`);
    }
  }, [dispatch, registerHFUser]);

  // Folder selection handler
  const handleFoldersSelect = useCallback((folders) => {
    // folders can be single object or array
    const folderArray = Array.isArray(folders) ? folders : [folders];
    const folderItems = folderArray.map(folder => ({
      name: folder.name,
      path: folder.full_path,
    }));
    dispatch(setSelectedFolders(folderItems));
    toast.success(`Selected ${folderItems.length} folder(s)`);
  }, [dispatch]);

  // Remove folder handler
  const handleRemoveFolder = useCallback((path) => {
    const newFolders = selectedFolders.filter(f => f.path !== path);
    dispatch(setSelectedFolders(newFolders));
  }, [dispatch, selectedFolders]);

  // Upload handler
  const handleUpload = useCallback(async () => {
    if (!hfUserId || selectedFolders.length === 0) {
      toast.error('Please register token and select folders');
      return;
    }

    dispatch(startUpload());
    toast.loading(`Starting upload of ${selectedFolders.length} folder(s)...`, {
      duration: 3000,
    });

    try {
      const folderPaths = selectedFolders.map(f => f.path);
      // Use saved token from server if hfToken is not in state
      const result = await uploadRosbagFolders(folderPaths, hfToken || '');

      if (result.success) {
        toast.success('Upload started successfully!');
        dispatch(completeUpload());
      } else {
        toast.error(`Upload failed: ${result.message}`);
        dispatch(setError(result.message));
      }
    } catch (error) {
      console.error('Upload error:', error);
      toast.error(`Upload failed: ${error.message}`);
      dispatch(setError(error.message));
    }
  }, [dispatch, hfUserId, hfToken, selectedFolders, uploadRosbagFolders]);

  // Render token section
  const renderTokenSection = () => (
    <div className={STYLES.card}>
      <h2 className={STYLES.sectionTitle}>
        <MdCloudUpload className="w-6 h-6 text-blue-500" />
        HuggingFace Hub Configuration
      </h2>

      {!hfUserId ? (
        <div className="flex flex-col items-start gap-3">
          <p className="text-sm text-gray-600">
            Register your HuggingFace token to upload rosbag data
          </p>
          <button
            className={clsx(STYLES.button, STYLES.buttonPrimary)}
            onClick={() => setShowTokenPopup(true)}
          >
            Register Token
          </button>
        </div>
      ) : (
        <div className="flex items-center justify-between">
          <div>
            <p className="text-sm text-gray-600 mb-1">Current User:</p>
            <p className="text-lg font-semibold text-blue-600">{hfUserId}</p>
          </div>
          <button
            className={clsx(STYLES.button, STYLES.buttonSecondary)}
            onClick={() => setShowTokenPopup(true)}
          >
            Change Token
          </button>
        </div>
      )}
    </div>
  );

  // Render upload section
  const renderUploadSection = () => (
    <div className={STYLES.card}>
      <h2 className={STYLES.sectionTitle}>
        <MdFolder className="w-6 h-6 text-green-500" />
        Upload Rosbag
      </h2>

      {/* Folder selection */}
      <div className="mb-6">
        <button
          className={clsx(STYLES.button, STYLES.buttonSecondary, 'w-full', 'justify-center')}
          onClick={() => setShowFileBrowser(true)}
        >
          <MdFolder className="w-5 h-5 mr-2" />
          Select Rosbag Folders
        </button>
      </div>

      {/* Selected folders list */}
      {selectedFolders.length > 0 && (
        <div className="mb-6">
          <h3 className="text-sm font-medium text-gray-700 mb-3">
            Selected Folders ({selectedFolders.length}):
          </h3>
          <div className="space-y-2">
            {selectedFolders.map((folder) => (
              <div
                key={folder.path}
                className="flex items-center justify-between p-3 bg-gray-50 rounded-lg border border-gray-200"
              >
                <div className="flex-1">
                  <p className="text-sm font-medium text-gray-900">{folder.name}</p>
                  <p className="text-xs text-gray-500 mt-1">
                    → {hfUserId}/{folder.name}
                  </p>
                </div>
                <button
                  className="p-2 text-gray-400 hover:text-red-500 transition-colors"
                  onClick={() => handleRemoveFolder(folder.path)}
                  disabled={isUploading}
                >
                  <MdClose className="w-5 h-5" />
                </button>
              </div>
            ))}
          </div>
        </div>
      )}

      {/* Upload button */}
      <button
        className={clsx(STYLES.button, STYLES.buttonPrimary, 'w-full', 'justify-center')}
        onClick={handleUpload}
        disabled={!hfUserId || selectedFolders.length === 0 || isUploading}
      >
        <MdCloudUpload className="w-5 h-5 mr-2" />
        {isUploading ? 'Uploading...' : `Upload All (${selectedFolders.length})`}
      </button>

      {/* Upload progress */}
      {uploadProgress.length > 0 && (
        <div className="mt-6">
          <h3 className="text-sm font-medium text-gray-700 mb-3">Upload Progress:</h3>
          <div className="space-y-3">
            {uploadProgress.map((item) => (
              <div key={item.folderName} className="space-y-1">
                <div className="flex justify-between text-sm">
                  <span className="text-gray-700">{item.folderName}</span>
                  <span className="text-gray-500">{item.percentage}%</span>
                </div>
                <div className="w-full bg-gray-200 rounded-full h-2">
                  <div
                    className={clsx(
                      'h-2 rounded-full transition-all duration-300',
                      {
                        'bg-blue-500': item.status === 'uploading' || item.status === 'pending',
                        'bg-green-500': item.status === 'completed',
                        'bg-red-500': item.status === 'failed',
                      }
                    )}
                    style={{ width: `${item.percentage}%` }}
                  />
                </div>
                <p className="text-xs text-gray-500">{item.repoId}</p>
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );

  return (
    <div className={STYLES.container}>
      <div className="w-full max-w-4xl mx-auto">
        <h1 className="text-3xl font-bold text-gray-900 mb-6">Data Uploader</h1>

        {renderTokenSection()}
        {renderUploadSection()}
      </div>

      {/* Modals */}
      <TokenInputPopup
        isOpen={showTokenPopup}
        onClose={() => setShowTokenPopup(false)}
        onSubmit={handleTokenSubmit}
      />

      <FileBrowserModal
        isOpen={showFileBrowser}
        onClose={() => setShowFileBrowser(false)}
        onFileSelect={handleFoldersSelect}
        initialPath={ROSBAG_BASE_PATH}
        defaultPath={ROSBAG_BASE_PATH}
        homePath=""
        title="Select Rosbag Folders"
        selectButtonText="Select Folders"
        allowDirectorySelect={true}
        allowFileSelect={false}
        multiSelect={true}
      />
    </div>
  );
}
