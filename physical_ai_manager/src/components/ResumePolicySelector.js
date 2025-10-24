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
import { useDispatch, useSelector } from 'react-redux';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import { MdFolderOpen, MdEdit, MdCheck, MdClose, MdWarning, MdDownload } from 'react-icons/md';
import {
  setResumePolicyPath,
  setTrainingInfo,
  setSelectedUser,
  setSelectedDataset,
  setHasTrainConfig as setHasTrainConfigRedux,
  setIsTrainingInfoLoaded,
} from '../features/training/trainingSlice';
import FileBrowserModal from './FileBrowserModal';
import { DEFAULT_PATHS, TARGET_FILES } from '../constants/paths';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';

export default function ResumePolicySelector() {
  const dispatch = useDispatch();
  const resumePolicyPath = useSelector((state) => state.training.resumePolicyPath);
  const isTrainingInfoLoaded = useSelector((state) => state.training.isTrainingInfoLoaded);
  const [showBrowserModal, setShowBrowserModal] = useState(false);
  const [isEditing, setIsEditing] = useState(false);
  const [isValidatingConfig, setIsValidatingConfig] = useState(false);
  const [hasTrainConfig, setHasTrainConfig] = useState(null);

  const { browseFile, getTrainingInfo } = useRosServiceCaller();
  const REQUIRED_BASE_PATH = DEFAULT_PATHS.POLICY_MODEL_PATH;

  // Get relative path after base path
  const getRelativePath = (fullPath) => {
    if (!fullPath) return '';
    if (fullPath.startsWith(REQUIRED_BASE_PATH)) {
      return fullPath.substring(REQUIRED_BASE_PATH.length);
    }
    return fullPath;
  };

  // Get full path from relative path
  const getFullPath = (relativePath) => {
    if (!relativePath) return REQUIRED_BASE_PATH;
    return REQUIRED_BASE_PATH + relativePath;
  };

  const [editValue, setEditValue] = useState(getRelativePath(resumePolicyPath));

  const classContainer = clsx(
    'flex',
    'flex-col',
    'gap-3',
    'p-6',
    'bg-white',
    'rounded-2xl',
    'shadow-md',
    'border',
    'border-gray-200',
    'min-w-[500px]'
  );

  const classHeader = clsx('flex', 'items-center', 'gap-2', 'mb-1');

  const classTitle = clsx('text-xl', 'font-bold', 'text-gray-700');

  const classPathDisplay = clsx(
    'flex',
    'items-center',
    'gap-2',
    'p-3',
    'bg-gray-50',
    'rounded-lg',
    'border',
    'border-gray-200',
    'min-h-[44px]'
  );

  const classPathText = clsx('flex-1', 'text-sm', 'font-mono', 'text-gray-700', 'truncate');

  const classButton = clsx(
    'flex',
    'items-center',
    'justify-center',
    'gap-2',
    'px-3',
    'py-2',
    'text-sm',
    'rounded-lg',
    'transition-colors',
    'disabled:opacity-50',
    'disabled:cursor-not-allowed',
    'whitespace-nowrap'
  );

  const classBrowseButton = clsx(
    classButton,
    'bg-blue-50',
    'text-blue-700',
    'border',
    'border-blue-200',
    'hover:bg-blue-100'
  );

  const classEditButton = clsx(
    classButton,
    'bg-gray-50',
    'text-gray-700',
    'border',
    'border-gray-200',
    'hover:bg-gray-100'
  );

  const classLoadButton = clsx(
    classButton,
    'bg-green-100',
    'text-green-700',
    'border',
    'font-medium',
    'border-green-300',
    'hover:bg-green-200'
  );

  const classConfirmButton = clsx(
    classButton,
    'bg-green-50',
    'text-green-700',
    'border',
    'border-green-200',
    'hover:bg-green-100'
  );

  const classCancelButton = clsx(
    classButton,
    'bg-red-50',
    'text-red-700',
    'border',
    'border-red-200',
    'hover:bg-red-100'
  );

  const classInput = clsx(
    'flex-1',
    'px-3',
    'py-2',
    'text-sm',
    'border',
    'border-blue-300',
    'rounded-lg',
    'focus:outline-none',
    'focus:ring-2',
    'focus:ring-blue-500',
    'focus:border-transparent'
  );

  const validatePath = (path) => {
    if (!path) return false;
    return path.startsWith(REQUIRED_BASE_PATH);
  };

  // Check if train_config.json exists in the selected path
  const checkTrainConfig = async (path) => {
    if (!path || !validatePath(path)) {
      setHasTrainConfig(null);
      dispatch(setHasTrainConfigRedux(null));
      return;
    }

    setIsValidatingConfig(true);
    try {
      const result = await browseFile('browse', path, '', [TARGET_FILES.TRAIN_CONFIG], []);
      console.log('checkTrainConfig result:', result);

      if (result.success && result.items) {
        const configExists = result.items.some(
          (item) => item.name === TARGET_FILES.TRAIN_CONFIG && !item.is_directory
        );
        setHasTrainConfig(configExists);
        dispatch(setHasTrainConfigRedux(configExists));

        if (!configExists) {
          toast.error(`${TARGET_FILES.TRAIN_CONFIG} file not found in selected path`, {
            duration: 5000,
          });
        }
      } else {
        setHasTrainConfig(false);
        dispatch(setHasTrainConfigRedux(false));
      }
    } catch (error) {
      console.error('Error checking train_config.json:', error);
      setHasTrainConfig(false);
      dispatch(setHasTrainConfigRedux(false));
      toast.error('Failed to check for train_config.json file');
    } finally {
      setIsValidatingConfig(false);
    }
  };

  // Check train_config.json when path changes
  useEffect(() => {
    if (resumePolicyPath) {
      checkTrainConfig(resumePolicyPath);
    } else {
      setHasTrainConfig(null);
      dispatch(setHasTrainConfigRedux(null));
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [resumePolicyPath]);

  const handleBrowserSelect = (item) => {
    setShowBrowserModal(false);
    if (item && item.full_path) {
      if (!validatePath(item.full_path)) {
        toast.error(`Please select a path under: ${REQUIRED_BASE_PATH}`, { duration: 5000 });
        return;
      }
      dispatch(setResumePolicyPath(item.full_path));
      setEditValue(getRelativePath(item.full_path));
    }
  };

  const handleEditStart = () => {
    setEditValue(getRelativePath(resumePolicyPath));
    setIsEditing(true);
  };

  const handleEditConfirm = () => {
    // If editValue is empty, clear the path
    if (!editValue || editValue.trim() === '') {
      dispatch(setResumePolicyPath(undefined));
      dispatch(setHasTrainConfigRedux(null));
      setIsEditing(false);
      return;
    }

    const fullPath = getFullPath(editValue);
    if (!validatePath(fullPath)) {
      toast.error(`Please select a path under: ${REQUIRED_BASE_PATH}`, { duration: 5000 });
      return;
    }
    dispatch(setResumePolicyPath(fullPath));
    setIsEditing(false);
  };

  const handleEditCancel = () => {
    setEditValue(getRelativePath(resumePolicyPath));
    setIsEditing(false);
  };

  const handleInputChange = (e) => {
    setEditValue(e.target.value);
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter') {
      handleEditConfirm();
    } else if (e.key === 'Escape') {
      handleEditCancel();
    }
  };

  const handleLoadTrainingInfo = async () => {
    try {
      const result = await getTrainingInfo(
        getRelativePath(resumePolicyPath) + '/' + TARGET_FILES.TRAIN_CONFIG
      );
      console.log('Training info received:', result);
      if (result) {
        if (result.success) {
          dispatch(
            setTrainingInfo({
              datasetRepoId: result.training_info.dataset,
              policyType: result.training_info.policy_type,
              policyDevice: result.training_info.policy_device,
              outputFolderName: result.training_info.output_folder_name,
              resume: result.training_info.resume,
              seed: result.training_info.seed,
              numWorkers: result.training_info.num_workers,
              batchSize: result.training_info.batch_size,
              steps: result.training_info.steps,
              evalFreq: result.training_info.eval_freq,
              logFreq: result.training_info.log_freq,
              saveFreq: result.training_info.save_freq,
            })
          );
          const selectedUser = result.training_info.dataset.split('/')[0];
          const selectedDataset = result.training_info.dataset.split('/')[1];
          console.log('selectedUser:', selectedUser);
          console.log('selectedDataset:', selectedDataset);
          dispatch(setSelectedUser(selectedUser));
          dispatch(setSelectedDataset(selectedDataset));
          dispatch(setIsTrainingInfoLoaded(true));
          toast.success('Training info loaded successfully');
        } else {
          dispatch(setIsTrainingInfoLoaded(false));
          toast.error('Failed to get training info: ' + result.message);
        }
      } else {
        dispatch(setIsTrainingInfoLoaded(false));
        toast.error('Failed to get training info: Invalid response');
      }
    } catch (error) {
      console.error('Error fetching training info:', error);
      dispatch(setIsTrainingInfoLoaded(false));
      toast.error('Failed to get training info');
    }
  };

  return (
    <>
      <div className={classContainer}>
        <div className={classHeader}>
          <h3 className={classTitle}>Checkpoint Path to Resume</h3>
          <span className="text-xs text-red-500">*</span>
        </div>

        {isEditing ? (
          <>
            {/* Base Path (Fixed) and Relative Path (Editable) - Responsive Layout */}
            <div className="flex flex-col gap-2">
              {/* Base Path (Fixed) */}
              <div className="flex-shrink-0">
                <span className="text-sm text-gray-600 font-mono">{REQUIRED_BASE_PATH}</span>
              </div>

              {/* Relative Path (Editable) */}
              <div className="flex items-center gap-2 flex-1">
                <input
                  type="text"
                  value={editValue}
                  onChange={handleInputChange}
                  onKeyDown={handleKeyDown}
                  className={classInput}
                  placeholder="example_policy/checkpoint-1000/"
                  autoFocus
                />
                <button onClick={handleEditConfirm} className={classConfirmButton} title="Confirm">
                  <MdCheck size={18} />
                </button>
                <button onClick={handleEditCancel} className={classCancelButton} title="Cancel">
                  <MdClose size={18} />
                </button>
              </div>
            </div>

            {editValue && !validatePath(getFullPath(editValue)) && (
              <div className="flex items-center gap-2 p-2 bg-red-50 border border-red-200 rounded-lg">
                <MdWarning className="text-red-500 flex-shrink-0" size={18} />
                <span className="text-xs text-red-700">
                  Path must be under: {REQUIRED_BASE_PATH}
                </span>
              </div>
            )}
          </>
        ) : (
          <>
            {/* Base Path (Fixed) and Relative Path (Display) - Responsive Layout */}
            <div className="flex flex-col gap-2">
              {/* Base Path (Fixed) */}
              <div className="flex-shrink-0">
                <span className="text-sm text-gray-600 font-mono">{REQUIRED_BASE_PATH}</span>
              </div>

              {/* Relative Path (Display) */}
              <div className={clsx(classPathDisplay, 'flex-1')}>
                {resumePolicyPath ? (
                  <span className={classPathText} title={getRelativePath(resumePolicyPath)}>
                    {getRelativePath(resumePolicyPath) || '/'}
                  </span>
                ) : (
                  <span className="flex-1 text-sm text-gray-400 italic">
                    No policy path selected
                  </span>
                )}
              </div>
            </div>

            {resumePolicyPath && !validatePath(resumePolicyPath) && (
              <div className="flex items-center gap-2 p-2 bg-red-50 border border-red-200 rounded-lg">
                <MdWarning className="text-red-500 flex-shrink-0" size={18} />
                <span className="text-xs text-red-700">
                  Invalid path. Must be under: {REQUIRED_BASE_PATH}
                </span>
              </div>
            )}

            {isValidatingConfig && (
              <div className="flex items-center gap-2 p-2 bg-blue-50 border border-blue-200 rounded-lg">
                <div className="animate-spin rounded-full h-4 w-4 border-b-2 border-blue-500"></div>
                <span className="text-xs text-blue-700">Checking for train_config.json...</span>
              </div>
            )}

            {!isValidatingConfig && hasTrainConfig === false && resumePolicyPath && (
              <div className="flex items-center gap-2 p-2 bg-red-50 border border-red-200 rounded-lg">
                <MdWarning className="text-red-500 flex-shrink-0" size={18} />
                <span className="text-xs text-red-700">
                  {TARGET_FILES.TRAIN_CONFIG} file not found in this directory
                </span>
              </div>
            )}

            <div className="flex items-center gap-2">
              <button
                onClick={() => setShowBrowserModal(true)}
                className={classBrowseButton}
                title="Browse for policy checkpoint"
              >
                <MdFolderOpen size={18} />
                <span>Browse</span>
              </button>
              <button
                onClick={handleEditStart}
                className={classEditButton}
                title="Edit path manually"
              >
                <MdEdit size={18} />
                <span>Edit</span>
              </button>
              <div className="flex items-center gap-2 ml-auto">
                <button
                  onClick={handleLoadTrainingInfo}
                  className={classLoadButton}
                  title="Load training info"
                >
                  <MdDownload size={18} />
                  <span>Load</span>
                </button>
                {isTrainingInfoLoaded && (
                  <MdCheck
                    className="text-green-600 flex-shrink-0"
                    size={20}
                    title="Training info loaded successfully"
                  />
                )}
              </div>
            </div>
          </>
        )}

        <p className="text-xs text-gray-500 mt-1">
          Select a policy directory containing {TARGET_FILES.TRAIN_CONFIG} file
        </p>
      </div>

      <FileBrowserModal
        isOpen={showBrowserModal}
        onClose={() => setShowBrowserModal(false)}
        onFileSelect={handleBrowserSelect}
        title="Select Checkpoint Path"
        selectButtonText="Select"
        allowDirectorySelect={true}
        targetFileName={[TARGET_FILES.TRAIN_CONFIG]}
        targetFileLabel="Policy checkpoint found! 🎯"
        initialPath={DEFAULT_PATHS.POLICY_MODEL_PATH}
        defaultPath={DEFAULT_PATHS.POLICY_MODEL_PATH}
        homePath=""
      />
    </>
  );
}
