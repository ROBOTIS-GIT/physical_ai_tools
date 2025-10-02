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

import React, { useState, useCallback, useMemo } from 'react';
import clsx from 'clsx';
import { useSelector, useDispatch } from 'react-redux';
import toast from 'react-hot-toast';
import { MdFolderOpen, MdRefresh } from 'react-icons/md';
import {
  setDatasetToDeleteEpisode,
  setDeleteEpisodeNums,
  setDatasetInfo,
} from '../editDatasetSlice';
import { useRosServiceCaller } from '../../../hooks/useRosServiceCaller';
import FileBrowserModal from '../../../components/FileBrowserModal';
import { DEFAULT_PATHS, TARGET_FOLDERS } from '../../../constants/paths';

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
  textarea: clsx(
    'w-full',
    'text-sm',
    'resize-y',
    'min-h-16',
    'max-h-24',
    'p-2',
    'border',
    'border-gray-300',
    'rounded-lg',
    'focus:outline-none',
    'focus:ring-2',
    'focus:ring-blue-500',
    'focus:border-transparent'
  ),
  button: clsx(
    'px-5',
    'py-3',
    'm-5',
    'bg-blue-500',
    'text-white',
    'rounded-xl',
    'hover:bg-blue-600',
    'focus:outline-none',
    'focus:ring-2',
    'focus:ring-blue-500',
    'flex',
    'items-center',
    'gap-5',
    'text-xl',
    'font-medium',
    'shadow-md',
    'disabled:opacity-50',
    'disabled:cursor-not-allowed'
  ),
};

// Utility Functions
const parseEpisodeNumbers = (input) => {
  if (!input || typeof input !== 'string') return [];

  const numbers = new Set();
  const parts = input.split(',').map((part) => part.trim());

  for (const part of parts) {
    if (part.includes('-')) {
      // Handle range (e.g., "10-15")
      const [start, end] = part.split('-').map((num) => parseInt(num.trim()));
      if (!isNaN(start) && !isNaN(end) && start <= end) {
        for (let i = start; i <= end; i++) {
          numbers.add(i);
        }
      }
    } else {
      // Handle single number
      const num = parseInt(part.trim());
      if (!isNaN(num) && num >= 0) {
        numbers.add(num);
      }
    }
  }

  return Array.from(numbers).sort((a, b) => a - b);
};

const showOperationSuccess = (operation, episodeNums = []) => {
  if (operation === 'delete') {
    const episodeText = episodeNums.length > 0 ? ` (Episodes: ${episodeNums.join(', ')})` : '';
    toast.success(`Dataset deleted successfully!${episodeText}`);
  }
};

const showOperationError = (operation, errorMessage = '') => {
  const operationText = 'delete';
  const message = errorMessage
    ? `Failed to ${operationText} dataset:\n${errorMessage}`
    : `Failed to ${operationText} dataset`;
  toast.error(message);
};

// EpisodeNumberInput Component
const EpisodeNumberInput = ({ value, onChange, disabled = false, className, parseFunction }) => {
  const parsedNumbers = useMemo(() => parseFunction(value), [value, parseFunction]);

  const hasValidInput = value && parsedNumbers.length > 0;
  const previewText = hasValidInput ? parsedNumbers.join(', ') : 'No valid episodes';

  return (
    <div className="flex flex-col gap-2 w-full">
      <input
        className={clsx(className, {
          'bg-gray-100 cursor-not-allowed': disabled,
          'bg-white': !disabled,
        })}
        type="text"
        placeholder="Enter episode numbers to delete (e.g., 0,1,2,3,10-15,20)"
        value={value}
        onChange={(e) => onChange(e.target.value)}
        disabled={disabled}
        aria-label="Episode numbers input"
      />
      {value && (
        <div className="text-sm text-gray-600" role="status" aria-live="polite">
          <span className="font-medium">Preview:</span> {previewText} ({parsedNumbers.length}{' '}
          {parsedNumbers.length === 1 ? 'episode' : 'episodes'})
        </div>
      )}
    </div>
  );
};

const DeleteSection = ({ isEditable = true }) => {
  const dispatch = useDispatch();
  const { datasetToDeleteEpisode, deleteEpisodeNums, datasetInfo } = useSelector(
    (state) => state.editDataset
  );
  const { sendEditDatasetCommand, getDatasetInfo } = useRosServiceCaller();

  // Local states
  const [deleteEpisodeNumsInput, setDeleteEpisodeNumsInput] = useState('');
  const [showSelectDatasetPathBrowserModal, setShowSelectDatasetPathBrowserModal] = useState(false);

  const fetchDatasetInfo = useCallback(
    async (datasetPath) => {
      if (!datasetPath || datasetPath === '') {
        toast.error('Dataset path is empty');
        return;
      }

      try {
        const result = await getDatasetInfo(datasetPath);
        console.log('Dataset info result:', result);
        if (result?.success) {
          dispatch(
            setDatasetInfo({
              ...result.dataset_info,
              totalEpisodes: result.dataset_info.total_episodes,
              totalTasks: result.dataset_info.total_tasks,
              fps: result.dataset_info.fps,
              codebaseVersion: result.dataset_info.codebase_version,
              robotType: result.dataset_info.robot_type,
            })
          );
        } else {
          toast.error('Failed to get dataset info: ' + result.message);
        }
      } catch (error) {
        console.error('Error fetching dataset info:', error);
        toast.error('Failed to get dataset info: ' + error.message);
      }
    },
    [getDatasetInfo, dispatch]
  );

  // Event handlers
  const handlers = {
    datasetToDeleteEpisodeChange: (newDatasetToDelete) => {
      dispatch(setDatasetToDeleteEpisode(newDatasetToDelete));
    },

    deleteEpisodeNumsChange: (inputValue) => {
      setDeleteEpisodeNumsInput(inputValue);
      dispatch(setDeleteEpisodeNums(parseEpisodeNumbers(inputValue)));
    },

    selectDatasetPathSelect: useCallback(
      (item) => {
        dispatch(setDatasetToDeleteEpisode(item.full_path));
        setShowSelectDatasetPathBrowserModal(false);
        fetchDatasetInfo(item.full_path);
      },
      [dispatch, fetchDatasetInfo]
    ),
  };

  // Operations
  const operations = {
    deleteDataset: async () => {
      try {
        const result = await sendEditDatasetCommand('delete');
        console.log('Delete dataset result:', result);

        if (result?.success) {
          showOperationSuccess('delete', deleteEpisodeNums);
          fetchDatasetInfo(datasetToDeleteEpisode);
        } else {
          if (result?.message !== '') showOperationError('delete', result.message);
          else showOperationError('delete');
        }
      } catch (error) {
        console.error('Error deleting dataset:', error);
        showOperationError('delete', error.message);
      }
    },
  };

  return (
    <div className="w-full flex flex-col items-center justify-start bg-gray-100 p-10 gap-8 rounded-xl">
      <div className="w-full flex items-center justify-start">
        <h1 className="text-2xl font-bold mb-4">Delete Episodes from Dataset</h1>
      </div>

      <div className="flex flex-row items-center justify-start gap-20 w-full">
        <div className="flex flex-col items-start justify-start gap-2 w-full">
          <div className="flex items-center justify-start gap-2 w-full">
            <div className="flex flex-row items-center justify-start gap-2 bg-white pr-2 pl-4 py-2 rounded-full shadow-md">
              <span className="text-md font-bold">Total Episodes</span>
              <span className="text-lg font-bold bg-gray-200 px-3 py-0 rounded-full">
                {datasetInfo.totalEpisodes}
              </span>
            </div>
            <button
              onClick={() => fetchDatasetInfo(datasetToDeleteEpisode)}
              className="flex items-center justify-center text-blue-500 rounded-md p-1 hover:text-blue-700 hover:bg-gray-200"
            >
              <MdRefresh className="w-8 h-8" />
            </button>
          </div>
          <div className="flex items-center justify-center gap-2 w-full">
            <textarea
              className={clsx(STYLES.textarea, {
                'bg-gray-100 cursor-not-allowed': !isEditable,
                'bg-white': isEditable,
                'shadow-sm': isEditable,
              })}
              value={datasetToDeleteEpisode}
              onChange={(e) => handlers.datasetToDeleteEpisodeChange(e.target.value)}
              disabled={!isEditable}
              placeholder="Enter dataset to delete episodes"
            />

            <button
              type="button"
              onClick={() => setShowSelectDatasetPathBrowserModal(true)}
              className="flex items-center justify-center w-12 h-12 text-blue-500 bg-gray-200 rounded-md hover:text-blue-700"
              aria-label="Browse files for dataset to delete"
            >
              <MdFolderOpen className="w-10 h-10" />
            </button>
          </div>
        </div>

        <div className="flex flex-col items-start justify-start gap-2 w-full">
          <span className="text-lg font-bold">Episode Numbers to Delete</span>
          <EpisodeNumberInput
            value={deleteEpisodeNumsInput}
            onChange={handlers.deleteEpisodeNumsChange}
            disabled={!isEditable}
            className={clsx(STYLES.textInput, {
              'bg-gray-100 cursor-not-allowed': !isEditable,
              'bg-white': isEditable,
              'shadow-sm': isEditable,
            })}
            parseFunction={parseEpisodeNumbers}
          />
        </div>
      </div>

      <button
        className={STYLES.button}
        onClick={operations.deleteDataset}
        disabled={datasetToDeleteEpisode === '' || deleteEpisodeNums.length === 0 || !isEditable}
      >
        Delete
      </button>

      {/* File Browser Modal */}
      <FileBrowserModal
        isOpen={showSelectDatasetPathBrowserModal}
        onClose={() => setShowSelectDatasetPathBrowserModal(false)}
        onFileSelect={handlers.selectDatasetPathSelect}
        title="Select Dataset Path"
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
    </div>
  );
};

export default DeleteSection;
