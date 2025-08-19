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

import React, { useEffect, useState, useMemo, useCallback } from 'react';
import clsx from 'clsx';
import toast, { useToasterStore } from 'react-hot-toast';
import { useSelector, useDispatch } from 'react-redux';
import { TbArrowMerge } from 'react-icons/tb';
import { MdFolderOpen } from 'react-icons/md';
import { MdDataset } from 'react-icons/md';
import { DEFAULT_PATHS, TARGET_FOLDERS } from '../constants/paths';
import FileBrowserModal from '../components/FileBrowserModal';

import {
  setMergeDatasetList,
  setDatasetToDeleteEpisode,
  setDeleteEpisodeNums,
  setMergeOutputPath,
  setMergeOutputFolderName,
} from '../features/editDataset/editDatasetSlice';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';

// Constants
const TOAST_LIMIT = 3;

// Style Classes
const STYLES = {
  container: clsx(
    'w-full',
    'h-full',
    'flex',
    'flex-col',
    'items-start',
    'justify-start',
    'overflow-scroll'
  ),
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
    'rounded-md',
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
  datasetTextarea: clsx(
    'w-full',
    'p-2',
    'border',
    'border-gray-200',
    'rounded',
    'text-sm',
    'resize-none',
    'min-h-8',
    'h-12',
    'focus:ring-2',
    'focus:ring-blue-500',
    'focus:border-transparent'
  ),
  removeButton: clsx(
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
  ),
  addButton: clsx(
    'px-3',
    'py-1',
    'bg-blue-500',
    'text-white',
    'rounded-md',
    'hover:bg-blue-600',
    'focus:outline-none',
    'focus:ring-2',
    'focus:ring-blue-500',
    'flex',
    'items-center',
    'gap-2',
    'text-sm',
    'font-medium'
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

const manageTostLimit = (toasts) => {
  toasts
    .filter((t) => t.visible) // Only consider visible toasts
    .filter((_, i) => i >= TOAST_LIMIT) // Is toast index over limit?
    .forEach((t) => toast.dismiss(t.id)); // Dismiss
};

const showOperationSuccess = (operation, episodeNums = []) => {
  if (operation === 'delete') {
    const episodeText = episodeNums.length > 0 ? ` (Episodes: ${episodeNums.join(', ')})` : '';
    toast.success(`Dataset deleted successfully!${episodeText}`);
  } else if (operation === 'merge') {
    toast.success('Dataset merged successfully!');
  }
};

const showOperationError = (operation, errorMessage = '') => {
  const operationText = operation === 'delete' ? 'delete' : 'merge';
  const message = errorMessage
    ? `Failed to ${operationText} dataset: ${errorMessage}`
    : `Failed to ${operationText} dataset`;
  toast.error(message);
};

// Check for duplicate datasets in the list
const checkForDuplicateDatasets = (datasets) => {
  const normalizedPaths = datasets
    .filter((path) => path && path.trim() !== '') // Filter out empty paths
    .map((path) => path.trim().replace(/\/$/, '')); // Normalize paths by removing trailing slashes

  const uniquePaths = new Set(normalizedPaths);
  const duplicates = [];

  // Find duplicates
  normalizedPaths.forEach((path, index) => {
    const firstIndex = normalizedPaths.indexOf(path);
    if (firstIndex !== index && !duplicates.some((dup) => dup.path === path)) {
      duplicates.push({
        path: path,
        indices: normalizedPaths.map((p, i) => (p === path ? i : -1)).filter((i) => i !== -1),
      });
    }
  });

  return {
    hasDuplicates: duplicates.length > 0,
    duplicates: duplicates,
  };
};

const showDuplicateWarning = (duplicates) => {
  const duplicateList = duplicates
    .map((dup) => `"${dup.path}" (positions: ${dup.indices.map((i) => i + 1).join(', ')})`)
    .join('\n');

  toast.error(
    `Duplicate datasets detected:\n${duplicateList}\n\nPlease remove duplicates before merging.`,
    {
      duration: 6000,
      style: {
        maxWidth: '500px',
        whiteSpace: 'pre-line',
      },
    }
  );
};

const DatasetListInput = ({
  datasets = [''],
  onChange,
  disabled = false,
  className,
  setShowDatasetFileBrowserModal,
  selectingDatasetIndex,
  setSelectingDatasetIndex,
}) => {
  // Initialize local state with proper validation
  const [localDatasets, setLocalDatasets] = useState(() =>
    Array.isArray(datasets) && datasets.length > 0 ? datasets : ['']
  );

  // Sync local state with props
  useEffect(() => {
    const validDatasets = Array.isArray(datasets) && datasets.length > 0 ? datasets : [''];
    setLocalDatasets(validDatasets);
  }, [datasets]);

  // Dataset management functions
  const datasetActions = {
    add: () => {
      const newDatasets = [...localDatasets, ''];
      setLocalDatasets(newDatasets);
      onChange(newDatasets);
    },

    remove: (index) => {
      if (localDatasets.length > 1) {
        const newDatasets = localDatasets.filter((_, i) => i !== index);
        setLocalDatasets(newDatasets);
        onChange(newDatasets);
      }
    },

    update: (index, value) => {
      const newDatasets = [...localDatasets];
      newDatasets[index] = value;
      setLocalDatasets(newDatasets);
      onChange(newDatasets);
    },

    selectFile: (index) => {
      setSelectingDatasetIndex(index);
      setShowDatasetFileBrowserModal(true);
    },
  };

  // Render individual dataset input row
  const renderDatasetRow = (dataset, index) => (
    <div key={index} className="flex flex-row items-center justify-start gap-2 w-full">
      <div className="relative w-full">
        <textarea
          value={dataset}
          onChange={(e) => datasetActions.update(index, e.target.value)}
          disabled={disabled}
          placeholder={`Dataset ${index + 1}`}
          className={clsx(STYLES.datasetTextarea, {
            'bg-gray-100 cursor-not-allowed': disabled,
            'bg-white': !disabled,
            'pr-10': !disabled && localDatasets.length > 1,
          })}
          rows={2}
        />
        {!disabled && localDatasets.length > 1 && (
          <button
            type="button"
            onClick={() => datasetActions.remove(index)}
            className={STYLES.removeButton}
            aria-label={`Remove dataset ${index + 1}`}
          >
            ×
          </button>
        )}
      </div>
      <button
        type="button"
        onClick={() => datasetActions.selectFile(index)}
        className="flex items-center justify-center w-10 h-10 text-blue-500 bg-gray-200 rounded-md hover:text-blue-700"
        aria-label={`Browse files for dataset ${index + 1}`}
      >
        <MdFolderOpen className="w-8 h-8" />
      </button>
    </div>
  );

  return (
    <div className={clsx('w-full', className)}>
      <div className="max-h-48 overflow-y-auto border border-gray-300 rounded-md bg-white">
        <div className="p-2 space-y-2">{localDatasets.map(renderDatasetRow)}</div>
      </div>

      {!disabled && (
        <div className="mt-3 flex justify-between items-center">
          <button
            type="button"
            onClick={datasetActions.add}
            className={STYLES.addButton}
            aria-label="Add new dataset"
          >
            <span className="text-base font-bold">+</span>
            Add Dataset
          </button>
          <div className="flex flex-row items-center justify-start gap-2 text-md text-green-600 mb-0.5 px-0 select-none">
            <MdDataset className="w-5 h-5 text-green-600" />
            {localDatasets.length}
          </div>
        </div>
      )}
    </div>
  );
};

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
        placeholder="Enter episode numbers to delete (e.g., 1,2,3,10-15,20)"
        value={value}
        onChange={(e) => onChange(e.target.value)}
        disabled={disabled}
        aria-label="Episode numbers input"
      />
      {value && (
        <div className="text-sm text-gray-600" role="status" aria-live="polite">
          <span className="font-medium">Preview:</span> {previewText} ({parsedNumbers.length}{' '}
          episodes)
        </div>
      )}
    </div>
  );
};

export default function EditDatasetPage() {
  // Hooks and state management
  const { toasts } = useToasterStore();
  const dispatch = useDispatch();
  const { sendEditDatasetCommand } = useRosServiceCaller();

  // Redux state selectors
  const {
    mergeDatasetList,
    datasetToDeleteEpisode,
    deleteEpisodeNums,
    mergeOutputPath,
    mergeOutputFolderName,
  } = useSelector((state) => state.editDataset);

  // Local state
  const [isEditable] = useState(true);
  const [deleteEpisodeNumsInput, setDeleteEpisodeNumsInput] = useState('');
  const [showDatasetFileBrowserModal, setShowDatasetFileBrowserModal] = useState(false);
  const [showMergeOutputPathBrowserModal, setShowMergeOutputPathBrowserModal] = useState(false);
  const [selectingDatasetIndex, setSelectingDatasetIndex] = useState(null);
  const [showSelectDatasetPathBrowserModal, setShowSelectDatasetPathBrowserModal] = useState(false);

  // Effects
  useEffect(() => {
    manageTostLimit(toasts);
  }, [toasts]);

  // Event handlers
  const handlers = {
    mergeDatasetsChange: (newDatasets) => {
      dispatch(setMergeDatasetList(newDatasets));
    },

    datasetToDeleteEpisodeChange: (newDatasetToDelete) => {
      dispatch(setDatasetToDeleteEpisode(newDatasetToDelete));
    },

    deleteEpisodeNumsChange: (inputValue) => {
      setDeleteEpisodeNumsInput(inputValue);
      dispatch(setDeleteEpisodeNums(parseEpisodeNumbers(inputValue)));
    },

    datasetFileSelect: useCallback(
      (item) => {
        if (!isEditable) return;

        const updatedDatasets = [
          ...mergeDatasetList.slice(0, selectingDatasetIndex),
          item.full_path,
          ...mergeDatasetList.slice(selectingDatasetIndex + 1),
        ];

        dispatch(setMergeDatasetList(updatedDatasets));
        setShowDatasetFileBrowserModal(false);
      },
      [isEditable, mergeDatasetList, selectingDatasetIndex, dispatch]
    ),

    mergeOutputPathSelect: useCallback(
      (item) => {
        dispatch(setMergeOutputPath(item.full_path));
        setShowMergeOutputPathBrowserModal(false);
      },
      [dispatch]
    ),

    selectDatasetPathSelect: useCallback(
      (item) => {
        dispatch(setDatasetToDeleteEpisode(item.full_path));
        setShowSelectDatasetPathBrowserModal(false);
      },
      [dispatch]
    ),
  };

  // Async operations
  const operations = {
    deleteDataset: async () => {
      try {
        const result = await sendEditDatasetCommand('delete');
        console.log('Delete dataset result:', result);

        if (result?.success) {
          showOperationSuccess('delete', deleteEpisodeNums);
        } else {
          showOperationError('delete');
        }
      } catch (error) {
        console.error('Error deleting dataset:', error);
        showOperationError('delete', error.message);
      }
    },

    mergeDataset: async () => {
      try {
        // Check for duplicate datasets before merging
        const duplicateCheck = checkForDuplicateDatasets(mergeDatasetList);

        if (duplicateCheck.hasDuplicates) {
          showDuplicateWarning(duplicateCheck.duplicates);
          return; // Stop execution if duplicates are found
        }

        const result = await sendEditDatasetCommand('merge');
        console.log('Merge dataset result:', result);

        if (result?.success) {
          showOperationSuccess('merge');
        } else {
          showOperationError('merge');
        }
      } catch (error) {
        console.error('Error merging dataset:', error);
        showOperationError('merge', error.message);
      }
    },
  };

  // Calculate merge button state
  const duplicateCheck = useMemo(
    () => checkForDuplicateDatasets(mergeDatasetList),
    [mergeDatasetList]
  );
  const isMergeDisabled =
    mergeDatasetList.length < 2 ||
    mergeOutputPath === '' ||
    mergeOutputFolderName === '' ||
    !isEditable ||
    duplicateCheck.hasDuplicates;

  // Render sections
  const renderMergeSection = () => (
    <div className="w-full flex flex-col items-center justify-start bg-gray-100 p-10 gap-8 rounded-xl">
      <div className="w-full flex items-center justify-start">
        <span className="text-2xl font-bold mb-4">Merge Datasets</span>
      </div>
      <div className="w-full h-full flex flex-row items-center justify-start gap-8">
        <div className="w-full min-w-72 bg-white p-5 rounded-md flex flex-col items-start justify-center gap-2 shadow-md">
          <span className="text-xl font-bold">Enter Datasets to Merge</span>
          <DatasetListInput
            datasets={mergeDatasetList}
            onChange={handlers.mergeDatasetsChange}
            setShowDatasetFileBrowserModal={setShowDatasetFileBrowserModal}
            selectingDatasetIndex={selectingDatasetIndex}
            setSelectingDatasetIndex={setSelectingDatasetIndex}
          />
        </div>
        <div className="w-10 h-full flex flex-col items-center justify-center">
          <TbArrowMerge className="w-12 h-12 rotate-90" />
        </div>
        <div className="w-full min-w-72 bg-white p-5 rounded-md shadow-md">
          <div className="flex flex-col items-start justify-center gap-2">
            <span className="text-xl font-bold">Enter Output Path</span>
            <div className="flex flex-row items-center justify-start gap-2 w-full">
              <input
                className={clsx(STYLES.textInput, {
                  'bg-gray-100 cursor-not-allowed': !isEditable,
                  'bg-white': isEditable,
                })}
                type="text"
                placeholder="Enter output directory"
                value={mergeOutputPath || ''}
                onChange={(e) => dispatch(setMergeOutputPath(e.target.value))}
                disabled={!isEditable}
              />
              <button
                type="button"
                onClick={() => setShowMergeOutputPathBrowserModal(true)}
                className="flex items-center justify-center w-10 h-10 text-blue-500 bg-gray-200 rounded-md hover:text-blue-700"
                aria-label="Browse files for merge output path"
              >
                <MdFolderOpen className="w-8 h-8" />
              </button>
            </div>
            <input
              className={clsx(STYLES.textInput, {
                'bg-gray-100 cursor-not-allowed': !isEditable,
                'bg-white': isEditable,
              })}
              type="text"
              placeholder="Enter output folder name"
              value={mergeOutputFolderName || ''}
              onChange={(e) => dispatch(setMergeOutputFolderName(e.target.value))}
              disabled={!isEditable}
            />
            <div className="flex flex-row items-center justify-start gap-2 w-full">
              <span className="min-w-24 text-sm text-white font-bold bg-blue-400 py-1 px-2 rounded-full shadow-sm">
                Output path
              </span>
              <span className="text-sm text-blue-600">
                {/* Remove trailing slash from mergeOutputPath before displaying */}
                {(mergeOutputPath || '').replace(/\/$/, '')}/{mergeOutputFolderName}
              </span>
            </div>
          </div>
        </div>
      </div>
      {/* Duplicate Warning */}
      {duplicateCheck.hasDuplicates && (
        <div className="w-full p-4 bg-red-50 border border-red-200 rounded-lg">
          <div className="flex items-center gap-2 text-red-800 font-medium mb-2">
            <span className="text-lg">⚠️</span>
            Duplicate Datasets Detected
          </div>
          <div className="text-red-700 text-sm">
            {duplicateCheck.duplicates.map((dup, index) => (
              <div key={index} className="mb-1">
                <span className="font-mono bg-red-100 px-1 rounded">{dup.path}</span>
                <span className="text-red-600 ml-2">
                  (positions: {dup.indices.map((i) => i + 1).join(', ')})
                </span>
              </div>
            ))}
          </div>
          <div className="text-red-600 text-sm mt-2">Please remove duplicates before merging.</div>
        </div>
      )}
      <button
        className={STYLES.button}
        onClick={operations.mergeDataset}
        disabled={isMergeDisabled}
      >
        Merge
      </button>
    </div>
  );

  const renderDeleteSection = () => (
    <div className="w-full flex flex-col items-center justify-start bg-gray-100 p-10 gap-8 rounded-xl">
      <div className="w-full flex items-center justify-start">
        <h1 className="text-2xl font-bold mb-4">Delete Episodes from Dataset</h1>
      </div>

      <div className="flex flex-row items-center justify-start gap-20 w-full">
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
    </div>
  );

  return (
    <div className={STYLES.container}>
      <div className="w-full flex flex-col items-start justify-start p-10 gap-6">
        <h1 className="text-4xl font-bold flex flex-row items-center justify-start gap-2">
          <MdDataset className="w-10 h-10" />
          Edit Dataset
        </h1>
        {renderMergeSection()}
        {renderDeleteSection()}
      </div>

      <FileBrowserModal
        isOpen={showDatasetFileBrowserModal}
        onClose={() => setShowDatasetFileBrowserModal(false)}
        onFileSelect={handlers.datasetFileSelect}
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

      <FileBrowserModal
        isOpen={showMergeOutputPathBrowserModal}
        onClose={() => setShowMergeOutputPathBrowserModal(false)}
        onFileSelect={handlers.mergeOutputPathSelect}
        title="Select Merge Output Directory"
        selectButtonText="Select"
        allowDirectorySelect={true}
        allowFileSelect={false}
        initialPath={DEFAULT_PATHS.DATASET_PATH}
        defaultPath={DEFAULT_PATHS.DATASET_PATH}
        homePath=""
      />

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
}
