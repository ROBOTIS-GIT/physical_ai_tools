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

import React, { useState, useCallback, useEffect, useMemo } from 'react';
import clsx from 'clsx';
import { useSelector, useDispatch } from 'react-redux';
import toast from 'react-hot-toast';
import { TbArrowMerge } from 'react-icons/tb';
import { MdFolderOpen, MdDataset } from 'react-icons/md';
import {
  setMergeDatasetList,
  setMergeOutputPath,
  setMergeOutputFolderName,
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
  deleteAllButton: clsx(
    'px-3',
    'py-1',
    'bg-red-500',
    'text-white',
    'rounded-md',
    'hover:bg-red-600',
    'focus:outline-none',
    'focus:ring-2',
    'focus:ring-red-500',
    'flex',
    'items-center',
    'gap-2',
    'text-sm',
    'font-medium'
  ),
};

// Utility Functions
const checkForDuplicateDatasets = (datasets) => {
  const normalizedPaths = datasets
    .filter((path) => path && path.trim() !== '') // Filter out empty paths
    .map((path) => path.trim().replace(/\/$/, '')); // Normalize paths by removing trailing slashes

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

const checkForEmptyDataset = (datasets) => {
  return datasets.some((dataset) => !dataset || dataset.trim() === '') || datasets.length === 0;
};

const checkFolderNameConflict = (folderName, existingFolders) => {
  if (!folderName || !folderName.trim()) return false;

  const normalizedFolderName = folderName.trim().toLowerCase();
  return existingFolders.some((folder) => folder.toLowerCase() === normalizedFolderName);
};

const showOperationSuccess = (operation) => {
  if (operation === 'merge') {
    toast.success('Dataset merged successfully!');
  }
};

const showOperationError = (operation, errorMessage = '') => {
  const operationText = 'merge';
  const message = errorMessage
    ? `Failed to ${operationText} dataset:\n${errorMessage}`
    : `Failed to ${operationText} dataset`;
  toast.error(message);
};

// DatasetListInput Component
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

    deleteAll: () => {
      if (localDatasets.length > 0) {
        setLocalDatasets([]);
        onChange([]);
      }
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
      <div className="max-h-48 overflow-y-auto border border-gray-300 rounded-md bg-white scrollbar-thin">
        <div className="p-2 space-y-2">{localDatasets.map(renderDatasetRow)}</div>
      </div>

      {!disabled && (
        <div className="mt-3 flex justify-between items-center">
          <div className="flex flex-row items-center justify-start gap-2">
            <button
              type="button"
              onClick={datasetActions.add}
              className={STYLES.addButton}
              aria-label="Add new dataset"
            >
              <span className="text-base font-bold">+</span>
              Add Dataset
            </button>
            <button
              type="button"
              onClick={datasetActions.deleteAll}
              className={STYLES.deleteAllButton}
              aria-label="Delete all datasets"
            >
              <span className="text-base font-bold">×</span>
              Delete All
            </button>
          </div>
          <div className="flex flex-row items-center justify-start gap-2 text-md text-green-600 mb-0.5 px-0 select-none">
            <MdDataset className="w-5 h-5 text-green-600" />
            {localDatasets.length}
          </div>
        </div>
      )}
    </div>
  );
};

const MergeSection = ({ isEditable = true }) => {
  const dispatch = useDispatch();
  const { mergeDatasetList, mergeOutputPath, mergeOutputFolderName } = useSelector(
    (state) => state.editDataset
  );
  const { sendEditDatasetCommand, browseFile } = useRosServiceCaller();

  // Local states
  const [showDatasetFileBrowserModal, setShowDatasetFileBrowserModal] = useState(false);
  const [showMergeOutputPathBrowserModal, setShowMergeOutputPathBrowserModal] = useState(false);
  const [selectingDatasetIndex, setSelectingDatasetIndex] = useState(null);
  const [existingFolders, setExistingFolders] = useState([]);

  // Function to fetch existing folders in the output path
  const fetchExistingFolders = useCallback(
    async (path) => {
      if (!path || !path.trim()) {
        setExistingFolders([]);
        return;
      }

      try {
        const result = await browseFile('browse', path.trim());
        if (result.success && result.items) {
          const folders = result.items.filter((item) => item.is_directory).map((item) => item.name);
          setExistingFolders(folders);
        } else {
          setExistingFolders([]);
        }
      } catch (error) {
        console.error('Failed to fetch existing folders:', error);
        setExistingFolders([]);
      }
    },
    [browseFile]
  );

  // Fetch existing folders when output path changes
  useEffect(() => {
    if (mergeOutputPath) {
      fetchExistingFolders(mergeOutputPath);
    } else {
      setExistingFolders([]);
    }
  }, [mergeOutputPath, fetchExistingFolders]);

  // Event handlers
  const handlers = {
    mergeDatasetsChange: (newDatasets) => {
      dispatch(setMergeDatasetList(newDatasets));
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
  };

  // Operations
  const operations = {
    mergeDataset: async () => {
      try {
        // Check for duplicate datasets before merging
        const duplicateCheck = checkForDuplicateDatasets(mergeDatasetList);

        if (duplicateCheck.hasDuplicates) {
          const duplicateList = duplicateCheck.duplicates
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
          return; // Stop execution if duplicates are found
        }

        // Check for folder name conflict
        const hasFolderConflict = checkFolderNameConflict(mergeOutputFolderName, existingFolders);

        if (hasFolderConflict) {
          toast.error(
            `Folder "${mergeOutputFolderName}" already exists in the output directory.\nPlease choose a different folder name.`,
            {
              duration: 5000,
              style: {
                maxWidth: '400px',
                whiteSpace: 'pre-line',
              },
            }
          );
          return; // Stop execution if folder name conflicts
        }

        const result = await sendEditDatasetCommand('merge');
        console.log('Merge dataset result:', result);

        if (result?.success) {
          showOperationSuccess('merge');
          dispatch(setMergeOutputPath(''));
          dispatch(setMergeOutputFolderName(''));
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
  const hasEmptyDatasets = useMemo(
    () => checkForEmptyDataset(mergeDatasetList),
    [mergeDatasetList]
  );
  const hasFolderConflict = useMemo(
    () => checkFolderNameConflict(mergeOutputFolderName, existingFolders),
    [mergeOutputFolderName, existingFolders]
  );
  const isMergeDisabled =
    mergeDatasetList.length < 2 ||
    mergeOutputPath === '' ||
    mergeOutputFolderName === '' ||
    !isEditable ||
    duplicateCheck.hasDuplicates ||
    hasEmptyDatasets ||
    hasFolderConflict;

  return (
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
            disabled={!isEditable}
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
              <span className="text-sm text-blue-600 break-all">
                {/* Remove trailing slash from mergeOutputPath before displaying */}
                {(mergeOutputPath || '').replace(/\/$/, '')}/{mergeOutputFolderName}
              </span>
            </div>
          </div>
        </div>
      </div>

      {/* Empty Dataset Warning */}
      {hasEmptyDatasets && (
        <div className="w-full p-4 bg-yellow-50 border border-yellow-200 rounded-lg">
          <div className="flex items-center gap-2 text-yellow-800 font-medium mb-2">
            <span className="text-lg">⚠️</span>
            Empty Dataset Paths Detected
          </div>
          <div className="text-yellow-700 text-sm">
            {mergeDatasetList.map(
              (dataset, index) =>
                (!dataset || dataset.trim() === '') && (
                  <div key={index} className="mb-1">
                    <span className="font-medium">Position {index + 1}:</span>
                    <span className="ml-2 text-yellow-600">Empty dataset path</span>
                  </div>
                )
            )}
          </div>
          <div className="text-yellow-600 text-sm mt-2">
            Please fill in all dataset paths before merging.
          </div>
        </div>
      )}

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

      {/* Folder Name Conflict Warning */}
      {hasFolderConflict && (
        <div className="w-full p-4 bg-orange-50 border border-orange-200 rounded-lg">
          <div className="flex items-center gap-2 text-orange-800 font-medium mb-2">
            <span className="text-lg">📁</span>
            Folder Name Conflict
          </div>
          <div className="text-orange-700 text-sm">
            <span className="font-mono bg-orange-100 px-1 rounded">{mergeOutputFolderName}</span>
            <span className="ml-2">already exists in the output directory</span>
          </div>
          <div className="text-orange-600 text-sm mt-2">
            Please choose a different folder name to avoid overwriting existing data.
          </div>
          {existingFolders.length > 0 && (
            <div className="text-orange-600 text-sm mt-2">
              <span className="font-medium">Existing folders:</span>
              <div className="mt-1 flex flex-wrap gap-1">
                {existingFolders.slice(0, 10).map((folder, index) => (
                  <span key={index} className="font-mono bg-orange-100 px-1 rounded text-xs">
                    {folder}
                  </span>
                ))}
                {existingFolders.length > 10 && (
                  <span className="text-xs text-orange-500">
                    +{existingFolders.length - 10} more...
                  </span>
                )}
              </div>
            </div>
          )}
        </div>
      )}

      <button
        className={STYLES.button}
        onClick={operations.mergeDataset}
        disabled={isMergeDisabled}
      >
        Merge
      </button>

      {/* File Browser Modals */}
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
    </div>
  );
};

export default MergeSection;
