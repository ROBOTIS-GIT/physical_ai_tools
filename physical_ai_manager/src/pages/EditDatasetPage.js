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
import { DEFAULT_PATHS, TARGET_FILES } from '../constants/paths';
import FileBrowserModal from '../components/FileBrowserModal';

import {
  setMergeDatasetList,
  setDatasetToDelete,
  setDeleteEpisodeNums,
  setMergeOutputPath,
} from '../features/editDataset/editDatasetSlice';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';

const DatasetListInput = ({
  datasets = [''],
  onChange,
  disabled,
  className,
  setShowDatasetFileBrowserModal,
  selectingDatasetIndex,
  setSelectingDatasetIndex,
}) => {
  const [localDatasets, setLocalDatasets] = useState(() => {
    // Ensure instructions is always an array
    if (Array.isArray(datasets) && datasets.length > 0) {
      return datasets;
    }
    return [''];
  });

  useEffect(() => {
    // Add proper type checking to ensure instructions is an array
    if (Array.isArray(datasets)) {
      setLocalDatasets(datasets.length > 0 ? datasets : ['']);
    } else {
      setLocalDatasets(['']);
    }
  }, [datasets]);

  const addDataset = () => {
    const newDatasets = [...localDatasets, ''];
    setLocalDatasets(newDatasets);
    onChange(newDatasets);
  };

  const removeDataset = (index) => {
    if (localDatasets.length > 1) {
      const newDatasets = localDatasets.filter((_, i) => i !== index);
      setLocalDatasets(newDatasets);
      onChange(newDatasets);
    }
  };

  const updateDataset = (index, value) => {
    const newDatasets = [...localDatasets];
    newDatasets[index] = value;
    setLocalDatasets(newDatasets);
    onChange(newDatasets);
  };

  const selectDatasetUsingFileBrowser = (index) => {
    setSelectingDatasetIndex(index);
    setShowDatasetFileBrowserModal(true);
  };

  const classDatasetTextarea = clsx(
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
  );

  const classRemoveDatasetButton = clsx(
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
  );

  const classAddDatasetButton = clsx(
    'px-3',
    'py-1',
    'bg-blue-500',
    'text-white',
    'rounded',
    'hover:bg-blue-600',
    'focus:outline-none',
    'focus:ring-2',
    'focus:ring-blue-500',
    'flex',
    'items-center',
    'gap-2',
    'text-sm',
    'font-medium'
  );

  return (
    <div className={clsx('w-full', className)}>
      <div className="max-h-48 overflow-y-auto border border-gray-300 rounded-md bg-white">
        <div className="p-2 space-y-2">
          {localDatasets.map((dataset, index) => (
            <div className="flex flex-row items-center justify-start gap-2 w-full">
              <div key={index} className="relative w-full">
                <textarea
                  value={dataset}
                  onChange={(e) => updateDataset(index, e.target.value)}
                  disabled={disabled}
                  placeholder={`Dataset ${index + 1}`}
                  className={clsx(classDatasetTextarea, {
                    'bg-gray-100 cursor-not-allowed': disabled,
                    'bg-white': !disabled,
                    'pr-10': !disabled && localDatasets.length > 1,
                  })}
                  rows={2}
                />
                {!disabled && localDatasets.length > 1 && (
                  <button
                    type="button"
                    onClick={() => removeDataset(index)}
                    className={classRemoveDatasetButton}
                  >
                    ×
                  </button>
                )}
              </div>
              <button
                type="button"
                onClick={() => selectDatasetUsingFileBrowser(index)}
                className="flex items-center justify-center w-10 h-10 text-blue-500 bg-gray-200 rounded-md hover:text-blue-700"
              >
                <MdFolderOpen className="w-8 h-8" />
              </button>
            </div>
          ))}
        </div>
      </div>

      {!disabled && (
        <div className="mt-3 flex justify-between items-center">
          <button type="button" onClick={addDataset} className={classAddDatasetButton}>
            <span className="text-base font-bold">+</span>
            Add Dataset
          </button>
          <span className="block text-md text-gray-600 mb-0.5 px-0 select-none">
            <span role="img" aria-label="task">
              📝
            </span>{' '}
            {localDatasets.length}
          </span>
        </div>
      )}
    </div>
  );
};

const EpisodeNumberInput = ({ value, onChange, disabled = false, className, parseFunction }) => {
  const parsedNumbers = useMemo(() => {
    return parseFunction(value);
  }, [value, parseFunction]);

  return (
    <div className="flex flex-col gap-2 w-full">
      <input
        className={className}
        type="text"
        placeholder="Enter episode numbers to delete (e.g., 1,2,3,10-15,20)"
        value={value}
        onChange={(e) => onChange(e.target.value)}
        disabled={disabled}
      />
      {value && (
        <div className="text-sm text-gray-600">
          Preview: {parsedNumbers.length > 0 ? parsedNumbers.join(', ') : 'No valid episodes'} (
          {parsedNumbers.length} episodes)
        </div>
      )}
    </div>
  );
};

export default function EditDatasetPage() {
  // Toast limit implementation using useToasterStore
  const { toasts } = useToasterStore();
  const TOAST_LIMIT = 3;

  // Helper function to parse episode numbers from string
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

  const dispatch = useDispatch();

  const mergeDatasetList = useSelector((state) => state.editDataset.mergeDatasetList);
  const datasetToDelete = useSelector((state) => state.editDataset.datasetToDelete);
  const deleteEpisodeNums = useSelector((state) => state.editDataset.deleteEpisodeNums);
  const mergeOutputPath = useSelector((state) => state.editDataset.mergeOutputPath);

  const { sendEditDatasetCommand } = useRosServiceCaller();

  const [isEditable] = useState(true);
  const [deleteEpisodeNumsInput, setDeleteEpisodeNumsInput] = useState('');
  const [showDatasetFileBrowserModal, setShowDatasetFileBrowserModal] = useState(false);
  const [selectingDatasetIndex, setSelectingDatasetIndex] = useState(null);

  useEffect(() => {
    toasts
      .filter((t) => t.visible) // Only consider visible toasts
      .filter((_, i) => i >= TOAST_LIMIT) // Is toast index over limit?
      .forEach((t) => toast.dismiss(t.id)); // Dismiss – Use toast.remove(t.id) for no exit animation
  }, [toasts]);

  const handleMergeDatasetListChange = (newDatasets) => {
    dispatch(setMergeDatasetList(newDatasets));
  };

  const handleDatasetToDeleteChange = (newDatasetToDelete) => {
    dispatch(setDatasetToDelete(newDatasetToDelete));
  };

  const handleDeleteEpisodeNumsChange = (inputValue) => {
    setDeleteEpisodeNumsInput(inputValue);
    // parsedEpisodeNums will automatically update via useMemo
    dispatch(setDeleteEpisodeNums(parseEpisodeNumbers(inputValue)));
  };

  const handleDeleteDataset = async () => {
    try {
      const result = await sendEditDatasetCommand('delete');
      console.log('sendEditDatasetCommand result:', result);

      if (result && result.success) {
        const episodeText =
          deleteEpisodeNums.length > 0 ? ` (Episodes: ${deleteEpisodeNums.join(', ')})` : '';
        toast.success(`Dataset deleted successfully!${episodeText}`);
      } else {
        toast.error('Failed to delete dataset');
      }
    } catch (error) {
      console.error('Error deleting dataset:', error);
      toast.error(`Failed to delete dataset: ${error.message}`);
    }
  };

  const handleMergeDataset = async () => {
    try {
      const result = await sendEditDatasetCommand('merge');
      console.log('sendEditDatasetCommand result:', result);

      if (result && result.success) {
        toast.success('Dataset merged successfully!');
      } else {
        toast.error('Failed to merge dataset');
      }
    } catch (error) {
      console.error('Error merging dataset:', error);
      toast.error(`Failed to merge dataset: ${error.message}`);
    }
  };

  const handleDatasetFileSelect = useCallback(
    (item) => {
      if (!isEditable) return;
      console.log('selectingDatasetIndex:', selectingDatasetIndex);
      console.log('item:', item);
      handleMergeDatasetListChange([
        ...mergeDatasetList.slice(0, selectingDatasetIndex),
        item.full_path,
        ...mergeDatasetList.slice(selectingDatasetIndex + 1),
      ]);
      setShowDatasetFileBrowserModal(false);
    },
    [isEditable, mergeDatasetList, selectingDatasetIndex]
  );

  const classContainer = clsx(
    'w-full',
    'h-full',
    'flex',
    'flex-col',
    'items-start',
    'justify-start',
    'pt-10',
    'overflow-scroll'
  );

  const classMergeButton = clsx(
    'p-3',
    'm-5',
    'bg-blue-500',
    'text-white',
    'rounded',
    'hover:bg-blue-600',
    'focus:outline-none',
    'focus:ring-2',
    'focus:ring-blue-500',
    'flex',
    'items-center',
    'gap-5',
    'text-sm',
    'font-medium'
  );

  const classDeleteButton = classMergeButton;

  const classDatasetToDeleteTextarea = clsx(
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
    'focus:border-transparent',
    {
      'bg-gray-100 cursor-not-allowed': !isEditable,
      'bg-white': isEditable,
    }
  );

  const classTextInput = clsx(
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
    'focus:border-transparent',
    {
      'bg-gray-100 cursor-not-allowed': !isEditable,
      'bg-white': isEditable,
    }
  );

  return (
    <div className={classContainer}>
      <div className="w-full h-full flex flex-col items-start justify-start p-10 gap-8">
        <div className="w-full flex flex-col items-center justify-start">
          <h1 className="text-3xl font-bold m-5">Edit Dataset</h1>
          <div className="w-full flex flex-col items-center justify-start bg-gray-100 p-10 gap-8">
            <span className="text-2xl font-bold">Merge Dataset</span>
            <div className="w-full h-full flex flex-row items-center justify-start gap-8">
              <div className="w-full min-w-72 bg-white p-5 rounded-md flex flex-col items-start justify-center gap-2 shadow-md">
                <span className="text-xl font-bold">Enter Datasets to Merge</span>
                <DatasetListInput
                  datasets={mergeDatasetList}
                  onChange={handleMergeDatasetListChange}
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
                  <input
                    className={classTextInput}
                    type="text"
                    placeholder="Enter output path"
                    value={mergeOutputPath || ''}
                    onChange={(e) => dispatch(setMergeOutputPath(e.target.value))}
                    disabled={!isEditable}
                  />
                </div>
              </div>
            </div>
            <button className={classMergeButton} onClick={handleMergeDataset}>
              Merge
            </button>
          </div>
        </div>
        <div className="w-full flex flex-col items-center justify-start bg-gray-100 p-10 gap-8">
          <h1 className="text-2xl font-bold">Delete Dataset</h1>
          <textarea
            className={classDatasetToDeleteTextarea}
            value={datasetToDelete}
            onChange={(e) => handleDatasetToDeleteChange(e.target.value)}
            disabled={false}
            placeholder="Enter Dataset to Delete"
          />
          <EpisodeNumberInput
            value={deleteEpisodeNumsInput}
            onChange={handleDeleteEpisodeNumsChange}
            disabled={!isEditable}
            className={classTextInput}
            parseFunction={parseEpisodeNumbers}
          />
          <button className={classDeleteButton} onClick={handleDeleteDataset}>
            Delete
          </button>
        </div>
      </div>

      <FileBrowserModal
        isOpen={showDatasetFileBrowserModal}
        onClose={() => setShowDatasetFileBrowserModal(false)}
        onFileSelect={handleDatasetFileSelect}
        title="Select Policy Path"
        selectButtonText="Select"
        allowDirectorySelect={true}
        targetFileName={TARGET_FILES.POLICY_MODEL}
        targetFileLabel="Policy file found! 🎯"
        initialPath={DEFAULT_PATHS.POLICY_MODEL_PATH}
        defaultPath={DEFAULT_PATHS.POLICY_MODEL_PATH}
        homePath=""
      />
    </div>
  );
}
