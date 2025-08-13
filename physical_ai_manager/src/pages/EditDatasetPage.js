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

import React, { useEffect, useState } from 'react';
import clsx from 'clsx';
import toast, { useToasterStore } from 'react-hot-toast';
import { useSelector, useDispatch } from 'react-redux';
import { setMergeDatasetList, setDatasetToDelete } from '../features/editDataset/editDatasetSlice';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';

const DatasetListInput = ({ datasets = [''], onChange, disabled, className }) => {
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

  const classDatasetTextarea = clsx(
    'w-full',
    'p-2',
    'border',
    'border-gray-200',
    'rounded',
    'text-sm',
    'resize-none',
    'min-h-8',
    'h-14',
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
            <div key={index} className="relative">
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
          ))}
        </div>
      </div>

      {!disabled && (
        <div className="mt-3 flex justify-between items-center">
          <button type="button" onClick={addDataset} className={classAddDatasetButton}>
            <span className="text-base font-bold">+</span>
            Add Dataset
          </button>
          <span className="block text-xs text-gray-600 mb-0.5 px-0 select-none">
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

export default function EditDatasetPage() {
  // Toast limit implementation using useToasterStore
  const { toasts } = useToasterStore();
  const TOAST_LIMIT = 3;

  const dispatch = useDispatch();

  const mergeDatasetList = useSelector((state) => state.editDataset.mergeDatasetList);
  const datasetToDelete = useSelector((state) => state.editDataset.datasetToDelete);

  const { sendEditDatasetCommand } = useRosServiceCaller();

  const [isEditable] = useState(true);

  useEffect(() => {
    toasts
      .filter((t) => t.visible) // Only consider visible toasts
      .filter((_, i) => i >= TOAST_LIMIT) // Is toast index over limit?
      .forEach((t) => toast.dismiss(t.id)); // Dismiss – Use toast.remove(t.id) for no exit animation
  }, [toasts]);

  const handleMergeDatasetListChange = (newDatasets) => {
    dispatch(setMergeDatasetList(newDatasets));
  };

  const handleDatasetToDeleteChange = (newDatasets) => {
    dispatch(setDatasetToDelete(newDatasets));
  };

  const handleDeleteDataset = async () => {
    try {
      const result = await sendEditDatasetCommand('delete');
      console.log('sendEditDatasetCommand result:', result);

      if (result && result.success) {
        toast.success('Dataset deleted successfully!');
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

  return (
    <div className="w-full h-full flex flex-col items-start justify-start pt-10">
      <div className="w-full h-full flex flex-col items-start justify-start pt-10">
        <div className="w-full h-full flex flex-col items-start justify-start p-10 gap-8">
          <div className="w-full flex flex-col items-start justify-start">
            <h1 className="text-2xl font-bold">Edit Dataset</h1>
            <div className="w-full flex flex-col items-center justify-start bg-gray-100 p-10">
              <span className="text-2xl font-bold">Merge Dataset</span>
              <DatasetListInput
                datasets={mergeDatasetList}
                onChange={handleMergeDatasetListChange}
              />
              <button className={classMergeButton} onClick={handleMergeDataset}>
                Merge
              </button>
            </div>
          </div>
          <div className="w-full flex flex-col items-center justify-start bg-gray-100 p-10">
            <h1 className="text-2xl font-bold">Delete Dataset</h1>
            <textarea
              className={classDatasetToDeleteTextarea}
              value={datasetToDelete}
              onChange={(e) => handleDatasetToDeleteChange(e.target.value)}
              disabled={false}
              placeholder="Enter Dataset to Delete"
            />
            <button className={classDeleteButton} onClick={handleDeleteDataset}>
              Delete
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}
