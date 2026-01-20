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

import React, { useCallback, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import { MdFolderOpen, MdClear, MdDownload } from 'react-icons/md';
import { setDatasetRepoId } from '../features/training/trainingSlice';
import FileBrowserModal from './FileBrowserModal';
import DatasetDownloadModal from './DatasetDownloadModal';
import { DEFAULT_PATHS, TARGET_FOLDERS } from '../constants/paths';

export default function DatasetSelector() {
  const dispatch = useDispatch();

  const datasetRepoId = useSelector((state) => state.training.trainingInfo.datasetRepoId);
  const isTraining = useSelector((state) => state.training.isTraining);

  const [showDatasetBrowserModal, setShowDatasetBrowserModal] = useState(false);
  const [showDatasetDownloadModal, setShowDatasetDownloadModal] = useState(false);

  // Handle dataset path selection from file browser
  const handleDatasetPathSelect = useCallback(
    (item) => {
      if (item && item.full_path) {
        dispatch(setDatasetRepoId(item.full_path));

        // Truncate long path for toast display
        const fullPath = item.full_path;
        const maxLength = 50;
        let displayPath = fullPath;

        if (fullPath.length > maxLength) {
          const start = fullPath.substring(0, 20);
          const end = fullPath.substring(fullPath.length - 25);
          displayPath = `${start}...${end}`;
        }

        toast.success(`Dataset selected:\n${displayPath}`);
      }
    },
    [dispatch]
  );

  // Handle download complete - set the downloaded path as selected dataset
  const handleDownloadComplete = useCallback(
    (downloadedPath) => {
      dispatch(setDatasetRepoId(downloadedPath));
      toast.success(`Dataset downloaded and selected:\n${downloadedPath}`);
    },
    [dispatch]
  );

  // Clear selected dataset
  const handleClearDataset = useCallback(() => {
    dispatch(setDatasetRepoId(undefined));
    toast.success('Dataset selection cleared');
  }, [dispatch]);

  const classCard = clsx(
    'bg-white',
    'border',
    'border-gray-200',
    'rounded-2xl',
    'shadow-lg',
    'p-6',
    'w-full',
    'max-w-lg'
  );

  const classTitle = clsx('text-xl', 'font-bold', 'text-gray-800', 'mb-6', 'text-left');

  const classButtonContainer = clsx('flex', 'flex-col', 'gap-2');

  const classBrowseButton = clsx(
    'w-full',
    'px-4',
    'py-3',
    'bg-blue-50',
    'text-blue-700',
    'border',
    'border-blue-200',
    'rounded-lg',
    'font-medium',
    'transition-colors',
    'hover:bg-blue-100',
    'disabled:bg-gray-100',
    'disabled:text-gray-400',
    'disabled:border-gray-200',
    'disabled:cursor-not-allowed',
    'flex',
    'items-center',
    'justify-center',
    'gap-2'
  );

  const classDownloadButton = clsx(
    'w-full',
    'px-4',
    'py-3',
    'bg-green-50',
    'text-green-700',
    'border',
    'border-green-200',
    'rounded-lg',
    'font-medium',
    'transition-colors',
    'hover:bg-green-100',
    'disabled:bg-gray-100',
    'disabled:text-gray-400',
    'disabled:border-gray-200',
    'disabled:cursor-not-allowed',
    'flex',
    'items-center',
    'justify-center',
    'gap-2'
  );

  const classCurrentSelection = clsx(
    'mt-4',
    'p-4',
    'bg-gray-50',
    'border',
    'border-gray-200',
    'rounded-lg'
  );

  const classSelectionLabel = clsx('text-sm', 'font-medium', 'text-gray-600', 'mb-2');

  const classSelectionPath = clsx(
    'text-sm',
    'text-blue-600',
    'font-mono',
    'break-all',
    'bg-white',
    'p-2',
    'rounded',
    'border',
    'border-gray-200'
  );

  const classClearButton = clsx(
    'mt-3',
    'px-3',
    'py-1.5',
    'text-sm',
    'text-gray-600',
    'bg-white',
    'border',
    'border-gray-300',
    'rounded-md',
    'hover:bg-gray-50',
    'hover:text-red-600',
    'hover:border-red-300',
    'transition-colors',
    'flex',
    'items-center',
    'gap-1'
  );

  const classEmptyState = clsx(
    'mt-4',
    'p-6',
    'bg-gray-50',
    'border',
    'border-dashed',
    'border-gray-300',
    'rounded-lg',
    'text-center',
    'text-gray-500'
  );

  return (
    <div className={classCard}>
      <h1 className={classTitle}>Dataset Selection</h1>

      {/* Button Container */}
      <div className={classButtonContainer}>
        {/* Browse Dataset Path Button */}
        <button
          className={classBrowseButton}
          onClick={() => setShowDatasetBrowserModal(true)}
          disabled={isTraining}
        >
          <MdFolderOpen size={20} />
          Browse Dataset Path
        </button>

        {/* Download Dataset Button */}
        <button
          className={classDownloadButton}
          onClick={() => setShowDatasetDownloadModal(true)}
          disabled={isTraining}
        >
          <MdDownload size={20} />
          Download Dataset
        </button>
      </div>

      {/* Current Selection Display */}
      {datasetRepoId ? (
        <div className={classCurrentSelection}>
          <div className={classSelectionLabel}>Selected Dataset:</div>
          <div className={classSelectionPath}>{datasetRepoId}</div>
          <button
            className={classClearButton}
            onClick={handleClearDataset}
            disabled={isTraining}
          >
            <MdClear size={16} />
            Clear Selection
          </button>
        </div>
      ) : (
        <div className={classEmptyState}>
          <MdFolderOpen size={32} className="mx-auto mb-2 text-gray-400" />
          <p>No dataset selected</p>
          <p className="text-sm mt-1">Browse local path or download from Hugging Face</p>
        </div>
      )}

      {/* File Browser Modal */}
      <FileBrowserModal
        isOpen={showDatasetBrowserModal}
        onClose={() => setShowDatasetBrowserModal(false)}
        onFileSelect={handleDatasetPathSelect}
        title="Select Dataset Path"
        selectButtonText="Select"
        allowDirectorySelect={false}
        targetFolderName={[
          TARGET_FOLDERS.DATASET_METADATA,
          TARGET_FOLDERS.DATASET_VIDEO,
          TARGET_FOLDERS.DATASET_DATA,
        ]}
        targetFileLabel="Dataset folder found!"
        initialPath={DEFAULT_PATHS.DATASET_PATH}
        defaultPath={DEFAULT_PATHS.DATASET_PATH}
        homePath=""
      />

      {/* Dataset Download Modal */}
      <DatasetDownloadModal
        isOpen={showDatasetDownloadModal}
        onClose={() => setShowDatasetDownloadModal(false)}
        onDownloadComplete={handleDownloadComplete}
      />
    </div>
  );
}
