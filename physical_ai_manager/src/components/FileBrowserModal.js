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

import React, { useState, useCallback } from 'react';
import clsx from 'clsx';
import { MdClose, MdFolderOpen } from 'react-icons/md';
import FileBrowser from './FileBrowser';

export default function FileBrowserModal({
  isOpen,
  onClose,
  onFileSelect,
  initialPath = '',
  fileFilter = null,
  title = 'Select File',
  selectButtonText = 'Select',
  allowDirectorySelect = false,
  allowFileSelect = true,
  targetFileName = null,
  targetFolderName = null,
  targetFileLabel = null,
  homePath = null,
  defaultPath = null,
}) {
  const [selectedItem, setSelectedItem] = useState(null);
  const [currentPath, setCurrentPath] = useState(initialPath);

  const handleFileSelect = useCallback((item) => {
    setSelectedItem(item);
  }, []);

  const handlePathChange = useCallback((path) => {
    setCurrentPath(path);
    // Clear selection when path changes (navigation)
    setSelectedItem(null);
  }, []);

  const handleConfirm = useCallback(() => {
    if (selectedItem) {
      onFileSelect(selectedItem);
      onClose();
      setSelectedItem(null);
    } else if (allowDirectorySelect && currentPath && !targetFileName && !targetFolderName) {
      // If directory selection is allowed and no file is selected, select current directory
      // But only when targetFileName is not specified
      onFileSelect({
        name: currentPath.split('/').pop() || currentPath,
        full_path: currentPath,
        is_directory: true,
        size: -1,
        modified_time: '',
      });
      onClose();
    }
  }, [
    selectedItem,
    currentPath,
    allowDirectorySelect,
    targetFileName,
    targetFolderName,
    onFileSelect,
    onClose,
  ]);

  const handleCancel = useCallback(() => {
    onClose();
    setSelectedItem(null);
  }, [onClose]);

  if (!isOpen) return null;

  const classOverlay = clsx('fixed', 'inset-0', 'z-50', 'overflow-y-auto');

  const classBackdrop = clsx('fixed', 'inset-0', 'bg-black', 'bg-opacity-50', 'transition-opacity');

  const classModalContainer = clsx('flex', 'min-h-full', 'items-center', 'justify-center', 'p-4');

  const classModal = clsx(
    'relative',
    'bg-white',
    'rounded-lg',
    'shadow-xl',
    'max-w-4xl',
    'w-full',
    'max-h-[90vh]',
    'flex',
    'flex-col'
  );

  const classHeader = clsx(
    'flex',
    'items-center',
    'justify-between',
    'px-6',
    'py-4',
    'border-b',
    'border-gray-200'
  );

  const classTitle = clsx('text-xl', 'font-semibold', 'text-gray-900');

  const classCloseButton = clsx(
    'p-2',
    'text-gray-400',
    'hover:text-gray-600',
    'hover:bg-gray-100',
    'rounded-lg',
    'transition-colors'
  );

  const classBrowserContent = clsx('flex-1 h-full');

  const classBrowserStyles = clsx('border-0', 'rounded-none', 'h-full', 'max-h-[60vh]');

  const classFooter = clsx(
    'flex',
    'items-center',
    'justify-between',
    'px-6',
    'py-4',
    'border-t',
    'border-gray-200',
    'bg-gray-50'
  );

  const classStatusContainer = clsx('flex', 'items-center', 'text-sm', 'text-gray-600');

  const classStatusRow = clsx('flex', 'items-center');

  const classIcon = clsx('w-4', 'h-4', 'mr-2');

  const classLabel = clsx('font-medium');

  const classValue = clsx('ml-2', 'font-mono', 'break-all');

  const classButtonContainer = clsx('flex', 'items-center', 'space-x-3');

  const classCancelButton = clsx(
    'px-4',
    'py-2',
    'text-gray-700',
    'bg-white',
    'border',
    'border-gray-300',
    'rounded-lg',
    'hover:bg-gray-50',
    'transition-colors'
  );

  const classConfirmButton = clsx(
    'px-4',
    'py-2',
    'bg-blue-600',
    'text-white',
    'rounded-lg',
    'hover:bg-blue-700',
    'disabled:bg-gray-300',
    'disabled:cursor-not-allowed',
    'transition-colors'
  );

  return (
    <div className={classOverlay}>
      <div className={classBackdrop} />

      <div className={classModalContainer}>
        <div className={classModal}>
          <div className={classHeader}>
            <h2 className={classTitle}>{title}</h2>
            <button onClick={handleCancel} className={classCloseButton}>
              <MdClose size={24} />
            </button>
          </div>

          <div className={classBrowserContent}>
            <FileBrowser
              onFileSelect={handleFileSelect}
              onPathChange={handlePathChange}
              initialPath={initialPath}
              fileFilter={fileFilter}
              className={classBrowserStyles}
              title=""
              targetFileName={targetFileName}
              targetFolderName={targetFolderName}
              onDirectorySelect={handleFileSelect}
              targetFileLabel={targetFileLabel}
              homePath={homePath}
              defaultPath={defaultPath}
              allowDirectorySelect={allowDirectorySelect}
              allowFileSelect={allowFileSelect}
            />
          </div>

          <div className={classFooter}>
            <div className={classStatusContainer}>
              {selectedItem ? (
                <div className={classStatusRow}>
                  <MdFolderOpen className={classIcon} />
                  <span className={classLabel}>Selected:</span>
                  <span className={classValue}>{selectedItem.name}</span>
                </div>
              ) : allowDirectorySelect && currentPath && !targetFileName && !targetFolderName ? (
                <div className={classStatusRow}>
                  <MdFolderOpen className={classIcon} />
                  <span className={classLabel}>Current Directory:</span>
                  <span className={classValue}>{currentPath}</span>
                </div>
              ) : (
                <span>
                  {targetFileName || targetFolderName
                    ? `Select a directory containing ${targetFileName || targetFolderName}`
                    : allowDirectorySelect && allowFileSelect
                    ? 'Select a file or folder, or use current directory'
                    : allowDirectorySelect
                    ? 'Select a folder or use current directory'
                    : allowFileSelect
                    ? 'Select a file to continue'
                    : 'Navigation only'}
                </span>
              )}
            </div>

            <div className={classButtonContainer}>
              <button onClick={handleCancel} className={classCancelButton}>
                Cancel
              </button>
              <button
                onClick={handleConfirm}
                disabled={
                  targetFileName || targetFolderName
                    ? !(selectedItem && selectedItem.is_directory)
                    : (!selectedItem && !(allowDirectorySelect && currentPath)) ||
                      (selectedItem && selectedItem.is_directory && !allowDirectorySelect) ||
                      (selectedItem && !selectedItem.is_directory && !allowFileSelect)
                }
                className={classConfirmButton}
              >
                {selectButtonText}
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
