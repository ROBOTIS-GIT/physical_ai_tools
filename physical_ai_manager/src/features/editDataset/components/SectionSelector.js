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

import React from 'react';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import { MdOutlineFileUpload, MdOutlineFileDownload } from 'react-icons/md';

const SectionSelector = ({
  activeSection,
  onSectionChange,
  canChangeSection = true,
  className = '',
}) => {
  const handleSectionClick = (section) => {
    if (canChangeSection) {
      onSectionChange(section);
    } else if (!canChangeSection && activeSection !== section) {
      toast.error('Cannot switch sections while upload/download is in progress');
    }
  };

  return (
    <div className={clsx('w-full flex justify-center', className)}>
      <div className="flex bg-gray-200 rounded-lg p-1">
        <button
          className={clsx('px-4 py-2 text-sm font-medium rounded-md transition-all duration-200', {
            'bg-white text-blue-600 shadow-sm': activeSection === 'upload',
            'text-gray-600 hover:text-gray-800': activeSection !== 'upload',
            'cursor-not-allowed opacity-50': !canChangeSection && activeSection !== 'upload',
          })}
          onClick={() => handleSectionClick('upload')}
          disabled={!canChangeSection && activeSection !== 'upload'}
          aria-label="Switch to upload section"
        >
          <div className="flex items-center gap-2">
            <MdOutlineFileUpload className="w-4 h-4" />
            Upload
          </div>
        </button>
        <button
          className={clsx('px-4 py-2 text-sm font-medium rounded-md transition-all duration-200', {
            'bg-white text-blue-600 shadow-sm': activeSection === 'download',
            'text-gray-600 hover:text-gray-800': activeSection !== 'download',
            'cursor-not-allowed opacity-50': !canChangeSection && activeSection !== 'download',
          })}
          onClick={() => handleSectionClick('download')}
          disabled={!canChangeSection && activeSection !== 'download'}
          aria-label="Switch to download section"
        >
          <div className="flex items-center gap-2">
            <MdOutlineFileDownload className="w-4 h-4" />
            Download
          </div>
        </button>
      </div>
    </div>
  );
};

export default SectionSelector;
