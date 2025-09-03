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
import { MdDataset } from 'react-icons/md';
import HuggingfaceSection from '../features/editDataset/components/DatasetHuggingfaceSection';
import MergeSection from '../features/editDataset/components/DatasetMergeSection';
import DeleteSection from '../features/editDataset/components/DatasetDeleteSection';

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
};

// Utility Functions
const manageTostLimit = (toasts) => {
  toasts
    .filter((t) => t.visible) // Only consider visible toasts
    .filter((_, i) => i >= TOAST_LIMIT) // Is toast index over limit?
    .forEach((t) => toast.dismiss(t.id)); // Dismiss
};

export default function EditDatasetPage() {
  // Hooks and state management
  const { toasts } = useToasterStore();

  // Local state
  const [isEditable] = useState(true);

  // Effects
  useEffect(() => {
    manageTostLimit(toasts);
  }, [toasts]);

  // Render sections
  const renderHuggingfaceSection = () => <HuggingfaceSection isEditable={isEditable} />;
  const renderMergeSection = () => <MergeSection isEditable={isEditable} />;
  const renderDeleteSection = () => <DeleteSection isEditable={isEditable} />;

  return (
    <div className={STYLES.container}>
      <div className="w-full flex flex-col items-start justify-start p-10 gap-6">
        <h1 className="text-4xl font-bold flex flex-row items-center justify-start gap-2">
          <MdDataset className="w-10 h-10" />
          Edit Dataset
        </h1>
        {renderHuggingfaceSection()}
        {renderMergeSection()}
        {renderDeleteSection()}
      </div>
    </div>
  );
}
