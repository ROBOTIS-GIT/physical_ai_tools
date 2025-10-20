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

import React, { useEffect } from 'react';
import { useSelector } from 'react-redux';
import clsx from 'clsx';
import toast, { useToasterStore } from 'react-hot-toast';
import HeartbeatStatus from '../components/HeartbeatStatus';
import DatasetSelector from '../components/DatasetSelector';
import PolicySelector from '../components/PolicySelector';
import TrainingOutputFolderInput from '../components/TrainingOutputFolderInput';
import TrainingControlPanel from '../components/TrainingControlPanel';
import TrainingOptionInput from '../components/TrainingOptionInput';
import TrainingProgressBar from '../components/TrainingProgressBar';
import TrainingLossDisplay from '../components/TrainingLossDisplay';
import ResumePolicySelector from '../components/ResumePolicySelector';

export default function TrainingPage() {
  const trainingMode = useSelector((state) => state.training.trainingMode);

  const classContainer = clsx(
    'w-full',
    'h-full',
    'flex',
    'flex-col',
    'items-start',
    'justify-start',
    'pt-10'
  );

  const classHeartbeatStatus = clsx('absolute', 'top-5', 'left-35', 'z-10');

  const classComponentsContainer = clsx(
    'w-full',
    'flex',
    'p-10',
    'gap-8',
    'items-start',
    'justify-center'
  );

  // Toast limit implementation using useToasterStore
  const { toasts } = useToasterStore();
  const TOAST_LIMIT = 3;

  useEffect(() => {
    toasts
      .filter((t) => t.visible) // Only consider visible toasts
      .filter((_, i) => i >= TOAST_LIMIT) // Is toast index over limit?
      .forEach((t) => toast.dismiss(t.id)); // Dismiss – Use toast.remove(t.id) for no exit animation
  }, [toasts]);

  const renderTrainingComponents = () => {
    if (trainingMode === 'resume') {
      return (
        <div className={classComponentsContainer}>
          <ResumePolicySelector />
          <DatasetSelector />
          <div className="flex flex-col items-start justify-center gap-5">
            <PolicySelector readonly={true} />
            <TrainingOutputFolderInput readonly={true} />
          </div>
          <TrainingOptionInput />
        </div>
      );
    } else {
      return (
        <div className={classComponentsContainer}>
          <DatasetSelector />
          <PolicySelector />
          <TrainingOutputFolderInput />
          <TrainingOptionInput />
        </div>
      );
    }
  };

  return (
    <div className={classContainer}>
      <div className={classHeartbeatStatus}>
        <HeartbeatStatus />
      </div>

      {/* Components based on selected mode */}
      <div className="overflow-scroll h-full w-full">
        <div>{renderTrainingComponents()}</div>
        <div className="flex justify-center items-center mt-5 mb-8">
          <div className="rounded-full bg-gray-200 w-32 h-3"></div>
        </div>
        <div className="flex justify-center items-center mb-10">
          <div className="w-full max-w-md">
            <TrainingLossDisplay />
          </div>
        </div>
      </div>

      {/* Training Control Buttons */}
      <div className="w-full flex items-center justify-around gap-2 bg-gray-100 p-2">
        <div className="flex-shrink-0">
          <TrainingControlPanel />
        </div>
        {/* Training Progress and Loss Display */}
        <div className="flex-1 min-w-0 max-w-4xl flex gap-10 justify-center items-center">
          <div className="flex-1 max-w-md">
            <TrainingProgressBar />
          </div>
        </div>
      </div>
    </div>
  );
}
