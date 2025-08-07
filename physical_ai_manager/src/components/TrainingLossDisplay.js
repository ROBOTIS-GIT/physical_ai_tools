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
import { useSelector } from 'react-redux';
import clsx from 'clsx';

export default function TrainingLossDisplay() {
  const currentLoss = useSelector((state) => state.training.currentLoss);
  const currentStep = useSelector((state) => state.training.currentStep);
  const isTraining = useSelector((state) => state.training.isTraining);

  // Format loss value for display
  const formatLoss = (loss) => {
    if (loss === null || loss === undefined || Number.isNaN(loss)) return 'N/A';
    if (typeof loss !== 'number') return 'N/A';

    if (loss === 0) {
      return loss.toFixed(3);
    }

    // Format to appropriate decimal places
    if (loss >= 1) {
      return loss.toFixed(3);
    } else if (loss >= 0.01) {
      return loss.toFixed(4);
    } else {
      return loss.toExponential(2);
    }
  };

  // Format step number with commas
  const formatStep = (step) => {
    return step.toLocaleString();
  };

  const classContainer = clsx(
    'bg-white',
    'border',
    'border-gray-200',
    'rounded-2xl',
    'shadow-lg',
    'p-6',
    'w-full',
    'max-w-lg'
  );

  const classHeader = clsx(
    'text-xl',
    'font-semibold',
    'text-gray-700',
    'mb-2',
    'flex',
    'justify-between',
    'items-center'
  );

  const classLossContainer = clsx(
    'w-full',
    'bg-gradient-to-r',
    'from-orange-50',
    'to-red-50',
    'border',
    'border-orange-200',
    'rounded-lg',
    'p-3',
    'relative'
  );

  const classLossValue = clsx(
    'text-2xl',
    'font-bold',
    currentLoss !== null ? 'text-orange-600' : 'text-gray-400',
    'text-center'
  );

  const classStepInfo = clsx('text-sm', 'text-gray-500', 'text-center', 'mt-1');

  const classStatus = clsx('absolute', 'top-2', 'right-2', 'flex', 'items-center', 'gap-1');

  const classIndicator = clsx(
    'w-2',
    'h-2',
    'rounded-full',
    currentLoss !== null && isTraining ? 'bg-green-400' : 'bg-gray-300',
    currentLoss !== null && isTraining ? 'animate-pulse' : ''
  );

  return (
    <div className={classContainer}>
      <div className={classHeader}>
        <span>Training Loss</span>
      </div>

      <div className={classLossContainer}>
        <div className={classStatus}>
          <div className={classIndicator} />
        </div>

        <div className={classLossValue}>{formatLoss(currentLoss)}</div>

        <div className={classStepInfo}>
          {currentLoss !== null ? (
            <>at step {formatStep(currentStep)}</>
          ) : (
            <>{isTraining ? 'Waiting for loss data...' : 'No data available'}</>
          )}
        </div>
      </div>

      {/* Placeholder message when no data */}
      {!isTraining && currentLoss === null && (
        <div className="text-xs text-gray-400 text-center mt-1">
          Loss will be displayed when training starts
        </div>
      )}
    </div>
  );
}
