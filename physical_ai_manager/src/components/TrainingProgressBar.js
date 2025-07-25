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

import React, { useEffect, useState, useCallback } from 'react';
import { useSelector } from 'react-redux';
import { setUpdateCounter } from '../features/training/trainingSlice';
import clsx from 'clsx';

export default function TrainingProgressBar() {
  const currentStep = useSelector((state) => state.training.currentStep);
  const totalSteps = useSelector((state) => state.training.trainingInfo.steps);
  const isTraining = useSelector((state) => state.training.isTraining);
  const updateCounter = useSelector((state) => state.training.updateCounter);

  const [spinnerIndex, setSpinnerIndex] = useState(0);

  const progressPercentage = totalSteps > 0 ? Math.min((currentStep / totalSteps) * 100, 100) : 0;

  const classContainer = clsx('w-full', 'rounded-lg', 'p-2');

  const spinnerFrames = ['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧'];

  const classProgressContainer = clsx(
    'w-full',
    'bg-gray-200',
    'rounded-full',
    'h-6',
    'overflow-hidden',
    'relative'
  );

  const classProgressBar = clsx(
    'h-full',
    'transition-all',
    'duration-300',
    // 'ease-out',
    isTraining ? 'bg-blue-500' : 'bg-gray-400'
  );

  const classStepText = clsx(
    'text-lg',
    'font-semibold',
    'text-gray-700',
    'mb-2',
    'flex',
    'justify-between',
    'items-center'
  );

  const classPercentageText = clsx(
    'absolute',
    'inset-0',
    'flex',
    'items-center',
    'justify-center',
    'text-sm',
    'font-bold',
    'text-white',
    'z-10'
  );

  // Add commas to numbers for thousands separator
  const formatNumber = (num) => {
    return num.toLocaleString();
  };

  const updateSpinnerFrame = useCallback(() => {
    setSpinnerIndex((prevIndex) => (prevIndex + 1) % spinnerFrames.length);
  }, [spinnerFrames.length]);

  useEffect(() => {
    updateSpinnerFrame();

    if (updateCounter > 100) {
      setUpdateCounter(0);
    }
  }, [updateCounter, updateSpinnerFrame]);

  return (
    <div className={classContainer}>
      <div className={classStepText}>
        <span>Training Progress</span>
        <span className="text-blue-600">
          {formatNumber(currentStep)} / {formatNumber(totalSteps)} steps
        </span>
      </div>

      <div className={classProgressContainer}>
        <div className={classProgressBar} style={{ width: `${progressPercentage}%` }} />
        <div className={classPercentageText}>{progressPercentage.toFixed(1)}%</div>
      </div>

      {isTraining && (
        <div className="mt-2 text-sm text-gray-500 text-center">
          Training in progress... {spinnerFrames[spinnerIndex]}
        </div>
      )}
    </div>
  );
}
