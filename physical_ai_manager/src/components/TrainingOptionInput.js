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
import { useSelector, useDispatch } from 'react-redux';
import clsx from 'clsx';
import {
  setSeed,
  setNumWorkers,
  setBatchSize,
  setSteps,
  setEvalFreq,
  setLogFreq,
  setSaveFreq,
  setDefaultTrainingInfo,
} from '../features/training/trainingSlice';

const TrainingOptionInput = () => {
  const dispatch = useDispatch();

  const seed = useSelector((state) => state.training.trainingInfo.seed);
  const numWorkers = useSelector((state) => state.training.trainingInfo.numWorkers);
  const batchSize = useSelector((state) => state.training.trainingInfo.batchSize);
  const steps = useSelector((state) => state.training.trainingInfo.steps);
  const evalFreq = useSelector((state) => state.training.trainingInfo.evalFreq);
  const logFreq = useSelector((state) => state.training.trainingInfo.logFreq);
  const saveFreq = useSelector((state) => state.training.trainingInfo.saveFreq);

  const isTraining = useSelector((state) => state.training.isTraining);

  const classLabel = clsx('text-sm', 'text-gray-600', 'w-28', 'flex-shrink-0', 'font-medium');

  const classPanel = clsx(
    'bg-white',
    'border',
    'border-gray-200',
    'rounded-2xl',
    'shadow-md',
    'p-6',
    'w-full',
    'max-w-[350px]',
    'min-w-[250px]',
    'relative',
    'overflow-y-auto',
    'scrollbar-thin'
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
      'bg-gray-100 cursor-not-allowed': isTraining,
      'bg-white': !isTraining,
    }
  );

  const classResetButton = clsx(
    'w-full',
    'px-4',
    'py-2',
    'bg-gray-500',
    'text-white',
    'rounded-md',
    'font-medium',
    'transition-colors',
    'hover:bg-gray-600',
    'disabled:bg-gray-400',
    'disabled:cursor-not-allowed'
  );

  return (
    <div className={classPanel}>
      <div className={clsx('text-lg', 'font-semibold', 'mb-3', 'text-gray-800')}>
        Additional Options
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>Seed</span>
        <input
          className={classTextInput}
          type="number"
          step="1"
          min={0}
          max={65535}
          value={seed || ''}
          onChange={(e) => dispatch(setSeed(Number(e.target.value) || 0))}
          disabled={isTraining}
          placeholder="Enter Seed"
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>Num Workers</span>
        <input
          className={classTextInput}
          type="number"
          step="1"
          min={0}
          max={65535}
          value={numWorkers || ''}
          onChange={(e) => dispatch(setNumWorkers(Number(e.target.value) || 0))}
          disabled={isTraining}
          placeholder="Enter Num Workers"
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>Batch Size</span>
        <input
          className={classTextInput}
          type="number"
          step="1"
          min={0}
          max={65535}
          value={batchSize || ''}
          onChange={(e) => dispatch(setBatchSize(Number(e.target.value) || 0))}
          disabled={isTraining}
          placeholder="Enter Batch Size"
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>Steps</span>
        <input
          className={classTextInput}
          type="number"
          step="1"
          min={0}
          max={65535}
          value={steps || ''}
          onChange={(e) => dispatch(setSteps(Number(e.target.value) || 0))}
          disabled={isTraining}
          placeholder="Enter Steps"
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>Eval Frequency</span>
        <input
          className={classTextInput}
          type="number"
          step="1"
          min={0}
          max={65535}
          value={evalFreq || ''}
          onChange={(e) => dispatch(setEvalFreq(Number(e.target.value) || 0))}
          disabled={isTraining}
          placeholder="Enter Eval Frequency"
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>Log Frequency</span>
        <input
          className={classTextInput}
          type="number"
          step="1"
          min={0}
          max={65535}
          value={logFreq || ''}
          onChange={(e) => dispatch(setLogFreq(Number(e.target.value) || 0))}
          disabled={isTraining}
          placeholder="Enter Log Frequency"
        />
      </div>

      <div className={clsx('flex', 'items-center', 'mb-2.5')}>
        <span className={classLabel}>Save Frequency</span>
        <input
          className={classTextInput}
          type="number"
          step="1"
          min={0}
          max={65535}
          value={saveFreq || ''}
          onChange={(e) => dispatch(setSaveFreq(Number(e.target.value) || 0))}
          disabled={isTraining}
          placeholder="Enter Save Frequency"
        />
      </div>

      <button className={classResetButton} onClick={() => dispatch(setDefaultTrainingInfo())}>
        Reset to Default
      </button>
    </div>
  );
};

export default TrainingOptionInput;
