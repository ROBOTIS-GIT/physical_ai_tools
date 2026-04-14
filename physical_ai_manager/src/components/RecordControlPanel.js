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

import React, { useState, useEffect } from 'react';
import { useSelector } from 'react-redux';
import clsx from 'clsx';
import TaskPhase from '../constants/taskPhases';

const phaseGuideMessages = {
  [TaskPhase.READY]: 'Ready',
  [TaskPhase.WARMING_UP]: 'Warming up...',
  [TaskPhase.RESETTING]: 'Resetting...',
  [TaskPhase.RECORDING]: 'Recording',
  [TaskPhase.SAVING]: 'Saving...',
  [TaskPhase.STOPPED]: 'Between segments',
  [TaskPhase.CONVERTING]: 'Merging...',
};

const spinnerFrames = ['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧'];

export default function RecordControlPanel() {
  const taskStatus = useSelector((state) => state.tasks.taskStatus);
  const [spinnerIndex, setSpinnerIndex] = useState(0);

  const phase = taskStatus.phase;
  const isRecording = phase === TaskPhase.RECORDING;
  const isSaving = phase === TaskPhase.SAVING;
  const isConverting = phase === TaskPhase.CONVERTING;
  const isBusy = isRecording || isSaving || isConverting;

  useEffect(() => {
    setSpinnerIndex((prev) => (prev + 1) % spinnerFrames.length);
  }, [taskStatus]);

  const classBody = clsx(
    'bg-white/90',
    'backdrop-blur-sm',
    'rounded-full',
    'px-3',
    'py-1',
    'flex',
    'flex-row',
    'items-center',
    'gap-1.5',
    'shadow-md',
    'border',
    'border-gray-100'
  );

  return (
    <div className={classBody}>
      <span className="text-lg font-semibold text-gray-500 whitespace-nowrap px-1 shrink-0">
        Record
      </span>
      <div className="w-px h-2/3 bg-gray-300 shrink-0"></div>

      <div className="flex items-center gap-1 shrink-0 px-1">
        <span className="text-gray-600 font-semibold text-lg whitespace-nowrap">
          {phaseGuideMessages[phase] || ''}
        </span>
        {isBusy && (
          <span className="font-mono text-blue-500 text-sm">
            {spinnerFrames[spinnerIndex]}
          </span>
        )}
      </div>

      {isRecording && (
        <>
          <div className="w-px h-2/3 bg-gray-400 shrink-0"></div>
          <div className="flex items-center gap-1 shrink-0 px-1">
            <span className="text-gray-500 text-lg font-medium">
              {taskStatus.proceedTime}s
              {taskStatus.totalTime > 0 ? ` / ${taskStatus.totalTime}s` : ''}
            </span>
          </div>
        </>
      )}
    </div>
  );
}
