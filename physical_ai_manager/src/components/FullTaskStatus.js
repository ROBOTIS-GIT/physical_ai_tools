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

import { useSelector } from 'react-redux';

const classBody = clsx(
  'h-full',
  'w-full',
  'max-w-xs',
  'text-center',
  'flex',
  'flex-col',
  'items-center',
  'justify-around',
  'gap-1',
  'rounded-2xl',
  'border',
  'border-gray-200',
  'py-2',
  'px-3',
  'box-border',
  'shadow-md',
  'bg-white'
);

const MultiTaskFontSizeTitle = 'clamp(1rem, 1.2vw, 1.4rem)';
const MultiTaskFontSizeNumber = 'clamp(1rem, 1.2vw, 1.4rem)';

const SingleTaskFontSizeTitle = 'clamp(1.5rem, 1.5vw, 2rem)';
const SingleTaskFontSizeNumber = 'clamp(1.5rem, 1.5vw, 2rem)';

export default function FullTaskStatus() {
  const useMultiTaskMode = useSelector((state) => state.tasks.useMultiTaskMode);

  const currentScenarioNumber = useSelector(
    (state) => state.tasks.taskStatus.currentScenarioNumber
  );

  return (
    <div className={classBody}>
      <div
        className="w-full h-full flex justify-center items-center"
        style={{ fontSize: useMultiTaskMode ? MultiTaskFontSizeTitle : SingleTaskFontSizeTitle }}
      >
        Scenario
      </div>
      <div
        className="w-full h-full flex justify-center items-center bg-gray-200 rounded-lg px-3 font-bold whitespace-nowrap"
        style={{
          fontSize: useMultiTaskMode ? MultiTaskFontSizeNumber : SingleTaskFontSizeNumber,
        }}
      >
        <span className="font-bold">{currentScenarioNumber}</span>
      </div>
    </div>
  );
}
