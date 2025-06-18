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

export default function ProgressBar({ percent = 0 }) {
  // If over 50%, white, otherwise dark color
  const textColor = percent > 50 ? 'text-white' : 'text-gray-800';

  const classProgressBarText = clsx(
    'absolute',
    'left-0',
    'top-0',
    'w-full',
    'h-full',
    'flex',
    'items-center',
    'justify-center',
    'font-bold',
    'text-lg',
    'pointer-events-none',
    'z-10',
    'transition-colors',
    'duration-300',
    textColor
  );

  return (
    <div className="w-full h-9 bg-gray-400 rounded-full relative overflow-hidden">
      <div
        className="h-full bg-gray-700 rounded-full transition-all duration-300"
        style={{ width: `${percent}%` }}
      ></div>
      <span className={classProgressBarText}>{percent}%</span>
    </div>
  );
}
