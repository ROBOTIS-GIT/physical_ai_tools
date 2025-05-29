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

export default function EpisodeStatus() {
  return (
    <div
      className={clsx(
        'h-full',
        'w-full',
        'max-w-xs',
        'text-center',
        'flex',
        'flex-col',
        'items-center',
        'rounded-xl',
        'border',
        'p-2',
        'box-border',
        'shadow-md'
      )}
    >
      <div className="mb-1 justify-start text-2xl">Episode</div>
      <div className="h-3"></div>
      <div className="w-full bg-gray-200 rounded-lg py-1.5 px-3 text-3xl font-bold">
        <span className="font-bold">14</span> / <span className="text-gray-600">150</span>
      </div>
    </div>
  );
}
