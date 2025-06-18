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

const classEpisodeStatusBody = clsx(
  'h-full',
  'w-full',
  'max-w-xs',
  'text-center',
  'flex',
  'flex-col',
  'items-center',
  'justify-center',
  'gap-1',
  'rounded-2xl',
  'border',
  'border-gray-200',
  'py-5',
  'px-4',
  'box-border',
  'shadow-md',
  'bg-white'
);

export default function EpisodeStatus({ episodeStatus }) {
  return (
    <div className={classEpisodeStatusBody}>
      <div className="mb-1 justify- text-3xl" style={{ fontSize: 'clamp(1.5rem, 1.5vw, 2rem)' }}>
        Episode
      </div>
      <div className="h-3"></div>
      <div
        className="w-full bg-gray-200 rounded-lg py-1.5 px-3 font-bold whitespace-nowrap"
        style={{ fontSize: 'clamp(1rem, 1.5vw, 2rem)' }}
      >
        <span className="font-bold">{episodeStatus?.currentEpisodeNumber}</span> /{' '}
        <span className="text-gray-600">{episodeStatus?.numEpisodes}</span>
      </div>
    </div>
  );
}
