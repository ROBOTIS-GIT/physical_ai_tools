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
import { MdClose } from 'react-icons/md';

export default function ImageGridCell({
  topic,
  aspect,
  idx,
  rosHost,
  onClose,
  onPlusClick,
  style = {},
}) {
  return (
    <div
      className={clsx(
        'relative',
        'bg-gray-100',
        'rounded-3xl',
        'flex',
        'items-center',
        'justify-center',
        'transition-all',
        'duration-300',
        'w-full',
        {
          'border-2 border-dashed border-gray-300 hover:border-gray-400': !topic,
          'bg-white': topic,
        }
      )}
      onClick={!topic ? () => onPlusClick(idx) : undefined}
      style={{ cursor: !topic ? 'pointer' : 'default', aspectRatio: aspect, ...style }}
    >
      {topic && topic.trim() !== '' && (
        <button
          className={clsx(
            'absolute',
            'top-2',
            'right-2',
            'w-8',
            'h-8',
            'bg-black',
            'bg-opacity-50',
            'text-white',
            'rounded-full',
            'flex',
            'items-center',
            'justify-center',
            'hover:bg-opacity-70',
            'z-10'
          )}
          onClick={(e) => {
            e.stopPropagation();
            const img = document.querySelector(`#img-stream-${idx}`);
            if (img) img.src = '';
            onClose(idx);
          }}
        >
          <MdClose size={20} />
        </button>
      )}
      {topic && topic.trim() !== '' ? (
        <img
          id={`img-stream-${idx}`}
          src={`http://${rosHost}/stream?topic=${topic}`}
          alt={topic}
          className="w-full h-full object-cover rounded-3xl bg-gray-100"
          onClick={(e) => e.stopPropagation()}
        />
      ) : (
        <div className="text-6xl text-gray-400 font-light">+</div>
      )}
    </div>
  );
}
