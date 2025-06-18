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

import React, { useState } from 'react';
import clsx from 'clsx';

const ImageTopicSelectModal = ({
  topicList,
  onSelect,
  onClose,
  isLoading,
  onRefresh,
  errorMessage,
}) => {
  const [hovered, setHovered] = useState(null);
  const [selected, setSelected] = useState(null);

  const classImageTopicSelectModal = clsx(
    'fixed',
    'top-0',
    'left-0',
    'w-screen',
    'h-screen',
    'bg-black',
    'bg-opacity-20',
    'flex',
    'items-center',
    'justify-center',
    'z-50'
  );

  return (
    <div className={classImageTopicSelectModal}>
      <div className="bg-white rounded-xl p-8 min-w-[420px] max-h-[80vh] overflow-hidden flex flex-col">
        <div className="flex justify-between gap-4 items-center mb-6">
          <h3 className="text-4xl">Select Image Topic</h3>
          <button
            onClick={onRefresh}
            disabled={isLoading}
            className={clsx('px-4 py-2 rounded-md text-sm font-medium transition-colors', {
              'bg-blue-500 text-white hover:bg-blue-600': !isLoading,
              'bg-gray-400 text-gray-600 cursor-not-allowed': isLoading,
            })}
          >
            {isLoading ? 'Refreshing...' : 'Refresh'}
          </button>
        </div>

        {/* Error message display */}
        {errorMessage && !isLoading && (
          <div className="mb-4 p-3 bg-red-100 border border-red-300 rounded-md">
            <div className="text-red-800 text-sm font-medium">⚠️ {errorMessage}</div>
          </div>
        )}

        <div className="flex-1 overflow-auto">
          {isLoading ? (
            <div className="flex items-center justify-center py-8">
              <div className="text-xl text-gray-600">Loading topics...</div>
            </div>
          ) : errorMessage ? (
            <div className="flex items-center justify-center py-8">
              <div className="text-xl text-gray-500 italic">No topics to display</div>
            </div>
          ) : topicList.length === 0 ? (
            <div className="flex items-center justify-center py-8">
              <div className="text-xl text-gray-600">No image topics available</div>
            </div>
          ) : (
            <ul className="list-none p-0 m-0">
              {topicList.map((topic) => (
                <li
                  key={topic}
                  className={clsx(
                    'my-2 cursor-pointer p-3 rounded-md text-xl transition-colors duration-200',
                    {
                      'bg-blue-700 text-white': selected === topic,
                      'bg-blue-200': hovered === topic && selected !== topic,
                      'bg-gray-200': hovered !== topic && selected !== topic,
                    }
                  )}
                  onMouseEnter={() => setHovered(topic)}
                  onMouseLeave={() => setHovered(null)}
                  onClick={() => {
                    setSelected(topic);
                    onSelect(topic);
                  }}
                >
                  {topic}
                </li>
              ))}
            </ul>
          )}
        </div>

        <button
          className="mt-5 w-1/3 min-h-[50px] text-3xl font-medium rounded-lg border-0 shadow-md"
          onClick={onClose}
        >
          Close
        </button>
      </div>
    </div>
  );
};

export default ImageTopicSelectModal;
