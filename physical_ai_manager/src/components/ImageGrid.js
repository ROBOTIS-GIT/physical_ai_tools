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
import ImageGridCell from './ImageGridCell';

const layout = [{ aspect: '16/9' }, { aspect: '16/9' }, { aspect: '16/9' }];

const topicList = [
  '/camera_left/camera_left/color/image_rect_raw',
  '/camera_right/camera_right/color/image_rect_raw',
  '/zed/zed_node/rgb/image_rect_color',
  '/image_raw',
];

function TopicSelectModal({ topicList, onSelect, onClose }) {
  const [hovered, setHovered] = useState(null);
  const [selected, setSelected] = useState(null);

  return (
    <div
      className={clsx(
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
      )}
    >
      <div className="bg-white rounded-xl p-8 min-w-[420px]">
        <h3 className="mb-8 text-4xl">Select Image Topic</h3>
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
        <button
          className="mt-5 w-1/3 min-h-[50px] text-3xl font-medium rounded-lg border-0 shadow-md"
          onClick={onClose}
        >
          Close
        </button>
      </div>
    </div>
  );
}

export default function ImageGrid({ topics, setTopics, rosHost }) {
  const [modalOpen, setModalOpen] = React.useState(false);
  const [selectedIdx, setSelectedIdx] = React.useState(null);

  // Adjust the length of the topics array
  React.useEffect(() => {
    if (topics.length !== layout.length) {
      setTopics(Array(layout.length).fill(null));
    }
    // eslint-disable-next-line
  }, []);

  const handlePlusClick = (idx) => {
    setSelectedIdx(idx);
    setModalOpen(true);
  };

  const handleTopicSelect = (topic) => {
    setTopics(topics.map((t, i) => (i === selectedIdx ? topic : t)));
    setModalOpen(false);
    setSelectedIdx(null);
  };

  const handleCellClose = (idx) => {
    const img = document.querySelector(`#img-stream-${idx}`);
    if (img) img.src = '';
    setTopics(topics.map((t, i) => (i === idx ? null : t)));
  };

  return (
    <div className="w-full h-full overflow-hidden">
      <div
        className={clsx(
          'flex',
          'flex-row',
          'justify-center',
          'items-center',
          'gap-[0.5vw]',
          'w-full',
          'h-full',
          'max-w-full',
          'max-h-full',
          'overflow-hidden'
        )}
      >
        {layout.map((cell, idx) => (
          <div
            key={idx}
            className={clsx(
              'min-w-0',
              'min-h-0',
              'flex',
              'items-center',
              'justify-center',
              'relative',
              { 'flex-[7_1_0]': idx === 1, 'flex-[3_1_0]': idx !== 1 }
            )}
          >
            <ImageGridCell
              topic={topics[idx]}
              aspect={cell.aspect}
              idx={idx}
              rosHost={rosHost}
              onClose={handleCellClose}
              onPlusClick={handlePlusClick}
            />
            <div
              className={clsx(
                'absolute',
                'bottom-2',
                'left-2',
                'text-xs',
                'text-white',
                'bg-black',
                'bg-opacity-50',
                'px-2',
                'py-1',
                'rounded'
              )}
            >
              {topics[idx] || ''}
            </div>
          </div>
        ))}
        {modalOpen && (
          <TopicSelectModal
            topicList={topicList}
            onSelect={handleTopicSelect}
            onClose={() => setModalOpen(false)}
          />
        )}
      </div>
    </div>
  );
}
