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
import './ImageGrid.css';
import ImageGridCell from './ImageGridCell';

const layout = [{ aspect: '16/9' }, { aspect: '16/9' }, { aspect: '16/9' }];

const topicList = [
  '/camera_left/camera_left/color/image_rect_raw',
  '/camera_right/camera_right/color/image_rect_raw',
  '/zed/zed_node/rgb/image_rect_color',
];

function TopicSelectModal({ topicList, onSelect, onClose }) {
  const [hovered, setHovered] = useState(null);
  const [selected, setSelected] = useState(null);

  return (
    <div
      className="modal-backdrop"
      style={{
        position: 'fixed',
        top: 0,
        left: 0,
        width: '100vw',
        height: '100vh',
        background: 'rgba(0,0,0,0.2)',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        zIndex: 1000,
      }}
    >
      <div
        className="modal"
        style={{ background: '#fff', borderRadius: 12, padding: 32, minWidth: 420 }}
      >
        <h3 style={{ marginBottom: 30, fontSize: 40 }}>Select Image Topic</h3>
        <ul style={{ listStyle: 'none', padding: 0, margin: 0 }}>
          {topicList.map((topic) => (
            <li
              key={topic}
              style={{
                margin: '8px 0',
                cursor: 'pointer',
                padding: 12,
                borderRadius: 6,
                fontSize: 20,
                background:
                  selected === topic
                    ? 'rgba(21, 101, 192, 0.92)'
                    : hovered === topic
                    ? '#90caf9'
                    : '#eee',
                color: selected === topic ? '#fff' : '#000',
                transition: 'background 0.2s',
              }}
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
          style={{
            marginTop: 20,
            width: '30%',
            minHeight: 50,
            fontSize: 30,
            fontWeight: 500,
            fontFamily: 'Pretendard Variable',
            borderRadius: 8,
            border: '0px solid',
            boxShadow: '0 2px 6px rgba(0, 0, 0, 0.15);',
          }}
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

  // Fixed 3 Horizontal (Vertical-Horizontal-Vertical)
  const gridStyle = {
    display: 'flex',
    flexDirection: 'row',
    justifyContent: 'center',
    alignItems: 'flex-start',
    gap: '2vw',
    width: '100%',
    height: '100%',
  };
  const cellWrapperStyle = {
    flex: '1 1 0',
    minWidth: 0,
    minHeight: 0,
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
  };
  const cellStyle = {};

  return (
    <div style={{ width: '100%', height: '100%', overflow: 'hidden' }}>
      <div
        className="image-grid"
        style={{ ...gridStyle, maxWidth: '100%', maxHeight: '100%', overflow: 'hidden' }}
      >
        {layout.map((cell, idx) => (
          <div
            key={idx}
            className="cell-wrapper"
            style={{
              ...cellWrapperStyle,
              flex: idx === 1 ? '3 1 0' : '1 1 0',
            }}
          >
            <ImageGridCell
              topic={topics[idx]}
              aspect={cell.aspect}
              idx={idx}
              rosHost={rosHost}
              onClose={handleCellClose}
              onPlusClick={handlePlusClick}
              style={cellStyle}
            />
            <div className="img-label">{topics[idx] || ''}</div>
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
