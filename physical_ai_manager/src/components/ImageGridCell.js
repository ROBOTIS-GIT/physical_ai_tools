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
import { MdClose } from 'react-icons/md';
import './ImageGrid.css';

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
      className={`image-cell${topic ? ' streaming' : ''}`}
      onClick={!topic ? () => onPlusClick(idx) : undefined}
      style={{ cursor: !topic ? 'pointer' : 'default', aspectRatio: aspect, ...style }}
    >
      {topic && topic.trim() !== '' && (
        <button
          className="close-btn"
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
          src={`http://${rosHost}/stream?quality=30&default_transport=compressed&topic=${topic}`}
          alt={topic}
          style={{ width: '100%', height: '100%', objectFit: 'cover', borderRadius: '20px' }}
          onClick={(e) => e.stopPropagation()}
        />
      ) : (
        <div className="plus-btn">+</div>
      )}
    </div>
  );
}
