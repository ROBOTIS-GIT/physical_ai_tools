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

import React, { useCallback, useEffect, useRef } from 'react';
import clsx from 'clsx';
import { MdClose } from 'react-icons/md';
import { useSelector } from 'react-redux';

const classImageGridCell = (topic) =>
  clsx(
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
  );

const classImageGridCellButton = clsx(
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
);

export default function ImageGridCell({
  topic,
  aspect,
  idx,
  onClose,
  onPlusClick,
  isActive = true,
  style = {},
}) {
  const rosHost = useSelector((state) => state.ros.rosHost);
  const containerRef = useRef(null);
  const currentImgRef = useRef(null);
  const isCreatingRef = useRef(false); // Track if createImage is in progress

  // Completely remove img element from DOM
  const destroyImage = useCallback(() => {
    if (currentImgRef.current) {
      console.log(`Destroying image stream for idx ${idx}`);
      // First set src to empty
      currentImgRef.current.src = '';
      // Remove from DOM completely
      if (currentImgRef.current.parentNode) {
        currentImgRef.current.parentNode.removeChild(currentImgRef.current);
      }
      currentImgRef.current = null;
    }
  }, [idx]);

  // Create new img element and add to DOM with staggered delay
  const createImage = useCallback(async () => {
    if (!topic || !topic.trim() || !isActive || !containerRef.current) {
      return;
    }

    // Prevent multiple createImage calls from running simultaneously
    if (isCreatingRef.current) {
      console.log(`CreateImage already in progress for idx ${idx}, skipping`);
      return;
    }

    isCreatingRef.current = true;
    destroyImage(); // Remove any existing image first

    try {
      // Staggered delay - center first, then left and right
      let staggeredDelay = 0;
      if (idx === 1) {
        // Center cell connects immediately
        staggeredDelay = 0;
      } else if (idx === 0 || idx === 2) {
        // Left and right cells connect after 300ms
        staggeredDelay = 300;
      }

      if (staggeredDelay > 0) {
        console.log(
          `Staggered delay ${staggeredDelay}ms for image stream idx ${idx}, topic: ${topic}`
        );
        await new Promise((resolve) => setTimeout(resolve, staggeredDelay));
      } else {
        console.log(`Immediate connection for center cell idx ${idx}, topic: ${topic}`);
      }

      // Check again if conditions are still valid after delay and if we should still proceed
      if (!topic || !topic.trim() || !isActive || !containerRef.current || !isCreatingRef.current) {
        console.log(
          `Conditions changed during delay or cancelled, aborting image stream for idx ${idx}`
        );
        return;
      }

      console.log(`Creating new image stream for idx ${idx}, topic: ${topic}`);

      const img = document.createElement('img');
      const timestamp = Date.now();
      img.src = `http://${rosHost}:8080/stream?quality=50&type=ros_compressed&default_transport=compressed&topic=${topic}&t=${timestamp}`;
      img.alt = topic;
      img.className = 'w-full h-full object-cover rounded-3xl bg-gray-100';
      img.onclick = (e) => e.stopPropagation();

      // Error and load handlers
      img.onerror = () => {
        console.error(`Image stream error for idx ${idx}, topic: ${topic}`);
      };

      if (containerRef.current && isCreatingRef.current) {
        containerRef.current.appendChild(img);
        currentImgRef.current = img;
      }
    } finally {
      isCreatingRef.current = false;
    }
  }, [topic, isActive, rosHost, idx, destroyImage]);

  // Create/recreate image when topic, isActive, or rosHost changes
  useEffect(() => {
    if (topic && topic.trim() !== '' && isActive) {
      // Call async createImage function with error handling
      createImage().catch((error) => {
        console.error(`Error creating image stream for idx ${idx}:`, error);
        isCreatingRef.current = false; // Reset flag on error
      });
    } else {
      destroyImage();
    }

    return () => {
      // Cancel any ongoing createImage operation
      isCreatingRef.current = false;
      destroyImage();
    };
  }, [topic, isActive, rosHost, idx, createImage, destroyImage]);

  // Force cleanup on unmount
  useEffect(() => {
    return () => {
      destroyImage();
    };
  }, [idx, destroyImage]);

  const handleClose = (e) => {
    e.stopPropagation();
    destroyImage();
    onClose(idx);
  };

  return (
    <div
      className={classImageGridCell(topic)}
      onClick={!topic ? () => onPlusClick(idx) : undefined}
      style={{ cursor: !topic ? 'pointer' : 'default', aspectRatio: aspect, ...style }}
    >
      {topic && topic.trim() !== '' && (
        <button className={classImageGridCellButton} onClick={handleClose}>
          <MdClose size={20} />
        </button>
      )}
      <div ref={containerRef} className="w-full h-full flex items-center justify-center">
        {(!topic || !isActive) && <div className="text-6xl text-gray-400 font-light">+</div>}
      </div>
    </div>
  );
}
