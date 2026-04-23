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
import { MdClose, MdScreenRotation } from 'react-icons/md';
import { useSelector } from 'react-redux';
import {
  isBrowserCameraTopic,
  getBrowserCameraDeviceId,
} from '../utils/browserCameraLabels';

const classCell = (topic) =>
  clsx(
    'relative',
    'bg-gray-100',
    'rounded-3xl',
    'overflow-hidden',
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

const classCloseBtn = clsx(
  'absolute', 'top-2', 'right-2',
  'w-8', 'h-8',
  'bg-black', 'bg-opacity-50', 'text-white',
  'rounded-full', 'flex', 'items-center', 'justify-center',
  'hover:bg-opacity-70', 'z-20'
);

const classRotateBtn = clsx(
  'absolute', 'top-2', 'left-2',
  'w-8', 'h-8',
  'bg-black', 'bg-opacity-50', 'text-white',
  'rounded-full', 'flex', 'items-center', 'justify-center',
  'hover:bg-opacity-70', 'z-20'
);

export default function ImageGridCell({
  topic,
  aspect,
  rotationDegrees = 0,
  onRotateClick,
  idx,
  onClose,
  onPlusClick,
  isActive = true,
  style = {},
}) {
  const rotate = rotationDegrees !== 0;
  const rosHost = useSelector((state) => state.ros.rosHost);
  const containerRef = useRef(null);
  const currentImgRef = useRef(null);
  const streamRef = useRef(null);
  const isCreatingRef = useRef(false);
  const cancelRef = useRef(false);
  const retryTimerRef = useRef(null);
  const retryCountRef = useRef(0);
  const MAX_RETRIES = 5;

  const destroyImage = useCallback(() => {
    // Signal any in-flight createImage waiting on the staggered delay to bail.
    cancelRef.current = true;
    if (retryTimerRef.current) {
      clearTimeout(retryTimerRef.current);
      retryTimerRef.current = null;
    }
    if (streamRef.current) {
      streamRef.current.getTracks().forEach((t) => {
        try { t.stop(); } catch (_) { /* ignore */ }
      });
      streamRef.current = null;
    }
    if (currentImgRef.current) {
      const el = currentImgRef.current;
      const img = el.tagName === 'IMG' ? el : el.querySelector('img');
      if (img) {
        // Detach handlers BEFORE clearing src so the late-fired onerror that
        // src='' triggers can't schedule a retry that revives the stream.
        img.onerror = null;
        img.onload = null;
        img.src = '';
      }
      const video = el.tagName === 'VIDEO' ? el : el.querySelector('video');
      if (video) {
        try { video.pause(); } catch (_) { /* ignore */ }
        video.srcObject = null;
      }
      if (el.parentNode) el.parentNode.removeChild(el);
      currentImgRef.current = null;
    }
  }, []);

  const createImage = useCallback(async () => {
    if (!topic || !topic.trim() || !isActive || !containerRef.current) return;
    if (isCreatingRef.current) return;

    isCreatingRef.current = true;
    destroyImage();
    // Clear cancel flag raised by destroyImage; later destroys will set it again.
    cancelRef.current = false;

    try {
      const staggeredDelay = (idx === 0 || idx === 2) ? 300 : 0;
      if (staggeredDelay > 0) {
        await new Promise((resolve) => setTimeout(resolve, staggeredDelay));
      }

      if (cancelRef.current || !topic || !topic.trim() || !isActive || !containerRef.current) return;

      // Browser-attached camera via getUserMedia → use <video> instead of <img>
      if (isBrowserCameraTopic(topic)) {
        const deviceId = getBrowserCameraDeviceId(topic);
        let stream;
        try {
          stream = await navigator.mediaDevices.getUserMedia({
            video: deviceId ? { deviceId: { exact: deviceId } } : true,
            audio: false,
          });
        } catch (err) {
          console.error(`getUserMedia failed for idx ${idx}:`, err);
          return;
        }
        if (cancelRef.current || !isActive || !containerRef.current) {
          stream.getTracks().forEach((t) => { try { t.stop(); } catch (_) { /* ignore */ } });
          return;
        }
        streamRef.current = stream;

        const video = document.createElement('video');
        video.autoplay = true;
        video.playsInline = true;
        video.muted = true;
        video.srcObject = stream;
        video.onclick = (e) => e.stopPropagation();

        if (rotate) {
          const wrapper = document.createElement('div');
          wrapper.style.position = 'absolute';
          wrapper.style.width = '133.33%';
          wrapper.style.height = '75%';
          wrapper.style.top = '50%';
          wrapper.style.left = '50%';
          wrapper.style.transform = `translate(-50%, -50%) rotate(${rotationDegrees}deg)`;
          wrapper.style.transformOrigin = 'center center';
          wrapper.style.overflow = 'hidden';
          video.style.width = '100%';
          video.style.height = '100%';
          video.style.objectFit = 'cover';
          video.style.display = 'block';
          wrapper.appendChild(video);
          if (containerRef.current && !cancelRef.current) {
            containerRef.current.appendChild(wrapper);
            currentImgRef.current = wrapper;
          }
        } else {
          video.className = 'w-full h-full object-cover bg-gray-100';
          if (containerRef.current && !cancelRef.current) {
            containerRef.current.appendChild(video);
            currentImgRef.current = video;
          }
        }
        return;
      }

      const img = document.createElement('img');
      const timestamp = Date.now();
      if (topic.startsWith('http://') || topic.startsWith('https://')) {
        // External MJPEG stream URL (e.g. GoPro on laptop) — use directly
        const separator = topic.includes('?') ? '&' : '?';
        img.src = `${topic}${separator}t=${timestamp}`;
      } else {
        // ROS topic — build web_video_server URL
        // web_video_server expects base topic (e.g. .../image_raw); use default_transport=compressed to subscribe to CompressedImage
        // Do not encode slashes: server rejects %2F and expects literal /
        const streamTopic = topic.endsWith('/compressed') ? topic.slice(0, -11) : topic;
        // On HTTPS pages, http://host:8085 would be blocked as mixed content.
        // Same-origin /ros_stream/ is proxied by nginx to 127.0.0.1:8085.
        const base =
          typeof window !== 'undefined' && window.location.protocol === 'https:'
            ? `${window.location.origin}/ros_stream`
            : `http://${rosHost}:8085`;
        img.src = `${base}/stream?quality=50&type=ros_compressed&default_transport=compressed&topic=${streamTopic}&t=${timestamp}`;
      }
      img.alt = topic;

      img.onclick = (e) => e.stopPropagation();
      img.onerror = () => {
        // Late-fired error after destroyImage (src='' triggers onerror) must
        // not schedule a retry — cancelRef tells us we're already torn down.
        if (cancelRef.current) return;
        if (retryCountRef.current >= MAX_RETRIES) {
          console.error(`Image stream failed after ${MAX_RETRIES} retries for idx ${idx}, topic: ${topic}`);
          return;
        }
        retryCountRef.current += 1;
        const delay = Math.min(1000 * Math.pow(2, retryCountRef.current - 1), 8000);
        console.warn(`Image stream error for idx ${idx}, retrying in ${delay}ms (${retryCountRef.current}/${MAX_RETRIES})`);
        retryTimerRef.current = setTimeout(() => {
          if (cancelRef.current) return;
          if (isActive && topic && containerRef.current) {
            destroyImage();
            isCreatingRef.current = false;
            createImage();
          }
        }, delay);
      };
      img.onload = () => {
        retryCountRef.current = 0;
      };

      if (rotate) {
        // Wrapper div handles rotation; img inside handles fitting.
        // For 3:4 container (W x H where H=4W/3):
        //   wrapper pre-rotation: width=H, height=W (landscape box)
        //   after rotate(-90deg): visual bounding box = W x H (matches container)
        const wrapper = document.createElement('div');
        wrapper.style.position = 'absolute';
        wrapper.style.width = '133.33%';   // container height = 4W/3 = 133.33% of W
        wrapper.style.height = '75%';      // container width  = 3H/4 = 75% of H
        wrapper.style.top = '50%';
        wrapper.style.left = '50%';
        wrapper.style.transform = `translate(-50%, -50%) rotate(${rotationDegrees}deg)`;
        wrapper.style.transformOrigin = 'center center';
        wrapper.style.overflow = 'hidden';

        img.style.width = '100%';
        img.style.height = '100%';
        img.style.objectFit = 'cover';
        img.style.display = 'block';

        wrapper.appendChild(img);

        if (containerRef.current && !cancelRef.current) {
          containerRef.current.appendChild(wrapper);
          currentImgRef.current = wrapper;
        }
      } else {
        img.className = 'w-full h-full object-cover bg-gray-100';

        if (containerRef.current && !cancelRef.current) {
          containerRef.current.appendChild(img);
          currentImgRef.current = img;
        }
      }
    } finally {
      isCreatingRef.current = false;
    }
  }, [topic, isActive, rosHost, idx, rotate, rotationDegrees, destroyImage]);

  useEffect(() => {
    retryCountRef.current = 0;
    if (topic && topic.trim() !== '' && isActive) {
      createImage().catch((error) => {
        console.error(`Error creating image stream for idx ${idx}:`, error);
        isCreatingRef.current = false;
      });
    } else {
      destroyImage();
    }

    return () => {
      isCreatingRef.current = false;
      retryCountRef.current = 0;
      destroyImage();
    };
  }, [topic, isActive, rosHost, idx, createImage, destroyImage]);

  useEffect(() => {
    return () => { destroyImage(); };
  }, [idx, destroyImage]);

  const handleClose = (e) => {
    e.stopPropagation();
    destroyImage();
    onClose(idx);
  };

  return (
    <div
      className={classCell(topic)}
      onClick={!topic ? () => onPlusClick(idx) : undefined}
      style={{ cursor: !topic ? 'pointer' : 'default', aspectRatio: aspect, ...style }}
    >
      {topic && topic.trim() !== '' && (
        <>
          <button
            type="button"
            className={classRotateBtn}
            onClick={(e) => { e.stopPropagation(); onRotateClick?.(idx); }}
            title={rotate ? '가로로 보기' : '세로로 보기'}
          >
            <MdScreenRotation size={20} />
          </button>
          <button type="button" className={classCloseBtn} onClick={handleClose}>
            <MdClose size={20} />
          </button>
        </>
      )}
      <div ref={containerRef} className="w-full h-full relative overflow-hidden rounded-3xl flex items-center justify-center">
        {(!topic || !isActive) && <div className="text-6xl text-gray-400 font-light">+</div>}
      </div>
    </div>
  );
}
