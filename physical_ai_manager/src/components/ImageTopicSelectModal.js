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

const WEBCAM_HOST_STORAGE_KEY = 'webcamServerHost';

const getInitialWebcamHost = () => {
  try {
    const saved = localStorage.getItem(WEBCAM_HOST_STORAGE_KEY);
    if (saved && saved.trim()) return saved.trim();
  } catch (_) {
    // localStorage may be unavailable (SSR, privacy mode); fall through
  }
  if (typeof window !== 'undefined' && window.location && window.location.hostname) {
    return `${window.location.hostname}:8091`;
  }
  return 'localhost:8091';
};

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
  const [manualUrl, setManualUrl] = useState('');

  const [webcamHost, setWebcamHost] = useState(getInitialWebcamHost);
  const [webcamCameras, setWebcamCameras] = useState([]);
  const [webcamError, setWebcamError] = useState(null);
  const [webcamLoading, setWebcamLoading] = useState(false);

  const handleScanWebcams = async () => {
    const host = webcamHost.trim();
    if (!host) {
      setWebcamError('Enter a webcam server host (e.g. 192.168.0.10:8091)');
      return;
    }
    setWebcamLoading(true);
    setWebcamError(null);
    try {
      const url = `http://${host}/cameras.json`;
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 5000);
      const res = await fetch(url, { signal: controller.signal });
      clearTimeout(timeoutId);
      if (!res.ok) {
        throw new Error(`HTTP ${res.status}`);
      }
      const data = await res.json();
      const cams = Array.isArray(data?.cameras) ? data.cameras : [];
      setWebcamCameras(cams);
      try {
        localStorage.setItem(WEBCAM_HOST_STORAGE_KEY, host);
      } catch (_) {
        // ignore persistence failure
      }
    } catch (err) {
      setWebcamCameras([]);
      const msg = err?.name === 'AbortError' ? 'Timed out' : (err?.message || 'Failed to fetch');
      setWebcamError(`Scan failed: ${msg}`);
    } finally {
      setWebcamLoading(false);
    }
  };

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

        {/* Local Webcam Server (all_cams.py) discovery */}
        <div className="mb-4 p-3 bg-gray-50 border border-gray-200 rounded-md">
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Local Webcam Server (all_cams.py)
          </label>
          <div className="flex gap-2">
            <input
              type="text"
              value={webcamHost}
              onChange={(e) => setWebcamHost(e.target.value)}
              onKeyDown={(e) => {
                if (e.key === 'Enter' && webcamHost.trim()) {
                  handleScanWebcams();
                }
              }}
              placeholder="laptop-ip:8091"
              className="flex-1 px-3 py-2 border border-gray-300 rounded-md text-sm focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
            <button
              onClick={handleScanWebcams}
              disabled={!webcamHost.trim() || webcamLoading}
              className={clsx('px-4 py-2 rounded-md text-sm font-medium transition-colors whitespace-nowrap', {
                'bg-blue-500 text-white hover:bg-blue-600': webcamHost.trim() && !webcamLoading,
                'bg-gray-300 text-gray-500 cursor-not-allowed': !webcamHost.trim() || webcamLoading,
              })}
            >
              {webcamLoading ? 'Scanning...' : 'Scan'}
            </button>
          </div>
          {webcamError && (
            <div className="mt-2 text-xs text-red-600">{webcamError}</div>
          )}
          {webcamCameras.length > 0 && (
            <ul className="list-none p-0 mt-2 mb-0">
              {webcamCameras.map((cam) => {
                const key = cam.stream_url || cam.cam_id;
                const label = cam.name || cam.label || cam.cam_id || cam.stream_url;
                return (
                  <li
                    key={key}
                    className={clsx(
                      'my-1 cursor-pointer px-3 py-2 rounded-md text-sm transition-colors duration-200',
                      {
                        'bg-blue-700 text-white': selected === cam.stream_url,
                        'bg-blue-200': hovered === cam.stream_url && selected !== cam.stream_url,
                        'bg-white border border-gray-200 hover:bg-blue-100':
                          hovered !== cam.stream_url && selected !== cam.stream_url,
                      }
                    )}
                    onMouseEnter={() => setHovered(cam.stream_url)}
                    onMouseLeave={() => setHovered(null)}
                    onClick={() => {
                      if (!cam.stream_url) return;
                      setSelected(cam.stream_url);
                      onSelect(cam.stream_url);
                    }}
                    title={cam.stream_url}
                  >
                    <span className="font-medium">{label}</span>
                    <span className="ml-2 text-xs opacity-70">{cam.stream_url}</span>
                  </li>
                );
              })}
            </ul>
          )}
        </div>

        {/* Manual URL input for external streams (e.g. GoPro MJPEG) */}
        <div className="mb-4 p-3 bg-gray-50 border border-gray-200 rounded-md">
          <label className="block text-sm font-medium text-gray-700 mb-1">
            External Stream URL
          </label>
          <div className="flex gap-2">
            <input
              type="text"
              value={manualUrl}
              onChange={(e) => setManualUrl(e.target.value)}
              onKeyDown={(e) => {
                if (e.key === 'Enter' && manualUrl.trim()) {
                  onSelect(manualUrl.trim());
                }
              }}
              placeholder="http://laptop-ip:8090/stream"
              className="flex-1 px-3 py-2 border border-gray-300 rounded-md text-sm focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
            <button
              onClick={() => { if (manualUrl.trim()) onSelect(manualUrl.trim()); }}
              disabled={!manualUrl.trim()}
              className={clsx('px-4 py-2 rounded-md text-sm font-medium transition-colors whitespace-nowrap', {
                'bg-green-500 text-white hover:bg-green-600': manualUrl.trim(),
                'bg-gray-300 text-gray-500 cursor-not-allowed': !manualUrl.trim(),
              })}
            >
              Use URL
            </button>
          </div>
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
