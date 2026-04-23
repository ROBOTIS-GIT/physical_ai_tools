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

import React, { useEffect, useState } from 'react';
import clsx from 'clsx';
import {
  BROWSER_CAMERA_PREFIX,
  rememberBrowserCameraLabel,
} from '../utils/browserCameraLabels';

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

  const [browserDevices, setBrowserDevices] = useState([]);
  const [browserPermissionGranted, setBrowserPermissionGranted] = useState(false);
  const [browserError, setBrowserError] = useState(null);
  const [browserLoading, setBrowserLoading] = useState(false);

  const isSecureContext =
    typeof window !== 'undefined' &&
    (window.isSecureContext ||
      window.location.hostname === 'localhost' ||
      window.location.hostname === '127.0.0.1');

  const hasMediaDevices =
    typeof navigator !== 'undefined' &&
    navigator.mediaDevices &&
    typeof navigator.mediaDevices.enumerateDevices === 'function';

  const refreshBrowserDevices = async (granted) => {
    if (!hasMediaDevices) return;
    try {
      const devices = await navigator.mediaDevices.enumerateDevices();
      const videoInputs = devices
        .filter((d) => d.kind === 'videoinput')
        .map((d) => ({ deviceId: d.deviceId, label: d.label }));
      setBrowserDevices(videoInputs);
      if (granted) {
        videoInputs.forEach((d) => rememberBrowserCameraLabel(d.deviceId, d.label));
      }
    } catch (err) {
      setBrowserError(`Device enumeration failed: ${err?.message || err}`);
    }
  };

  const handleEnableBrowserCameras = async () => {
    if (!isSecureContext) {
      setBrowserError('HTTPS (secure context) is required for browser cameras.');
      return;
    }
    if (!hasMediaDevices || typeof navigator.mediaDevices.getUserMedia !== 'function') {
      setBrowserError('Browser does not support getUserMedia.');
      return;
    }
    setBrowserLoading(true);
    setBrowserError(null);
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ video: true, audio: false });
      stream.getTracks().forEach((t) => t.stop());
      setBrowserPermissionGranted(true);
      await refreshBrowserDevices(true);
    } catch (err) {
      setBrowserError(err?.name === 'NotAllowedError'
        ? 'Permission denied. Allow camera access in the browser.'
        : `Failed to access cameras: ${err?.message || err}`);
    } finally {
      setBrowserLoading(false);
    }
  };

  useEffect(() => {
    if (!hasMediaDevices || !isSecureContext) return;
    // If permission was previously granted, labels come back immediately.
    refreshBrowserDevices(false).then(() => {
      // If any label is present we can assume permission is active.
      // (Labels are empty strings when permission has not been granted.)
      // This is a best-effort auto-detect; user can always press Enable.
    });
    const handler = () => refreshBrowserDevices(browserPermissionGranted);
    if (navigator.mediaDevices.addEventListener) {
      navigator.mediaDevices.addEventListener('devicechange', handler);
      return () => navigator.mediaDevices.removeEventListener('devicechange', handler);
    }
    return undefined;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

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

        {/* Browser Cameras — cameras connected to the device running this browser */}
        <div className="mb-4 p-3 bg-gray-50 border border-gray-200 rounded-md">
          <div className="flex items-center justify-between mb-1">
            <label className="block text-sm font-medium text-gray-700">
              Browser Cameras (this device)
            </label>
            {isSecureContext && hasMediaDevices && (
              <button
                onClick={handleEnableBrowserCameras}
                disabled={browserLoading}
                className={clsx('px-3 py-1 rounded-md text-xs font-medium transition-colors whitespace-nowrap', {
                  'bg-blue-500 text-white hover:bg-blue-600': !browserLoading,
                  'bg-gray-300 text-gray-500 cursor-not-allowed': browserLoading,
                })}
              >
                {browserLoading ? 'Requesting...' : (browserPermissionGranted ? 'Refresh' : 'Enable')}
              </button>
            )}
          </div>
          {!isSecureContext && (
            <div className="mt-1 text-xs text-red-600">
              HTTPS connection required. Open this page via https:// to use browser cameras.
            </div>
          )}
          {isSecureContext && !hasMediaDevices && (
            <div className="mt-1 text-xs text-red-600">
              This browser does not support mediaDevices API.
            </div>
          )}
          {browserError && (
            <div className="mt-2 text-xs text-red-600">{browserError}</div>
          )}
          {isSecureContext && hasMediaDevices && browserDevices.length === 0 && !browserError && (
            <div className="mt-1 text-xs text-gray-500">
              {browserPermissionGranted
                ? 'No camera devices detected.'
                : 'Press Enable to grant camera access and list devices.'}
            </div>
          )}
          {browserDevices.length > 0 && (
            <ul className="list-none p-0 mt-2 mb-0">
              {browserDevices.map((dev) => {
                const topic = `${BROWSER_CAMERA_PREFIX}${dev.deviceId}`;
                const displayLabel = dev.label || `Camera ${dev.deviceId.slice(0, 6)}…`;
                return (
                  <li
                    key={dev.deviceId || displayLabel}
                    className={clsx(
                      'my-1 cursor-pointer px-3 py-2 rounded-md text-sm transition-colors duration-200',
                      {
                        'bg-blue-700 text-white': selected === topic,
                        'bg-blue-200': hovered === topic && selected !== topic,
                        'bg-white border border-gray-200 hover:bg-blue-100':
                          hovered !== topic && selected !== topic,
                      }
                    )}
                    onMouseEnter={() => setHovered(topic)}
                    onMouseLeave={() => setHovered(null)}
                    onClick={() => {
                      if (!dev.deviceId) return;
                      rememberBrowserCameraLabel(dev.deviceId, dev.label);
                      setSelected(topic);
                      onSelect(topic);
                    }}
                    title={dev.deviceId}
                  >
                    <span className="font-medium">{displayLabel}</span>
                  </li>
                );
              })}
            </ul>
          )}
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
