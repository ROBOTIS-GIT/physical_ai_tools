// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

export const BROWSER_CAMERA_PREFIX = 'browser-camera:';

const STORAGE_KEY = 'browserCameraLabels';

const loadMap = () => {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return {};
    const parsed = JSON.parse(raw);
    return parsed && typeof parsed === 'object' ? parsed : {};
  } catch (_) {
    return {};
  }
};

const saveMap = (map) => {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(map));
  } catch (_) {
    // ignore (quota / private mode)
  }
};

export const rememberBrowserCameraLabel = (deviceId, label) => {
  if (!deviceId || !label) return;
  const map = loadMap();
  if (map[deviceId] === label) return;
  map[deviceId] = label;
  saveMap(map);
};

export const lookupBrowserCameraLabel = (deviceId) => {
  if (!deviceId) return '';
  return loadMap()[deviceId] || '';
};

export const isBrowserCameraTopic = (topic) =>
  typeof topic === 'string' && topic.startsWith(BROWSER_CAMERA_PREFIX);

export const getBrowserCameraDeviceId = (topic) =>
  isBrowserCameraTopic(topic) ? topic.slice(BROWSER_CAMERA_PREFIX.length) : '';

export const displayLabelForTopic = (topic) => {
  if (!isBrowserCameraTopic(topic)) return topic || '';
  const deviceId = getBrowserCameraDeviceId(topic);
  const label = lookupBrowserCameraLabel(deviceId);
  if (label) return `(browser) ${label}`;
  return `(browser) ${deviceId.slice(0, 8)}…`;
};
