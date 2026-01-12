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
// Author: Dongyun Kim

import React, { useEffect, useRef, useState, useCallback, useMemo } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { MdFolder, MdPlayArrow, MdPause, MdRefresh, MdExpandMore, MdExpandLess, MdUnfoldMore, MdUnfoldLess } from 'react-icons/md';
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  ReferenceLine,
  ReferenceDot,
} from 'recharts';
import toast from 'react-hot-toast';
import clsx from 'clsx';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import FileBrowserModal from '../components/FileBrowserModal';
import {
  setSelectedBagPath,
  setLoading,
  setReplayData,
  setError,
  setCurrentTime,
  setIsPlaying,
  setIsVideoLoaded,
  setVideoLoadProgress,
} from '../features/replay/replaySlice';

// Fixed colors for state and action
const STATE_COLOR = '#dc2626';  // Red for state
const ACTION_COLOR = '#2563eb'; // Blue for action

// Individual Joint Chart Component showing both State and Action
function JointChart({ name, stateData, actionData, currentTime, isExpanded, onToggle, hasAction }) {
  // Get current state value at currentTime
  const currentStateValue = useMemo(() => {
    if (!stateData.length) return null;
    const closest = stateData.reduce((prev, curr) =>
      Math.abs(curr.time - currentTime) < Math.abs(prev.time - currentTime) ? curr : prev
    );
    return closest[`state_${name}`];
  }, [stateData, name, currentTime]);

  // Get current action value at currentTime
  const currentActionValue = useMemo(() => {
    if (!actionData.length) return null;
    const closest = actionData.reduce((prev, curr) =>
      Math.abs(curr.time - currentTime) < Math.abs(prev.time - currentTime) ? curr : prev
    );
    return closest[`action_${name}`];
  }, [actionData, name, currentTime]);

  // Merge state and action data for the chart
  const mergedData = useMemo(() => {
    const timeMap = new Map();

    // Add state data
    stateData.forEach((point) => {
      const time = point.time;
      if (!timeMap.has(time)) {
        timeMap.set(time, { time });
      }
      timeMap.get(time)[`state_${name}`] = point[`state_${name}`];
    });

    // Add action data
    actionData.forEach((point) => {
      const time = point.time;
      if (!timeMap.has(time)) {
        timeMap.set(time, { time });
      }
      timeMap.get(time)[`action_${name}`] = point[`action_${name}`];
    });

    // Sort by time and return
    return Array.from(timeMap.values()).sort((a, b) => a.time - b.time);
  }, [stateData, actionData, name]);

  return (
    <div className="border border-gray-200 rounded-lg overflow-hidden bg-white">
      {/* Header - always visible */}
      <button
        onClick={onToggle}
        className="w-full flex items-center justify-between px-3 py-2 hover:bg-gray-50 transition-colors"
      >
        <div className="flex items-center gap-2">
          <span className="font-medium text-sm text-gray-700">{name}</span>
        </div>
        <div className="flex items-center gap-3">
          {/* State value */}
          <div className="flex items-center gap-1">
            <div className="w-2 h-2 rounded-full" style={{ backgroundColor: STATE_COLOR }} />
            <span className="text-xs text-gray-500 font-mono">
              {currentStateValue !== null && currentStateValue !== undefined
                ? currentStateValue.toFixed(4)
                : '-'}
            </span>
          </div>
          {/* Action value */}
          {hasAction && (
            <div className="flex items-center gap-1">
              <div className="w-2 h-2 rounded-full" style={{ backgroundColor: ACTION_COLOR }} />
              <span className="text-xs text-gray-500 font-mono">
                {currentActionValue !== null && currentActionValue !== undefined
                  ? currentActionValue.toFixed(4)
                  : '-'}
              </span>
            </div>
          )}
          {isExpanded ? (
            <MdExpandLess className="text-gray-400" size={20} />
          ) : (
            <MdExpandMore className="text-gray-400" size={20} />
          )}
        </div>
      </button>

      {/* Chart - collapsible */}
      {isExpanded && (
        <div className="h-28 px-2 pb-2">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={mergedData} margin={{ top: 5, right: 10, left: 0, bottom: 5 }}>
              <CartesianGrid strokeDasharray="3 3" stroke="#e5e7eb" />
              <XAxis
                dataKey="time"
                tickFormatter={(value) => `${value.toFixed(0)}s`}
                stroke="#9ca3af"
                fontSize={10}
                tick={{ fill: '#9ca3af' }}
              />
              <YAxis
                stroke="#9ca3af"
                fontSize={10}
                tick={{ fill: '#9ca3af' }}
                width={45}
                tickFormatter={(value) => value.toFixed(2)}
              />
              <Tooltip
                formatter={(value, dataKey) => {
                  const label = dataKey.startsWith('state_') ? 'State' : 'Action';
                  return [value?.toFixed(4) ?? '-', label];
                }}
                labelFormatter={(label) => `Time: ${label?.toFixed(2) ?? '-'}s`}
                contentStyle={{
                  backgroundColor: 'rgba(255, 255, 255, 0.95)',
                  border: '1px solid #e5e7eb',
                  borderRadius: '6px',
                  fontSize: '11px',
                }}
              />
              <ReferenceLine
                x={currentTime}
                stroke="#22c55e"
                strokeWidth={2}
                strokeDasharray="none"
                label={{ value: '', position: 'top' }}
              />
              {/* Current position marker for State */}
              {currentStateValue !== null && currentStateValue !== undefined && (
                <ReferenceDot
                  x={currentTime}
                  y={currentStateValue}
                  r={6}
                  fill={STATE_COLOR}
                  stroke="#fff"
                  strokeWidth={2}
                />
              )}
              {/* Current position marker for Action */}
              {hasAction && currentActionValue !== null && currentActionValue !== undefined && (
                <ReferenceDot
                  x={currentTime}
                  y={currentActionValue}
                  r={6}
                  fill={ACTION_COLOR}
                  stroke="#fff"
                  strokeWidth={2}
                />
              )}
              {/* State line - Red */}
              <Line
                type="monotone"
                dataKey={`state_${name}`}
                stroke={STATE_COLOR}
                strokeWidth={1.5}
                dot={false}
                isAnimationActive={false}
                name="State"
                connectNulls
              />
              {/* Action line - Blue */}
              {hasAction && (
                <Line
                  type="monotone"
                  dataKey={`action_${name}`}
                  stroke={ACTION_COLOR}
                  strokeWidth={1.5}
                  dot={false}
                  isAnimationActive={false}
                  name="Action"
                  connectNulls
                />
              )}
            </LineChart>
          </ResponsiveContainer>
        </div>
      )}
    </div>
  );
}

function ReplayPage({ isActive }) {
  const dispatch = useDispatch();
  const { getReplayData } = useRosServiceCaller();
  const rosHost = useSelector((state) => state.ros.rosHost);

  // Redux state
  const {
    selectedBagPath,
    isLoading,
    isLoaded,
    error,
    videoFiles,
    videoServerPort,
    bagPath,
    jointTimestamps,
    jointNames,
    jointPositions,
    actionTimestamps,
    actionNames,
    actionValues,
    duration,
    currentTime,
    isPlaying,
    isVideoLoaded,
    videoLoadProgress,
  } = useSelector((state) => state.replay);

  // Local state
  const [showFileBrowser, setShowFileBrowser] = useState(false);
  const [expandedJoints, setExpandedJoints] = useState(new Set());

  // Refs for multiple videos
  const videoRefs = useRef([]);

  // Calculate video URL for a given index
  const getVideoUrl = useCallback(
    (index) => {
      if (!bagPath || !videoFiles.length) return null;
      const videoFile = videoFiles[index];
      if (!videoFile) return null;
      return `http://${rosHost}:${videoServerPort}/video/${bagPath}/${videoFile}`;
    },
    [bagPath, videoFiles, rosHost, videoServerPort]
  );

  // Get all unique joint names from both state and action
  const allJointNames = useMemo(() => {
    const names = new Set([...jointNames, ...actionNames]);
    return Array.from(names);
  }, [jointNames, actionNames]);

  // Check if we have action data
  const hasActionData = actionTimestamps.length > 0 && actionNames.length > 0;

  // Prepare state chart data - memoized for performance
  const stateChartData = useMemo(() => {
    if (!jointTimestamps.length || !jointNames.length || !jointPositions.length) {
      return [];
    }

    const numJoints = jointNames.length;
    const data = [];

    // Sample data to avoid too many points (max 500 points for performance)
    const step = Math.max(1, Math.floor(jointTimestamps.length / 500));

    for (let i = 0; i < jointTimestamps.length; i += step) {
      const point = { time: jointTimestamps[i] };
      const startIdx = i * numJoints;

      jointNames.forEach((name, j) => {
        point[`state_${name}`] = jointPositions[startIdx + j] || 0;
      });

      data.push(point);
    }

    return data;
  }, [jointTimestamps, jointNames, jointPositions]);

  // Prepare action chart data - memoized for performance
  const actionChartData = useMemo(() => {
    if (!actionTimestamps.length || !actionNames.length || !actionValues.length) {
      return [];
    }

    const numActions = actionNames.length;
    const data = [];

    // Sample data to avoid too many points (max 500 points for performance)
    const step = Math.max(1, Math.floor(actionTimestamps.length / 500));

    for (let i = 0; i < actionTimestamps.length; i += step) {
      const point = { time: actionTimestamps[i] };
      const startIdx = i * numActions;

      actionNames.forEach((name, j) => {
        point[`action_${name}`] = actionValues[startIdx + j] || 0;
      });

      data.push(point);
    }

    return data;
  }, [actionTimestamps, actionNames, actionValues]);

  // Toggle individual joint expansion
  const toggleJoint = useCallback((jointName) => {
    setExpandedJoints((prev) => {
      const newSet = new Set(prev);
      if (newSet.has(jointName)) {
        newSet.delete(jointName);
      } else {
        newSet.add(jointName);
      }
      return newSet;
    });
  }, []);

  // Expand all joints
  const expandAllJoints = useCallback(() => {
    setExpandedJoints(new Set(allJointNames));
  }, [allJointNames]);

  // Collapse all joints
  const collapseAllJoints = useCallback(() => {
    setExpandedJoints(new Set());
  }, []);

  // Handle bag selection
  const handleSelectBag = async (path) => {
    setShowFileBrowser(false);
    dispatch(setSelectedBagPath(path));
    dispatch(setLoading(true));

    try {
      const result = await getReplayData(path);
      if (result.success) {
        dispatch(setReplayData(result));
        toast.success('Replay data loaded successfully');
      } else {
        dispatch(setError(result.message));
        toast.error(`Failed to load replay data: ${result.message}`);
      }
    } catch (err) {
      dispatch(setError(err.message));
      toast.error(`Error loading replay data: ${err.message}`);
    }
  };

  // Handle video load progress (use first video as reference)
  useEffect(() => {
    const video = videoRefs.current[0];
    if (!video || !isLoaded) return;

    const handleProgress = () => {
      if (video.buffered.length > 0) {
        const bufferedEnd = video.buffered.end(video.buffered.length - 1);
        const progress = (bufferedEnd / video.duration) * 100;
        dispatch(setVideoLoadProgress(progress));

        if (bufferedEnd >= video.duration - 0.5) {
          dispatch(setIsVideoLoaded(true));
        }
      }
    };

    const handleCanPlayThrough = () => {
      dispatch(setIsVideoLoaded(true));
    };

    const handleTimeUpdate = () => {
      dispatch(setCurrentTime(video.currentTime));
    };

    const handleEnded = () => {
      dispatch(setIsPlaying(false));
    };

    video.addEventListener('progress', handleProgress);
    video.addEventListener('canplaythrough', handleCanPlayThrough);
    video.addEventListener('timeupdate', handleTimeUpdate);
    video.addEventListener('ended', handleEnded);

    return () => {
      video.removeEventListener('progress', handleProgress);
      video.removeEventListener('canplaythrough', handleCanPlayThrough);
      video.removeEventListener('timeupdate', handleTimeUpdate);
      video.removeEventListener('ended', handleEnded);
    };
  }, [isLoaded, dispatch]);

  // Handle play/pause for all videos
  const togglePlayPause = () => {
    if (isPlaying) {
      videoRefs.current.forEach((video) => {
        if (video) video.pause();
      });
      dispatch(setIsPlaying(false));
    } else {
      videoRefs.current.forEach((video) => {
        if (video) video.play();
      });
      dispatch(setIsPlaying(true));
    }
  };

  // Handle seek for all videos
  const handleSeek = (e) => {
    if (!isVideoLoaded) return;

    const rect = e.currentTarget.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const percentage = x / rect.width;
    const newTime = percentage * duration;

    videoRefs.current.forEach((video) => {
      if (video) video.currentTime = newTime;
    });
    dispatch(setCurrentTime(newTime));
  };

  // Format time as MM:SS
  const formatTime = (seconds) => {
    const mins = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
  };

  // Extract short name from video file path
  const getShortVideoName = (filePath) => {
    const fileName = filePath.split('/').pop();
    // Remove .mp4 extension and common prefixes
    return fileName
      .replace('.mp4', '')
      .replace('_compressed', '')
      .split('_')
      .slice(-3)
      .join('_');
  };

  // Reset state when leaving page
  useEffect(() => {
    if (!isActive) {
      videoRefs.current.forEach((video) => {
        if (video) video.pause();
      });
      dispatch(setIsPlaying(false));
    }
  }, [isActive, dispatch]);

  return (
    <div className="flex flex-col h-full p-6 bg-gray-50">
      {/* Header */}
      <div className="flex items-center justify-between mb-4">
        <h1 className="text-2xl font-bold text-gray-800">Replay Viewer</h1>
        <button
          onClick={() => setShowFileBrowser(true)}
          className="flex items-center gap-2 px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition-colors"
        >
          <MdFolder size={20} />
          Select ROSbag
        </button>
      </div>

      {/* Main content */}
      <div className="flex-1 flex flex-col gap-4 min-h-0">
        {/* Video panel */}
        <div className="flex-1 flex flex-col bg-white rounded-xl shadow-sm overflow-hidden min-h-0">
          {isLoaded && videoFiles.length > 0 ? (
            <>
              {/* Video container - grid layout for multiple videos */}
              <div className="flex-1 bg-gray-100 flex items-center justify-center p-4 min-h-0">
                <div
                  className={clsx(
                    'grid gap-4 w-full h-full',
                    videoFiles.length === 1 && 'grid-cols-1',
                    videoFiles.length === 2 && 'grid-cols-2',
                    videoFiles.length >= 3 && 'grid-cols-2 lg:grid-cols-3'
                  )}
                >
                  {videoFiles.map((file, index) => (
                    <div
                      key={index}
                      className="relative bg-white rounded-lg overflow-hidden shadow flex flex-col"
                    >
                      {/* Video label */}
                      <div className="absolute top-2 left-2 z-10 px-2 py-1 bg-black bg-opacity-60 text-white text-xs rounded">
                        {getShortVideoName(file)}
                      </div>
                      <video
                        ref={(el) => (videoRefs.current[index] = el)}
                        src={getVideoUrl(index)}
                        className="w-full h-full object-contain bg-gray-50"
                        preload="auto"
                        muted={index > 0} // Mute all except first video
                      />
                    </div>
                  ))}
                </div>
                {!isVideoLoaded && (
                  <div className="absolute inset-0 bg-white bg-opacity-80 flex flex-col items-center justify-center">
                    <div className="text-gray-700 mb-4">
                      Loading video... {Math.round(videoLoadProgress)}%
                    </div>
                    <div className="w-64 h-2 bg-gray-300 rounded-full overflow-hidden">
                      <div
                        className="h-full bg-blue-500 transition-all duration-300"
                        style={{ width: `${videoLoadProgress}%` }}
                      />
                    </div>
                  </div>
                )}
              </div>

              {/* Video controls */}
              <div className="p-4 bg-gray-100 border-t">
                {/* Timeline */}
                <div
                  className={clsx(
                    'h-2 bg-gray-300 rounded-full mb-4 cursor-pointer relative',
                    { 'opacity-50 cursor-not-allowed': !isVideoLoaded }
                  )}
                  onClick={isVideoLoaded ? handleSeek : undefined}
                >
                  <div
                    className="h-full bg-blue-500 rounded-full"
                    style={{ width: `${(currentTime / duration) * 100}%` }}
                  />
                  <div
                    className="absolute top-1/2 -translate-y-1/2 w-4 h-4 bg-blue-600 rounded-full shadow"
                    style={{ left: `calc(${(currentTime / duration) * 100}% - 8px)` }}
                  />
                </div>

                {/* Controls */}
                <div className="flex items-center gap-4">
                  <button
                    onClick={togglePlayPause}
                    disabled={!isVideoLoaded}
                    className={clsx(
                      'p-2 rounded-full transition-colors',
                      isVideoLoaded
                        ? 'bg-blue-600 text-white hover:bg-blue-700'
                        : 'bg-gray-300 text-gray-500 cursor-not-allowed'
                    )}
                  >
                    {isPlaying ? <MdPause size={24} /> : <MdPlayArrow size={24} />}
                  </button>
                  <span className="text-sm text-gray-600">
                    {formatTime(currentTime)} / {formatTime(duration)}
                  </span>
                  <span className="text-sm text-gray-400">
                    ({videoFiles.length} camera{videoFiles.length > 1 ? 's' : ''})
                  </span>
                </div>
              </div>
            </>
          ) : (
            <div className="flex-1 flex items-center justify-center text-gray-500 bg-gray-50">
              {isLoading ? (
                <div className="flex flex-col items-center gap-4">
                  <MdRefresh className="animate-spin" size={48} />
                  <span>Loading replay data...</span>
                </div>
              ) : error ? (
                <div className="text-center text-red-500">
                  <p className="font-semibold">Error loading data</p>
                  <p className="text-sm">{error}</p>
                </div>
              ) : (
                <div className="text-center">
                  <MdFolder size={64} className="mx-auto mb-4 text-gray-400" />
                  <p>Select a ROSbag to start viewing</p>
                </div>
              )}
            </div>
          )}
        </div>

        {/* Joint data chart panel */}
        <div className="h-96 bg-white rounded-xl shadow-sm p-4 flex flex-col">
          {/* Header with legend and expand/collapse all buttons */}
          <div className="flex items-center justify-between mb-3">
            <div className="flex items-center gap-4">
              <h2 className="text-lg font-semibold text-gray-800">Joint Data</h2>
              {/* Legend */}
              <div className="flex items-center gap-3 text-xs">
                <div className="flex items-center gap-1">
                  <div className="w-3 h-1 rounded" style={{ backgroundColor: STATE_COLOR }} />
                  <span className="text-gray-600">State</span>
                </div>
                {hasActionData && (
                  <div className="flex items-center gap-1">
                    <div className="w-3 h-1 rounded" style={{ backgroundColor: ACTION_COLOR }} />
                    <span className="text-gray-600">Action</span>
                  </div>
                )}
              </div>
            </div>
            {allJointNames.length > 0 && (
              <div className="flex items-center gap-2">
                <span className="text-xs text-gray-500">
                  {expandedJoints.size} / {allJointNames.length} expanded
                </span>
                <button
                  onClick={expandAllJoints}
                  className="flex items-center gap-1 px-2 py-1 text-xs text-gray-600 hover:bg-gray-100 rounded transition-colors"
                  title="Expand all"
                >
                  <MdUnfoldMore size={16} />
                  All
                </button>
                <button
                  onClick={collapseAllJoints}
                  className="flex items-center gap-1 px-2 py-1 text-xs text-gray-600 hover:bg-gray-100 rounded transition-colors"
                  title="Collapse all"
                >
                  <MdUnfoldLess size={16} />
                  None
                </button>
              </div>
            )}
          </div>

          {/* Joint charts grid */}
          {(stateChartData.length > 0 || actionChartData.length > 0) && allJointNames.length > 0 ? (
            <div className="flex-1 overflow-y-auto">
              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-2">
                {allJointNames.map((name) => (
                  <JointChart
                    key={name}
                    name={name}
                    stateData={stateChartData}
                    actionData={actionChartData}
                    currentTime={currentTime}
                    isExpanded={expandedJoints.has(name)}
                    onToggle={() => toggleJoint(name)}
                    hasAction={hasActionData && actionNames.includes(name)}
                  />
                ))}
              </div>
            </div>
          ) : (
            <div className="flex-1 flex items-center justify-center text-gray-400">
              No joint data available
            </div>
          )}
        </div>
      </div>

      {/* Bag info */}
      {selectedBagPath && (
        <div className="mt-2 text-sm text-gray-500">
          <span className="font-medium">Selected:</span> {selectedBagPath}
        </div>
      )}

      {/* File browser modal */}
      {showFileBrowser && (
        <FileBrowserModal
          isOpen={showFileBrowser}
          onClose={() => setShowFileBrowser(false)}
          onFileSelect={(item) => handleSelectBag(item.full_path)}
          title="Select ROSbag Directory"
          allowDirectorySelect={true}
          allowFileSelect={false}
          targetFileName="metadata.yaml"
          targetFolderName="videos"
          defaultPath="/workspace"
        />
      )}
    </div>
  );
}

export default ReplayPage;
