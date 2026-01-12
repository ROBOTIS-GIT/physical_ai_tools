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

import React, { useRef } from 'react';
import { MdPlayArrow, MdPause, MdReplay, MdSpeed } from 'react-icons/md';
import clsx from 'clsx';
import { formatTime } from '../../utils/chartUtils';

/**
 * Playback controls component for video playback.
 * Includes play/pause, progress bar, time display, and speed controls.
 *
 * @param {Object} props
 * @param {boolean} props.isPlaying - Current playing state
 * @param {number} props.currentTime - Current playback time in seconds
 * @param {number} props.duration - Total duration in seconds
 * @param {number} props.playbackSpeed - Current playback speed
 * @param {Array<number>} props.availableSpeeds - Available playback speeds
 * @param {boolean} props.isVideoLoaded - Whether video is loaded
 * @param {number|null} props.loopStart - A-B loop start point
 * @param {number|null} props.loopEnd - A-B loop end point
 * @param {Object|null} props.trimStart - Trim start point { time, frame, instruction }
 * @param {Object|null} props.trimEnd - Trim end point { time, frame }
 * @param {Array<Object>} props.taskMarkers - Task markers array
 * @param {Array<Object>} props.excludeRegions - Exclude regions array
 * @param {Object|null} props.pendingExcludeStart - Pending exclude start point
 * @param {Function} props.onTogglePlayPause - Play/pause toggle callback
 * @param {Function} props.onRestart - Restart callback
 * @param {Function} props.onSeek - Seek callback (time) => void
 * @param {Function} props.onSpeedChange - Speed change callback
 * @param {Function} props.onProgressClick - Progress bar click callback
 */
function PlaybackControls({
    isPlaying = false,
    currentTime = 0,
    duration = 0,
    playbackSpeed = 1,
    availableSpeeds = [0.5, 1, 1.5, 2, 3],
    isVideoLoaded = false,
    loopStart = null,
    loopEnd = null,
    trimStart = null,
    trimEnd = null,
    taskMarkers = [],
    excludeRegions = [],
    pendingExcludeStart = null,
    onTogglePlayPause,
    onRestart,
    onSeek,
    onSpeedChange,
}) {
    const progressBarRef = useRef(null);

    /**
     * Handle click on progress bar
     */
    const handleProgressClick = (e) => {
        if (!progressBarRef.current || !isVideoLoaded) return;

        const rect = progressBarRef.current.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const percentage = x / rect.width;
        const newTime = percentage * duration;
        onSeek?.(newTime);
    };

    /**
     * Calculate percentage for time value
     */
    const getPercentage = (time) => {
        if (duration <= 0) return 0;
        return (time / duration) * 100;
    };

    return (
        <div className="p-3 bg-gray-50 border-t border-gray-200">
            {/* Progress bar with markers */}
            <div
                ref={progressBarRef}
                className="relative h-6 bg-gray-200 rounded-lg cursor-pointer mb-2 overflow-hidden"
                onClick={handleProgressClick}
            >
                {/* Valid region (between trim points) */}
                {trimStart && (
                    <div
                        className="absolute top-0 h-full bg-green-100"
                        style={{
                            left: `${getPercentage(trimStart.time)}%`,
                            width: `${getPercentage((trimEnd?.time || duration) - trimStart.time)}%`,
                        }}
                    />
                )}

                {/* Exclude regions */}
                {excludeRegions.map((region, i) => (
                    <div
                        key={i}
                        className="absolute top-0 h-full bg-red-200 opacity-60"
                        style={{
                            left: `${getPercentage(region.start.time)}%`,
                            width: `${getPercentage(region.end.time - region.start.time)}%`,
                        }}
                        title={`Excluded: ${formatTime(region.start.time)} - ${formatTime(region.end.time)}`}
                    />
                ))}

                {/* Pending exclude start */}
                {pendingExcludeStart && (
                    <div
                        className="absolute top-0 w-1 h-full bg-red-500"
                        style={{ left: `${getPercentage(pendingExcludeStart.time)}%` }}
                        title={`Exclude start: ${formatTime(pendingExcludeStart.time)}`}
                    />
                )}

                {/* A-B Loop markers */}
                {loopStart !== null && (
                    <div
                        className="absolute top-0 w-1 h-full bg-purple-500"
                        style={{ left: `${getPercentage(loopStart)}%` }}
                        title={`A: ${formatTime(loopStart)}`}
                    />
                )}
                {loopEnd !== null && (
                    <div
                        className="absolute top-0 w-1 h-full bg-purple-500"
                        style={{ left: `${getPercentage(loopEnd)}%` }}
                        title={`B: ${formatTime(loopEnd)}`}
                    />
                )}

                {/* Task markers */}
                {taskMarkers.map((marker, i) => (
                    <div
                        key={i}
                        className="absolute top-0 w-0.5 h-full bg-yellow-500"
                        style={{ left: `${getPercentage(marker.time)}%` }}
                        title={marker.instruction}
                    />
                ))}

                {/* Trim markers */}
                {trimStart && (
                    <div
                        className="absolute top-0 w-1 h-full bg-green-600"
                        style={{ left: `${getPercentage(trimStart.time)}%` }}
                        title={`Start: ${formatTime(trimStart.time)}`}
                    />
                )}
                {trimEnd && (
                    <div
                        className="absolute top-0 w-1 h-full bg-green-600"
                        style={{ left: `${getPercentage(trimEnd.time)}%` }}
                        title={`End: ${formatTime(trimEnd.time)}`}
                    />
                )}

                {/* Progress fill */}
                <div
                    className="absolute top-0 left-0 h-full bg-blue-500 opacity-50"
                    style={{ width: `${getPercentage(currentTime)}%` }}
                />

                {/* Current position indicator */}
                <div
                    className="absolute top-0 w-1 h-full bg-blue-600"
                    style={{ left: `${getPercentage(currentTime)}%` }}
                />
            </div>

            {/* Controls row */}
            <div className="flex items-center gap-4">
                {/* Play/Pause button */}
                <button
                    onClick={onTogglePlayPause}
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

                {/* Restart button */}
                <button
                    onClick={onRestart}
                    disabled={!isVideoLoaded}
                    className={clsx(
                        'p-2 rounded-full transition-colors',
                        isVideoLoaded
                            ? 'bg-gray-200 text-gray-700 hover:bg-gray-300'
                            : 'bg-gray-100 text-gray-400 cursor-not-allowed'
                    )}
                    title="Restart"
                >
                    <MdReplay size={20} />
                </button>

                {/* Time display */}
                <div className="text-sm font-mono text-gray-600">
                    {formatTime(currentTime)} / {formatTime(duration)}
                </div>

                {/* A-B Loop indicator */}
                {(loopStart !== null || loopEnd !== null) && (
                    <div className="text-xs text-purple-600 bg-purple-100 px-2 py-1 rounded">
                        A: {loopStart !== null ? formatTime(loopStart) : '--'}
                        {' → '}
                        B: {loopEnd !== null ? formatTime(loopEnd) : '--'}
                    </div>
                )}

                {/* Spacer */}
                <div className="flex-1" />

                {/* Speed selector */}
                <div className="flex items-center gap-1">
                    <MdSpeed className="text-gray-500" size={16} />
                    <select
                        value={playbackSpeed}
                        onChange={(e) => onSpeedChange?.(parseFloat(e.target.value))}
                        className="text-sm bg-white border border-gray-300 rounded px-2 py-1"
                        disabled={!isVideoLoaded}
                    >
                        {availableSpeeds.map((speed) => (
                            <option key={speed} value={speed}>
                                {speed}x
                            </option>
                        ))}
                    </select>
                </div>
            </div>
        </div>
    );
}

export default PlaybackControls;
