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

import React from 'react';
import clsx from 'clsx';

/**
 * VideoPlayer component for rendering single or multiple synchronized videos.
 * Supports grid view and expanded single video view.
 *
 * @param {Object} props
 * @param {Array<string>} props.videoUrls - URLs for each video
 * @param {Array<string>} props.videoNames - Display names for each video
 * @param {Array<string>} props.videoFiles - Video file paths (for fallback names)
 * @param {number|null} props.expandedIndex - Index of expanded video, or null for grid view
 * @param {Function} props.onExpandClick - Callback when video is clicked to expand/collapse
 * @param {Function} props.setVideoRef - Callback to set video ref: (index, element) => void
 * @param {boolean} props.isDownloading - Whether videos are still downloading
 * @param {number} props.downloadProgress - Download progress (0-100)
 * @param {string} props.loadingPhase - Current loading phase
 */
function VideoPlayer({
    videoUrls = [],
    videoNames = [],
    videoFiles = [],
    expandedIndex = null,
    onExpandClick,
    setVideoRef,
    isDownloading = false,
    downloadProgress = 0,
    loadingPhase = 'idle',
}) {
    /**
     * Extract short name from video file path
     */
    const getShortVideoName = (filePath) => {
        const fileName = filePath?.split('/').pop() || '';
        return fileName
            .replace('.mp4', '')
            .replace('_compressed', '')
            .split('_')
            .slice(-3)
            .join('_');
    };

    /**
     * Get display name for a video at index
     */
    const getDisplayName = (index) => {
        return videoNames[index] || getShortVideoName(videoFiles[index]) || `Video ${index + 1}`;
    };

    if (videoUrls.length === 0) {
        return (
            <div className="flex-1 flex items-center justify-center bg-gray-100 text-gray-500">
                No video available
            </div>
        );
    }

    // Expanded single video view
    if (expandedIndex !== null) {
        return (
            <div className="flex-1 bg-gray-100 flex items-center justify-center min-h-0 relative p-2">
                <div
                    className="relative bg-white rounded-lg overflow-hidden shadow cursor-pointer flex items-center justify-center"
                    style={{ width: '100%', height: '100%' }}
                    onClick={() => onExpandClick?.(null)}
                    title="Click to collapse"
                >
                    {/* Video label */}
                    <div className="absolute top-2 left-2 z-10 px-2 py-1 bg-black bg-opacity-60 text-white text-sm rounded font-medium">
                        {getDisplayName(expandedIndex)}
                    </div>
                    {/* Collapse hint */}
                    <div className="absolute top-2 right-2 z-10 px-2 py-1 bg-black bg-opacity-60 text-white text-xs rounded">
                        Click to collapse
                    </div>
                    {/* Loading indicator */}
                    {loadingPhase === 'streaming' && isDownloading && (
                        <div className="absolute bottom-2 right-2 z-10 px-2 py-1 bg-blue-600 bg-opacity-80 text-white text-xs rounded">
                            Buffering: {downloadProgress}%
                        </div>
                    )}
                    <video
                        ref={(el) => setVideoRef?.(expandedIndex, el)}
                        src={videoUrls[expandedIndex] || ''}
                        className="max-w-full max-h-full object-contain"
                        style={{ width: 'auto', height: '100%' }}
                        playsInline
                        muted={expandedIndex > 0}
                    />
                </div>
            </div>
        );
    }

    // Grid view
    return (
        <div className="flex-1 bg-gray-100 flex items-center justify-center min-h-0 relative p-4">
            <div
                className={clsx(
                    'grid gap-4 w-full h-full',
                    videoUrls.length === 1 && 'grid-cols-1',
                    videoUrls.length === 2 && 'grid-cols-2',
                    videoUrls.length >= 3 && 'grid-cols-2 lg:grid-cols-3'
                )}
            >
                {videoUrls.map((url, index) => (
                    <div
                        key={index}
                        className="relative bg-white rounded-lg overflow-hidden shadow flex flex-col cursor-pointer hover:ring-2 hover:ring-blue-400 transition-all"
                        onClick={() => onExpandClick?.(index)}
                        title="Click to expand"
                    >
                        {/* Video label */}
                        <div className="absolute top-2 left-2 z-10 px-2 py-1 bg-black bg-opacity-60 text-white text-xs rounded font-medium">
                            {getDisplayName(index)}
                        </div>
                        {/* Loading indicator for first video */}
                        {index === 0 && loadingPhase === 'streaming' && isDownloading && (
                            <div className="absolute bottom-2 right-2 z-10 px-2 py-1 bg-blue-600 bg-opacity-80 text-white text-xs rounded">
                                Buffering: {downloadProgress}%
                            </div>
                        )}
                        <video
                            ref={(el) => setVideoRef?.(index, el)}
                            src={url || ''}
                            className="flex-1 w-full h-full object-cover"
                            playsInline
                            muted={index > 0}
                        />
                    </div>
                ))}
            </div>
        </div>
    );
}

export default VideoPlayer;
