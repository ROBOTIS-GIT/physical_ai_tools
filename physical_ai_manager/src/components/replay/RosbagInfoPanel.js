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
import { formatFileSize, formatDateTime, formatTime } from '../../utils/chartUtils';

/**
 * RosbagInfoPanel component for displaying rosbag metadata.
 *
 * @param {Object} props
 * @param {string} props.bagPath - Path to the rosbag
 * @param {string} props.robotType - Robot type string
 * @param {string} props.recordingDate - Recording date ISO string
 * @param {number} props.fileSizeBytes - File size in bytes
 * @param {number} props.duration - Duration in seconds
 * @param {Object} props.frameCounts - Frame counts per topic
 * @param {Array<number>} props.videoFps - FPS for each video
 */
function RosbagInfoPanel({
    bagPath = '',
    robotType = '',
    recordingDate = null,
    fileSizeBytes = 0,
    duration = 0,
    frameCounts = {},
    videoFps = [],
}) {
    /**
     * Extract bag name from path
     */
    const bagName = bagPath?.split('/').pop() || 'Unknown';

    return (
        <div className="bg-white rounded-lg shadow-sm p-4 space-y-3">
            <h3 className="font-semibold text-gray-700 text-sm">Recording Info</h3>

            <div className="grid grid-cols-2 gap-2 text-sm">
                {/* Bag name */}
                <div className="text-gray-500">Name</div>
                <div className="font-medium text-gray-800 truncate" title={bagPath}>
                    {bagName}
                </div>

                {/* Robot type */}
                {robotType && (
                    <>
                        <div className="text-gray-500">Robot</div>
                        <div className="font-medium text-gray-800">{robotType}</div>
                    </>
                )}

                {/* Recording date */}
                <div className="text-gray-500">Date</div>
                <div className="font-medium text-gray-800">
                    {formatDateTime(recordingDate)}
                </div>

                {/* File size */}
                <div className="text-gray-500">Size</div>
                <div className="font-medium text-gray-800">
                    {formatFileSize(fileSizeBytes)}
                </div>

                {/* Duration */}
                <div className="text-gray-500">Duration</div>
                <div className="font-medium text-gray-800">
                    {formatTime(duration)}
                </div>

                {/* FPS */}
                {videoFps.length > 0 && (
                    <>
                        <div className="text-gray-500">FPS</div>
                        <div className="font-medium text-gray-800">
                            {videoFps[0]} fps
                        </div>
                    </>
                )}
            </div>

            {/* Frame counts */}
            {Object.keys(frameCounts).length > 0 && (
                <div className="pt-2 border-t border-gray-100">
                    <div className="text-xs text-gray-500 mb-1">Frame Counts</div>
                    <div className="space-y-1">
                        {Object.entries(frameCounts).map(([topic, count]) => (
                            <div key={topic} className="flex justify-between text-xs">
                                <span className="text-gray-500 truncate flex-1">{topic.split('/').pop()}</span>
                                <span className="font-mono text-gray-700 ml-2">{count}</span>
                            </div>
                        ))}
                    </div>
                </div>
            )}
        </div>
    );
}

export default RosbagInfoPanel;
