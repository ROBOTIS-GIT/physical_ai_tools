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

import React, { useMemo } from 'react';
import { MdExpandMore, MdExpandLess } from 'react-icons/md';
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

// Fixed colors for state and action
const STATE_COLOR = '#dc2626';  // Red for state
const ACTION_COLOR = '#2563eb'; // Blue for action

/**
 * Individual Joint Chart Component showing both State and Action.
 *
 * @param {Object} props
 * @param {string} props.name - Joint name
 * @param {Array} props.stateData - State data array with time and value
 * @param {Array} props.actionData - Action data array with time and value
 * @param {number} props.currentTime - Current playback time
 * @param {number} props.duration - Total duration
 * @param {boolean} props.isExpanded - Whether chart is expanded
 * @param {Function} props.onToggle - Toggle expand callback
 * @param {boolean} props.hasAction - Whether action data exists
 * @param {Function} props.onSeek - Seek callback when clicking on chart
 */
function JointChart({
    name,
    stateData = [],
    actionData = [],
    currentTime = 0,
    duration = 0,
    isExpanded = false,
    onToggle,
    hasAction = false,
    onSeek,
}) {
    // Get current state value at currentTime
    const currentStateValue = useMemo(() => {
        if (!stateData.length) return null;
        const closest = stateData.reduce((prev, curr) =>
            Math.abs(curr.time - currentTime) < Math.abs(prev.time - currentTime) ? curr : prev
        );
        const value = closest[`state_${name}`];
        return typeof value === 'number' ? value : null;
    }, [stateData, name, currentTime]);

    // Get current action value at currentTime
    const currentActionValue = useMemo(() => {
        if (!actionData.length) return null;
        const closest = actionData.reduce((prev, curr) =>
            Math.abs(curr.time - currentTime) < Math.abs(prev.time - currentTime) ? curr : prev
        );
        const value = closest[`action_${name}`];
        return typeof value === 'number' ? value : null;
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

    // Calculate X-axis domain to include full duration
    const xDomain = useMemo(() => {
        const maxDataTime = mergedData.length > 0
            ? Math.max(...mergedData.map(d => d.time))
            : 0;
        return [0, Math.max(maxDataTime, duration || 0)];
    }, [mergedData, duration]);

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
                        <LineChart
                            data={mergedData}
                            margin={{ top: 5, right: 10, left: 0, bottom: 5 }}
                            onClick={(e) => {
                                if (e && e.activeLabel !== undefined && onSeek) {
                                    onSeek(e.activeLabel);
                                }
                            }}
                            style={{ cursor: 'crosshair' }}
                        >
                            <CartesianGrid strokeDasharray="3 3" stroke="#e5e7eb" />
                            <XAxis
                                dataKey="time"
                                domain={xDomain}
                                type="number"
                                tickFormatter={(value) => `${value.toFixed(0)}s`}
                                stroke="#9ca3af"
                                fontSize={10}
                                tick={{ fill: '#9ca3af' }}
                                allowDataOverflow={true}
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
                            {/* State line - Red */}
                            <Line
                                type="linear"
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
                                    type="linear"
                                    dataKey={`action_${name}`}
                                    stroke={ACTION_COLOR}
                                    strokeWidth={1.5}
                                    dot={false}
                                    isAnimationActive={false}
                                    name="Action"
                                    connectNulls
                                />
                            )}
                            {/* Current time indicator line */}
                            <ReferenceLine
                                x={currentTime}
                                stroke="#22c55e"
                                strokeWidth={2}
                                strokeDasharray="none"
                            />
                            {/* Current position marker for State */}
                            {currentStateValue !== null && (
                                <ReferenceDot
                                    x={currentTime}
                                    y={currentStateValue}
                                    r={6}
                                    fill={STATE_COLOR}
                                    stroke="#fff"
                                    strokeWidth={2}
                                    isAnimationActive={false}
                                    ifOverflow="extendDomain"
                                />
                            )}
                            {/* Current position marker for Action */}
                            {hasAction && currentActionValue !== null && (
                                <ReferenceDot
                                    x={currentTime}
                                    y={currentActionValue}
                                    r={6}
                                    fill={ACTION_COLOR}
                                    stroke="#fff"
                                    strokeWidth={2}
                                    isAnimationActive={false}
                                    ifOverflow="extendDomain"
                                />
                            )}
                        </LineChart>
                    </ResponsiveContainer>
                </div>
            )}
        </div>
    );
}

export default JointChart;
