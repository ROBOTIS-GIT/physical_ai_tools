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

import React from 'react';
import clsx from 'clsx';
import { MdOpenInFull } from 'react-icons/md';
import { useSelector } from 'react-redux';

const CompactSystemStatus = ({
  label = 'System',
  type = 'storage', // 'cpu', 'ram', 'storage'
  className,
  showDetails = false,
}) => {
  const cpuPercentage = useSelector((state) => state.tasks.taskStatus.usedCpu);
  const totalRamSize = useSelector((state) => state.tasks.taskStatus.totalRamSize);
  const usedRamSize = useSelector((state) => state.tasks.taskStatus.usedRamSize);
  const totalStorageSize = useSelector((state) => state.tasks.taskStatus.totalStorageSize);
  const usedStorageSize = useSelector((state) => state.tasks.taskStatus.usedStorageSize);

  const totalCapacity =
    type === 'ram' ? totalRamSize * 1024 * 1024 * 1024 : totalStorageSize * 1024 * 1024 * 1024;
  const usedCapacity =
    type === 'ram' ? usedRamSize * 1024 * 1024 * 1024 : usedStorageSize * 1024 * 1024 * 1024;

  // Calculate usage percentage
  let usagePercentage;
  if (type === 'cpu') {
    usagePercentage = cpuPercentage || 0;
  } else {
    usagePercentage = totalCapacity > 0 ? (usedCapacity / totalCapacity) * 100 : 0;
  }

  // Format bytes to human readable format
  const formatBytes = (bytes) => {
    if (bytes === 0) return '0 B';

    const k = 1024;
    const sizes = ['B', 'KB', 'MB', 'GB', 'TB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));

    return parseFloat((bytes / Math.pow(k, i)).toFixed(1)) + ' ' + sizes[i];
  };

  // Get color based on usage percentage
  const getUsageColor = (percentage) => {
    if (percentage >= 90) return 'bg-red-500';
    if (percentage >= 75) return 'bg-orange-500';
    if (percentage >= 50) return 'bg-yellow-500';
    return 'bg-green-500';
  };

  // Get ring color for circular progress
  const getRingColor = (percentage) => {
    if (percentage >= 90) return 'text-red-500';
    if (percentage >= 75) return 'text-orange-500';
    if (percentage >= 50) return 'text-yellow-500';
    return 'text-green-500';
  };

  const containerClass = clsx(
    'flex items-center space-x-3 p-2 bg-white rounded-xl border border-gray-200',
    'hover:bg-gray-50 hover:border-gray-300 hover:shadow-md transition-all duration-200',
    'cursor-pointer transform hover:scale-105',
    className
  );

  const progressRingClass = clsx('transition-all duration-300', getRingColor(usagePercentage));

  const progressBarClass = clsx(
    'h-2 rounded-full transition-all duration-300 ease-in-out',
    getUsageColor(usagePercentage)
  );

  // Calculate circle stroke-dasharray for circular progress
  const radius = 14;
  const circumference = 2 * Math.PI * radius;
  const strokeDasharray = circumference;
  const strokeDashoffset = circumference - (usagePercentage / 100) * circumference;

  // Get display text for usage info
  const getUsageText = () => {
    if (type === 'cpu') {
      return `${Math.round(usagePercentage)}%`;
    } else {
      return `${formatBytes(usedCapacity)} / ${formatBytes(totalCapacity)}`;
    }
  };

  return (
    <div className={containerClass}>
      {/* Circular Progress Indicator */}
      <div className="relative w-8 h-8 flex-shrink-0">
        <svg className="w-8 h-8 transform -rotate-90" viewBox="0 0 32 32">
          {/* Background circle */}
          <circle
            cx="16"
            cy="16"
            r={radius}
            stroke="currentColor"
            strokeWidth="3"
            fill="transparent"
            className="text-gray-200"
          />
          {/* Progress circle */}
          <circle
            cx="16"
            cy="16"
            r={radius}
            stroke="currentColor"
            strokeWidth="3"
            fill="transparent"
            strokeDasharray={strokeDasharray}
            strokeDashoffset={strokeDashoffset}
            className={progressRingClass}
            strokeLinecap="round"
          />
        </svg>
        {/* Percentage text */}
        <div className="absolute inset-0 flex items-center justify-center">
          <span className="text-xs font-medium text-gray-700" style={{ minWidth: '3ch' }}>
            {Math.round(usagePercentage)}%
          </span>
        </div>
      </div>

      {/* System Info */}
      <div className="flex-1 min-w-0">
        <div className="flex items-center justify-between mb-1">
          <span className="text-sm font-medium text-gray-700">{label}</span>
          {usagePercentage >= 90 && (
            <svg className="w-4 h-4 text-red-500" fill="currentColor" viewBox="0 0 20 20">
              <path
                fillRule="evenodd"
                d="M8.257 3.099c.765-1.36 2.722-1.36 3.486 0l5.58 9.92c.75 1.334-.213 2.98-1.742 2.98H4.42c-1.53 0-2.493-1.646-1.743-2.98l5.58-9.92zM11 13a1 1 0 11-2 0 1 1 0 012 0zm-1-8a1 1 0 00-1 1v3a1 1 0 002 0V6a1 1 0 00-1-1z"
                clipRule="evenodd"
              />
            </svg>
          )}
        </div>

        {showDetails ? (
          <div className="space-y-1">
            {type === 'cpu' ? (
              <div className="flex justify-between text-xs">
                <span className="text-gray-500">Usage:</span>
                <span
                  className="font-medium"
                  style={type === 'cpu' ? { minWidth: '3ch' } : undefined}
                >
                  {Math.round(usagePercentage)}%
                </span>
              </div>
            ) : (
              <>
                <div className="flex justify-between text-xs">
                  <span className="text-gray-500">Used:</span>
                  <span className="font-medium">{formatBytes(usedCapacity)}</span>
                </div>
                <div className="flex justify-between text-xs">
                  <span className="text-gray-500">Total:</span>
                  <span className="font-medium">{formatBytes(totalCapacity)}</span>
                </div>
              </>
            )}
          </div>
        ) : (
          <div className="flex items-center justify-between">
            <div className="w-full bg-gray-200 rounded-full h-2 mr-2">
              <div
                className={progressBarClass}
                style={{ width: `${Math.min(usagePercentage, 100)}%` }}
              ></div>
            </div>
            <span
              className="text-xs text-gray-500 whitespace-nowrap"
              style={type === 'cpu' ? { minWidth: '3ch' } : undefined}
            >
              {getUsageText()}
            </span>
          </div>
        )}
      </div>

      {/* Expand Icon */}
      <div className="flex-shrink-0 ml-2">
        <MdOpenInFull
          size={16}
          className="text-gray-400 hover:text-gray-600 transition-colors duration-200"
        />
      </div>
    </div>
  );
};

export default CompactSystemStatus;
