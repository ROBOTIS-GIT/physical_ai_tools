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
// Author: Seongwoo Kim

import React, { useState } from 'react';
import { useSelector } from 'react-redux';
import clsx from 'clsx';
import { MdSmartToy, MdPlayArrow, MdStop } from 'react-icons/md';

export default function SystemPage({ isActive = true }) {
  const heartbeatStatus = useSelector((state) => state.tasks.heartbeatStatus);

  // State for mode selection
  const [selectedMode, setSelectedMode] = useState('manual'); // 'manual', 'autonomous'

  const classMainContainer = 'h-full flex flex-col overflow-hidden relative';

  // Connection status styling
  const getConnectionStatusStyle = () => {
    switch (heartbeatStatus) {
      case 'connected':
        return {
          bgColor: 'bg-green-100',
          textColor: 'text-green-800',
          dotColor: 'bg-green-500',
          label: 'Connected'
        };
      case 'timeout':
        return {
          bgColor: 'bg-yellow-100',
          textColor: 'text-yellow-800',
          dotColor: 'bg-yellow-500',
          label: 'Timeout'
        };
      case 'disconnected':
      default:
        return {
          bgColor: 'bg-red-100',
          textColor: 'text-red-800',
          dotColor: 'bg-red-500',
          label: 'Disconnected'
        };
    }
  };

  const statusStyle = getConnectionStatusStyle();

  return (
    <div className={classMainContainer}>
      {/* Robot Status in top left corner */}
      <div className="absolute top-10 left-8 z-10">
        <div className={clsx(
          'flex items-center gap-3 px-4 py-3 rounded-xl border-2 shadow-md',
          statusStyle.bgColor,
        )}>
          {/* Robot Icon */}
          <div className="flex items-center gap-2">
            <MdSmartToy size={28} className={clsx('transition-colors', statusStyle.textColor)} />
            <div className="flex flex-col">
              <div className="flex items-center gap-1.5">
                <div className={clsx(
                  'w-2 h-2 rounded-full transition-colors',
                  statusStyle.dotColor
                )}></div>
                <span className={clsx('text-m font-semibold', statusStyle.textColor)}>
                  {statusStyle.label}
                </span>
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Main Content */}
      <div className="flex-1 flex min-h-0 pt-32 px-0 justify-center items-start">
        <div className="w-full max-w-8xl mx-auto px-8">
          {/* Large light gray rectangle */}
          <div className="bg-gray-100 rounded-lg p-10 shadow-sm">
            {/* Operation Title */}
            <h2 className="text-2xl font-bold text-gray-800 mb-8">Operation</h2>

            {/* Manual Mode Row */}
            <div className="bg-white rounded-lg p-8 mb-8 shadow-sm">
              <div className="flex items-start justify-between">
                <div className="flex-1 pr-8">
                  <h3 className="text-lg font-semibold text-gray-800 mb-3">Manual Mode</h3>
                  <p className="text-sm text-gray-600 leading-relaxed">
                    Activate this mode to enable direct, human control over the robot, primarily for teleoperation demonstrations and collecting new dataset records.
                  </p>
                </div>
                <div className="flex items-center gap-4 flex-shrink-0">
                  <button
                    onClick={() => setSelectedMode(selectedMode === 'manual' ? '' : 'manual')}
                    className="w-52 h-20 px-4 py-3 rounded-lg font-medium transition-colors flex items-center justify-center bg-gray-400 text-white relative"
                  >
                    {selectedMode === 'manual' ? (
                      <MdStop size={32} className="text-red-400 absolute left-4" />
                    ) : (
                      <MdPlayArrow size={32} className="text-green-400 absolute left-4" />
                    )}
                    <span>{selectedMode === 'manual' ? 'Deactivate' : 'Activate'}</span>
                  </button>
                  {/* Status Box */}
                  <div className="w-52 h-20 px-4 py-2 rounded-lg border-2 border-black bg-white flex flex-col justify-center min-w-52 min-h-20">
                    <div className="flex items-center gap-2 mb-1">
                      <div className={`w-2 h-2 rounded-full ${selectedMode === 'manual' ? 'bg-green-500' : 'bg-red-500'}`}></div>
                      <span className={`text-sm font-semibold ${selectedMode === 'manual' ? 'text-green-800' : 'text-red-800'}`}>
                        Status: {selectedMode === 'manual' ? 'Active' : 'Inactive'}
                      </span>
                    </div>
                    <div className="text-xs text-gray-600 leading-tight">
                      {selectedMode === 'manual' ? 'Manual control enabled' : 'Manual control disabled'}
                    </div>
                  </div>
                </div>
              </div>
            </div>

            {/* Autonomous Mode Row */}
            <div className="bg-white rounded-lg p-8 mb-8 shadow-sm">
              <div className="flex items-start justify-between">
                <div className="flex-1 pr-8">
                  <h3 className="text-lg font-semibold text-gray-800 mb-3">Autonomous Mode</h3>
                  <p className="text-sm text-gray-600 leading-relaxed">
                    Activate this mode to allow the robot to operate independently for demonstrations or to conduct experiments using trained AI policies.
                  </p>
                </div>
                <div className="flex items-center gap-4 flex-shrink-0">
                  <button
                    onClick={() => setSelectedMode(selectedMode === 'autonomous' ? '' : 'autonomous')}
                    className="w-52 h-20 px-4 py-3 rounded-lg font-medium transition-colors flex items-center justify-center bg-gray-400 text-white relative"
                  >
                    {selectedMode === 'autonomous' ? (
                      <MdStop size={32} className="text-red-400 absolute left-4" />
                    ) : (
                      <MdPlayArrow size={32} className="text-green-400 absolute left-4" />
                    )}
                    <span>{selectedMode === 'autonomous' ? 'Deactivate' : 'Activate'}</span>
                  </button>
                  {/* Status Box */}
                  <div className="w-52 h-20 px-4 py-2 rounded-lg border-2 border-black bg-white flex flex-col justify-center min-w-52 min-h-20">
                    <div className="flex items-center gap-2 mb-1">
                      <div className={`w-2 h-2 rounded-full ${selectedMode === 'autonomous' ? 'bg-green-500' : 'bg-red-500'}`}></div>
                      <span className={`text-sm font-semibold ${selectedMode === 'autonomous' ? 'text-green-800' : 'text-red-800'}`}>
                        Status: {selectedMode === 'autonomous' ? 'Active' : 'Inactive'}
                      </span>
                    </div>
                    <div className="text-xs text-gray-600 leading-tight">
                      {selectedMode === 'autonomous' ? 'AI policy running' : 'AI policy stopped'}
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
